"""Part and link mass properties with provenance (M1 task 7, design doc section 6, item 3).

Mass properties are derived data. The model stores inputs only (geometry source and scale,
mass source, overrides); this module turns them into SI mass, CoM and inertia, and records
how each number was obtained so the UI and exporters can show it.

The pipeline for one part:

1. Geometry gives :class:`UnitMassProperties`: volume, CoM and the inertia tensor about the
   CoM at unit density (1 kg/m³), in the part file's frame and in metres. The B-Rep path is
   ``rigidforge.physics.brep``; the mesh path comes with the mesh importer (task 8). The
   geometry has already been scaled to SI at the import boundary, so nothing here rescales a
   tensor for units.
2. The part's mass source scales the unit properties: at fixed shape, mass and inertia are
   both linear in density, and the CoM does not depend on it.
3. An inertial override, if present, replaces geometry entirely.

A link's mass properties are its override, or else its parts moved into the link frame by
their placements and merged about the combined CoM (parallel-axis theorem).

Unit properties do not depend on density, so they are cached by :func:`geometry_cache_key`
(source hash, assembly path, scale, mirror, method and :data:`CALC_VERSION`) and the density
is applied afterwards, which is cheap.
"""

import hashlib
import json
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from uuid import UUID

import numpy as np
import numpy.typing as npt

from rigidforge.core.model import (
    Density,
    InertialOverride,
    MeasuredMass,
    MirrorPlane,
    Part,
    PartSource,
    RobotModel,
    SourceKind,
)
from rigidforge.core.transforms import matrix_from_pose
from rigidforge.physics.inertia import (
    InertiaProblem,
    MassProperties,
    combine,
    quat_to_matrix,
    rotate_inertia,
)

__all__ = [
    "CALC_VERSION",
    "InertiaMethod",
    "LinkMassProperties",
    "MassPropertiesError",
    "MassProvenance",
    "MassSourceKind",
    "PartMassProperties",
    "UnitMassProperties",
    "geometry_cache_key",
    "link_mass_properties",
    "method_for",
    "override_mass_properties",
    "part_mass_properties",
]

CALC_VERSION = 1
"""Version of the mass-property calculation. Bump it when results change, so caches expire."""


class MassPropertiesError(ValueError):
    """Mass properties cannot be computed, or the result is not physically valid."""


class InertiaMethod(StrEnum):
    """How the inertia was obtained. Values are stable identifiers."""

    BREP_EXACT = "brep_exact"
    """OCCT ``GProp`` volume integration over the exact B-Rep surfaces."""
    MESH = "mesh"
    """Integration over a closed triangle mesh (about 1e-3 relative error on curved parts)."""
    OVERRIDE = "override"
    """Given directly, for example from a datasheet."""


class MassSourceKind(StrEnum):
    """Where the mass came from. Values are stable identifiers."""

    DENSITY = "density"
    MEASURED_MASS = "measured_mass"
    OVERRIDE = "override"


@dataclass(frozen=True, slots=True, eq=False)
class UnitMassProperties:
    """Volume (m³), CoM (m) and inertia about the CoM at 1 kg/m³ (m⁵), in the part frame."""

    volume: float
    com: npt.NDArray[np.float64]
    inertia: npt.NDArray[np.float64]
    method: InertiaMethod

    def at_density(self, density: float) -> MassProperties:
        """Mass properties at a uniform density (kg/m³)."""
        return MassProperties.of(density * self.volume, self.com, density * self.inertia)


@dataclass(frozen=True, slots=True)
class MassProvenance:
    """How a part's mass properties were obtained.

    ``density`` is the density actually applied (kg/m³); for a measured mass it is the
    effective density. ``volume`` is the geometric volume (m³). Both are ``None`` for an
    override, which does not use geometry.
    """

    source: MassSourceKind
    method: InertiaMethod
    density: float | None
    volume: float | None
    calc_version: int = CALC_VERSION
    note: str | None = None
    """For an override, the free-text source recorded with it."""


@dataclass(frozen=True, slots=True, eq=False)
class PartMassProperties:
    """A part's mass properties in its own file frame, with provenance."""

    part: UUID
    props: MassProperties
    provenance: MassProvenance


@dataclass(frozen=True, slots=True, eq=False)
class LinkMassProperties:
    """A link's mass properties in the link frame.

    ``parts`` lists the contributing parts in model order; it is empty when the link's own
    override was used, in which case ``override`` holds its provenance.
    """

    link: UUID
    props: MassProperties
    parts: tuple[PartMassProperties, ...]
    override: MassProvenance | None = None


def method_for(source: PartSource) -> InertiaMethod:
    """The inertia method used for a part source: exact B-Rep for STEP, mesh otherwise."""
    return InertiaMethod.BREP_EXACT if source.kind == SourceKind.STEP else InertiaMethod.MESH


def geometry_cache_key(source: PartSource, mirror: MirrorPlane | None = None) -> str:
    """A hex sha256 identifying a part's unit mass properties.

    Covers the file content hash, the path inside an assembly, the scale to SI, a mirror
    plane, the method and :data:`CALC_VERSION`. Placement and density are not included: unit
    properties are in the part frame and independent of density.
    """
    payload = {
        "assembly_path": source.assembly_path,
        "calc_version": CALC_VERSION,
        "kind": str(source.kind),
        "method": str(method_for(source)),
        "mirror": None if mirror is None else mirror.model_dump(mode="json"),
        "scale_to_si": float(source.scale_to_si).hex(),
        "sha256": source.sha256,
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode()).hexdigest()


def override_mass_properties(override: InertialOverride) -> MassProperties:
    """An override as mass properties in its owner's frame (tensor rotated into that frame)."""
    rot = quat_to_matrix(override.orientation)
    return MassProperties.of(override.mass, override.com, rotate_inertia(override.inertia, rot))


def _checked(props: MassProperties, what: str) -> MassProperties:
    errors = [p for p in props.problems() if p is not InertiaProblem.EXTREME_RATIO]
    if errors:
        raise MassPropertiesError(f"{what}: {', '.join(errors)}")
    return props


def part_mass_properties(part: Part, unit: UnitMassProperties | None) -> PartMassProperties:
    """Mass properties of ``part`` in its file frame, from its override or its mass source.

    ``unit`` is the part's unit-density geometry result (with any mirror already baked into
    the geometry); it may be ``None`` only when the part has an inertial override. Raises
    :class:`MassPropertiesError` when the inputs are missing or the result is invalid.
    """
    what = f"part {part.name!r} ({part.id})"
    if part.inertial_override is not None:
        ov = part.inertial_override
        props = _checked(override_mass_properties(ov), what)
        prov = MassProvenance(
            MassSourceKind.OVERRIDE, InertiaMethod.OVERRIDE, None, None, note=ov.source
        )
        return PartMassProperties(part.id, props, prov)

    if unit is None:
        raise MassPropertiesError(f"{what}: no geometry mass properties and no override")
    if not (np.isfinite(unit.volume) and unit.volume > 0.0):
        raise MassPropertiesError(f"{what}: geometry volume {unit.volume} is not positive")

    match part.mass:
        case None:
            raise MassPropertiesError(f"{what}: no mass source (density or measured mass)")
        case Density(density=density):
            if not (np.isfinite(density) and density > 0.0):
                raise MassPropertiesError(f"{what}: density {density} is not positive")
            props = _checked(unit.at_density(density), what)
            prov = MassProvenance(MassSourceKind.DENSITY, unit.method, density, unit.volume)
            return PartMassProperties(part.id, props, prov)
        case MeasuredMass(mass=measured):
            if not (np.isfinite(measured) and measured > 0.0):
                raise MassPropertiesError(f"{what}: measured mass {measured} is not positive")
            # Effective density that makes the computed mass match; inertia scales with it
            # at fixed shape and the CoM comes from geometry.
            density = measured / unit.volume
            props = _checked(unit.at_density(density), what)
            prov = MassProvenance(MassSourceKind.MEASURED_MASS, unit.method, density, unit.volume)
            return PartMassProperties(part.id, props, prov)


def link_mass_properties(
    model: RobotModel, link_id: UUID, parts: Mapping[UUID, PartMassProperties]
) -> LinkMassProperties:
    """Mass properties of a link in its own frame.

    Uses the link's inertial override if it has one. Otherwise each part assigned to the
    link is moved into the link frame by its placement and the parts are merged about the
    combined CoM. ``parts`` maps part IDs to their results from :func:`part_mass_properties`.
    """
    link = model.link(link_id)
    what = f"link {link.name!r} ({link.id})"
    if link.inertial_override is not None:
        ov = link.inertial_override
        props = _checked(override_mass_properties(ov), what)
        prov = MassProvenance(
            MassSourceKind.OVERRIDE, InertiaMethod.OVERRIDE, None, None, note=ov.source
        )
        return LinkMassProperties(link.id, props, (), prov)

    members = model.parts_of(link_id)
    if not members:
        raise MassPropertiesError(f"{what}: no parts and no inertial override")
    used: list[PartMassProperties] = []
    placed: list[MassProperties] = []
    for part in members:
        result = parts.get(part.id)
        if result is None:
            raise MassPropertiesError(f"{what}: no mass properties for part {part.name!r}")
        m = matrix_from_pose(part.placement)
        placed.append(result.props.transformed(m[:3, :3], m[:3, 3]))
        used.append(result)
    return LinkMassProperties(link.id, _checked(combine(placed), what), tuple(used))
