"""Import, save and export wiring shared by the CLI and scripts (M1 task 12).

Headless: nothing here imports Qt, pyvista or VTK. The steps are

1. :func:`build_project` reads a STEP or mesh file, builds a tree model from a
   :class:`BuildSpec` (or a default star of fixed joints), and saves a ``*.rforge`` project
   with the source file copied into ``assets/``;
2. :func:`compute_link_masses` recomputes every link's mass properties from the project's own
   assets (derived data is never stored);
3. :func:`export_project` writes URDF and MJCF from the model and those masses.

Exports depend only on the saved model and the copied assets, so a project exported after a
reload gives the same bytes as the one exported straight after the build.

Frames: a spec places joints in the import root frame ("world"). The root link's frame is the
import frame; every other link's frame is its joint frame at zero position.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal
from uuid import UUID

import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, ConfigDict, Field

from rigidforge.core.extensions import Extensions, MjcfExtension, MjcfPositionActuator
from rigidforge.core.geometry import Pose, Quat, Vec3
from rigidforge.core.model import (
    Density,
    Joint,
    JointDynamics,
    JointLimits,
    JointType,
    Link,
    Massless,
    Part,
    RobotModel,
    SourceKind,
)
from rigidforge.core.project import (
    ProjectError,
    add_asset,
    asset_path,
    atomic_write_bytes,
    save_project,
)
from rigidforge.core.transforms import matrix_from_pose, pose_from_matrix
from rigidforge.core.validation import validate
from rigidforge.exporters.mjcf import export_mjcf
from rigidforge.exporters.urdf import export_urdf
from rigidforge.importers import mesh as mesh_importer
from rigidforge.importers import step as step_importer
from rigidforge.physics.brep import brep_unit_mass_properties
from rigidforge.physics.inertia import MassProperties
from rigidforge.physics.mass import (
    MassPropertiesError,
    PartMassProperties,
    UnitMassProperties,
    link_mass_properties,
    part_mass_properties,
)
from rigidforge.physics.mesh import mesh_unit_mass_properties
from rigidforge.physics.mirror import MirrorError, mirror_mesh, mirror_shape

__all__ = [
    "DEFAULT_DENSITY",
    "BuildError",
    "BuildSpec",
    "ExportResult",
    "build_model",
    "build_project",
    "compute_link_masses",
    "export_project",
    "load_spec",
]

DEFAULT_DENSITY = 1000.0
"""kg/m³ used when a spec gives none. A placeholder; real designs set density or mass."""

STEP_SUFFIXES = frozenset({".step", ".stp"})


class BuildError(Exception):
    """The input, the spec or the model cannot be turned into a project or an export."""


# --- Spec --------------------------------------------------------------------------------


class _Spec(BaseModel):
    model_config = ConfigDict(extra="forbid")


class LinkSpec(_Spec):
    name: str
    parts: list[str]
    """Leaf names: the instance path joined by ``/`` for STEP, the object name for a mesh."""


class PoseSpec(_Spec):
    xyz: Vec3 = (0.0, 0.0, 0.0)
    quat: Quat = (1.0, 0.0, 0.0, 0.0)


class JointSpec(_Spec):
    name: str
    type: JointType
    parent: str
    child: str
    origin: PoseSpec = Field(default_factory=PoseSpec)
    """Joint frame in the import root frame (metres)."""
    axis: Vec3 = (0.0, 0.0, 1.0)
    """Axis in the joint frame."""
    limits: JointLimits | None = None
    dynamics: JointDynamics | None = None
    position_actuator: MjcfPositionActuator | None = None


class BuildSpec(_Spec):
    """How to turn an imported file into a robot.

    The first link is the root. Every leaf must be assigned to exactly one link. Without
    links and joints, :func:`build_model` makes one link per leaf joined by fixed joints.
    """

    name: str | None = None
    scale_to_si: float | None = None
    density: float = DEFAULT_DENSITY
    densities: dict[str, float] = Field(default_factory=dict[str, float])
    """Per-leaf density overrides (kg/m³), keyed by leaf name."""
    links: list[LinkSpec] = Field(default_factory=list[LinkSpec])
    joints: list[JointSpec] = Field(default_factory=list[JointSpec])


def load_spec(path: Path) -> BuildSpec:
    """Read a JSON build spec. Raises :class:`BuildError` for an unreadable or invalid file."""
    try:
        return BuildSpec.model_validate_json(path.read_bytes())
    except (OSError, ValueError) as exc:
        raise BuildError(f"cannot read spec {path}: {exc}") from exc


# --- Build -------------------------------------------------------------------------------


@dataclass(frozen=True)
class _Leaf:
    name: str
    part: Part
    """The unassigned candidate part, placed in the import frame."""


def _read_leaves(source: Path, scale_to_si: float | None, asset: str, sha: str) -> list[_Leaf]:
    suffix = source.suffix.lower()
    try:
        if suffix in STEP_SUFFIXES:
            imported = step_importer.read_step(source, scale_to_si=scale_to_si)
            parts = step_importer.candidate_parts(imported, asset, sha, str(source.resolve()))
            names = ["/".join(leaf.path) or imported.root_name for leaf in imported.leaves]
        elif suffix in mesh_importer.MESH_SUFFIXES:
            mesh = mesh_importer.read_mesh(source, scale_to_si=scale_to_si)
            parts = mesh_importer.candidate_parts(mesh, asset, sha, str(source.resolve()))
            names = [obj.name for obj in mesh.objects]
        else:
            raise BuildError(f"unsupported input format {suffix!r}")
    except (step_importer.StepImportError, mesh_importer.MeshImportError, MirrorError) as exc:
        raise BuildError(str(exc)) from exc
    leaves = [_Leaf(n, p) for n, p in zip(names, parts, strict=True)]
    if len({n for n in names}) != len(names):
        raise BuildError(f"{source}: leaf names are not unique: {names}")
    return leaves


def _default_structure(leaves: Sequence[_Leaf]) -> tuple[list[LinkSpec], list[JointSpec]]:
    links = [LinkSpec(name=leaf.name.replace("/", "_"), parts=[leaf.name]) for leaf in leaves]
    if len({link.name for link in links}) != len(links):
        raise BuildError("leaf names collide once '/' is replaced; pass a spec")
    joints: list[JointSpec] = []
    for leaf, link in zip(leaves[1:], links[1:], strict=True):
        joints.append(
            JointSpec(
                name=f"{links[0].name}_to_{link.name}",
                type=JointType.FIXED,
                parent=links[0].name,
                child=link.name,
                origin=PoseSpec(xyz=leaf.part.placement.xyz, quat=leaf.part.placement.quat),
            )
        )
    return links, joints


def _world_frames(
    links: Sequence[LinkSpec], joints: Sequence[JointSpec]
) -> dict[str, NDArray[np.float64]]:
    """World (import frame) matrix of each link: identity for the root, joint frame otherwise."""
    if not links:
        raise BuildError("spec has no links")
    names = [link.name for link in links]
    if len(set(names)) != len(names):
        raise BuildError(f"duplicate link names in spec: {names}")
    world: dict[str, NDArray[np.float64]] = {links[0].name: np.eye(4)}
    pending = list(joints)
    while pending:
        progressed = False
        for js in list(pending):
            if js.parent in world and js.child not in world:
                world[js.child] = matrix_from_pose(Pose(xyz=js.origin.xyz, quat=js.origin.quat))
                pending.remove(js)
                progressed = True
        if not progressed:
            bad = ", ".join(j.name for j in pending)
            raise BuildError(
                f"joints not reachable from root link {links[0].name!r} (or child twice): {bad}"
            )
    missing = [n for n in names if n not in world]
    if missing:
        raise BuildError(f"links not connected to the root by a joint: {missing}")
    return world


def build_model(leaves: Sequence[_Leaf], spec: BuildSpec, fallback_name: str) -> RobotModel:
    """Turn imported leaves and a spec into a model. Raises :class:`BuildError` on bad specs."""
    if spec.joints and not spec.links:
        raise BuildError("a spec with joints needs links")
    if spec.links:
        links_spec, joints_spec = list(spec.links), list(spec.joints)
    else:
        links_spec, joints_spec = _default_structure(leaves)
    world = _world_frames(links_spec, joints_spec)

    by_name = {leaf.name: leaf for leaf in leaves}
    owner: dict[str, str] = {}
    for ls in links_spec:
        for leaf_name in ls.parts:
            if leaf_name not in by_name:
                raise BuildError(f"link {ls.name!r}: no such part {leaf_name!r} in the file")
            if leaf_name in owner:
                raise BuildError(
                    f"part {leaf_name!r} is in both {owner[leaf_name]!r} and {ls.name!r}"
                )
            owner[leaf_name] = ls.name
    unassigned = [n for n in by_name if n not in owner]
    if unassigned:
        raise BuildError(f"parts not assigned to any link: {unassigned}")
    unknown = [n for n in spec.densities if n not in by_name]
    if unknown:
        raise BuildError(f"densities given for unknown parts: {unknown}")

    links = {ls.name: Link(name=ls.name) for ls in links_spec}
    parts: list[Part] = []
    for leaf in leaves:
        link_name = owner[leaf.name]
        in_world = matrix_from_pose(leaf.part.placement)
        part = leaf.part.model_copy(deep=True)
        part.name = leaf.name
        part.link = links[link_name].id
        part.placement = pose_from_matrix(np.linalg.inv(world[link_name]) @ in_world)
        part.mass = Density(density=spec.densities.get(leaf.name, spec.density))
        parts.append(part)

    joints: list[Joint] = []
    for js in joints_spec:
        if js.parent not in links or js.child not in links:
            raise BuildError(f"joint {js.name!r}: unknown parent or child link")
        in_parent = np.linalg.inv(world[js.parent]) @ world[js.child]
        extensions = Extensions()
        if js.position_actuator is not None:
            extensions = Extensions(mjcf=MjcfExtension(position_actuator=js.position_actuator))
        joints.append(
            Joint(
                name=js.name,
                type=js.type,
                parent=links[js.parent].id,
                child=links[js.child].id,
                origin=pose_from_matrix(in_parent),
                axis=js.axis,
                limits=js.limits,
                dynamics=js.dynamics,
                extensions=extensions,
            )
        )
    return RobotModel(
        name=spec.name or fallback_name,
        links=[links[ls.name] for ls in links_spec],
        joints=joints,
        parts=parts,
    )


def build_project(source: Path, project: Path, spec: BuildSpec | None = None) -> RobotModel:
    """Import ``source``, build the model, and save it (with the source in ``assets/``).

    Raises :class:`BuildError` for input or spec problems, and for a model that fails
    validation (nothing is saved then). Mass problems surface at export time.
    """
    spec = spec or BuildSpec()
    if not source.is_file():
        raise BuildError(f"no such file: {source}")
    project.mkdir(parents=True, exist_ok=True)
    asset, sha = add_asset(project, source)
    leaves = _read_leaves(source, spec.scale_to_si, asset, sha)
    model = build_model(leaves, spec, fallback_name=_safe_name(source.stem))
    report = validate(model)
    if not report.ok:
        raise BuildError(
            "model is not valid: " + "; ".join(f"{i.code}: {i.message}" for i in report.errors)
        )
    save_project(model, project)
    return model


def _safe_name(stem: str) -> str:
    cleaned = "".join(c if c.isalnum() or c in "_-" else "_" for c in stem).strip("_")
    return cleaned or "robot"


# --- Masses ------------------------------------------------------------------------------


def compute_link_masses(model: RobotModel, project: Path) -> dict[UUID, MassProperties]:
    """Mass properties of every link in its own frame, from the project's assets.

    Raises :class:`BuildError` for a missing asset, a part that cannot be found in its file,
    or invalid mass properties.
    """
    steps: dict[tuple[str, float], Any] = {}
    meshes: dict[tuple[str, float], Any] = {}
    part_props: dict[UUID, PartMassProperties] = {}
    try:
        for part in model.parts:
            if part.link is None:
                continue
            if isinstance(part.mass, Massless) and part.inertial_override is None:
                continue
            unit: UnitMassProperties | None = None
            if part.inertial_override is None:
                unit = _unit_properties(part, project, steps, meshes)
            part_props[part.id] = part_mass_properties(part, unit)
        return {
            link.id: link_mass_properties(model, link.id, part_props).props for link in model.links
        }
    except MassPropertiesError as exc:
        raise BuildError(str(exc)) from exc


def _unit_properties(
    part: Part,
    project: Path,
    steps: dict[tuple[str, float], Any],
    meshes: dict[tuple[str, float], Any],
) -> UnitMassProperties:
    src = part.source
    try:
        file = asset_path(project, src.asset)
    except ProjectError as exc:
        raise BuildError(f"part {part.name!r}: {exc}") from exc
    if not file.is_file():
        raise BuildError(f"part {part.name!r}: asset {src.asset!r} is missing from {project}")
    key = (src.asset, src.scale_to_si)
    try:
        if src.kind is SourceKind.STEP:
            if key not in steps:
                steps[key] = step_importer.read_step(file, scale_to_si=src.scale_to_si)
            leaf = steps[key].leaf_at(src.assembly_path)
            shape = steps[key].product_of(leaf).shape
            if part.mirror is not None:
                shape = mirror_shape(shape, part.mirror)
            return brep_unit_mass_properties(shape, part.name)
        if key not in meshes:
            meshes[key] = mesh_importer.read_mesh(file, scale_to_si=src.scale_to_si)
        obj = meshes[key].object_named(src.assembly_path[0])
        vertices, faces = obj.vertices, obj.faces
        if part.mirror is not None:
            vertices, faces = mirror_mesh(vertices, faces, part.mirror)
        return mesh_unit_mass_properties(vertices, faces, part.name)
    except (step_importer.StepImportError, mesh_importer.MeshImportError, MirrorError) as exc:
        raise BuildError(f"part {part.name!r}: {exc}") from exc
    except (KeyError, IndexError) as exc:
        raise BuildError(
            f"part {part.name!r}: {src.assembly_path} not found in {src.asset!r}"
        ) from exc


# --- Export ------------------------------------------------------------------------------

FORMATS = ("urdf", "mjcf")


@dataclass(frozen=True)
class ExportResult:
    written: tuple[Path, ...]
    warnings: tuple[str, ...]


def export_project(
    model: RobotModel, project: Path, out: Path, formats: Sequence[Literal["urdf", "mjcf"]]
) -> ExportResult:
    """Write the chosen formats into ``out`` and return the files and warnings.

    Only the model and the project's assets are read. Raises :class:`BuildError` when the
    masses cannot be computed or an exporter rejects the model.
    """
    from rigidforge.exporters.mjcf import MjcfExportError
    from rigidforge.exporters.urdf import UrdfExportError

    masses = compute_link_masses(model, project)
    files: dict[str, bytes] = {}
    warnings: list[str] = []
    try:
        if "urdf" in formats:
            urdf = export_urdf(model, masses)
            files.update(urdf.files)
            warnings.extend(urdf.warnings)
        if "mjcf" in formats:
            mjcf = export_mjcf(model, masses)
            files.update(mjcf.files)
            warnings.extend(mjcf.warnings)
    except (UrdfExportError, MjcfExportError) as exc:
        raise BuildError(str(exc)) from exc
    out.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for name in sorted(files):
        target = out / name
        atomic_write_bytes(target, files[name])
        written.append(target)
    return ExportResult(tuple(written), tuple(warnings))
