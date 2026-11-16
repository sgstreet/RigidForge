"""Tests for part and link mass properties and their provenance (M1 task 7, no OCP)."""

from uuid import uuid4

import numpy as np
import pytest

from rigidforge.core.geometry import InertiaTensor, Pose
from rigidforge.core.model import (
    Density,
    InertialOverride,
    Link,
    MeasuredMass,
    MirrorPlane,
    Part,
    PartSource,
    RobotModel,
    SourceKind,
)
from rigidforge.core.transforms import quat_from_rotation
from rigidforge.physics.inertia import MassProperties, combine, solid_box, solid_cylinder
from rigidforge.physics.mass import (
    CALC_VERSION,
    InertiaMethod,
    MassPropertiesError,
    MassSourceKind,
    PartMassProperties,
    UnitMassProperties,
    geometry_cache_key,
    link_mass_properties,
    method_for,
    part_mass_properties,
)

PETG = 1270.0


def _source(**kw: object) -> PartSource:
    args: dict[str, object] = {"kind": SourceKind.STEP, "asset": "a.step", "sha256": "ab" * 32}
    args.update(kw)
    return PartSource.model_validate(args)


def _unit_box(size: tuple[float, float, float]) -> UnitMassProperties:
    v = size[0] * size[1] * size[2]
    ref = solid_box(v, size)  # mass = volume at 1 kg/m³
    return UnitMassProperties(v, ref.com, ref.inertia, InertiaMethod.BREP_EXACT)


def _rot_z(deg: float) -> np.ndarray:
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1.0]])


def test_density_scales_unit_properties() -> None:
    size = (0.1, 0.08, 0.04)
    part = Part(name="base", source=_source(), mass=Density(density=PETG))
    result = part_mass_properties(part, _unit_box(size))
    v = size[0] * size[1] * size[2]
    assert result.props.allclose(solid_box(PETG * v, size), rtol=1e-12)
    assert result.part == part.id
    prov = result.provenance
    assert prov.source is MassSourceKind.DENSITY
    assert prov.method is InertiaMethod.BREP_EXACT
    assert prov.density == PETG
    assert prov.volume == pytest.approx(v)
    assert prov.calc_version == CALC_VERSION


def test_part_override_replaces_geometry_and_is_rotated() -> None:
    tensor = InertiaTensor(ixx=1e-5, iyy=2e-5, izz=2.5e-5)
    rot = _rot_z(90.0)
    ov = InertialOverride(
        mass=0.018,
        com=(0.001, 0.0, -0.002),
        inertia=tensor,
        orientation=quat_from_rotation(rot),
        source="datasheet",
    )
    part = Part(name="servo", source=_source(), inertial_override=ov)
    result = part_mass_properties(part, None)
    assert result.props.mass == 0.018
    np.testing.assert_allclose(result.props.com, (0.001, 0.0, -0.002))
    np.testing.assert_allclose(np.diag(result.props.inertia), (2e-5, 1e-5, 2.5e-5), atol=1e-18)
    assert result.provenance.source is MassSourceKind.OVERRIDE
    assert result.provenance.method is InertiaMethod.OVERRIDE
    assert result.provenance.note == "datasheet"
    assert result.provenance.density is None and result.provenance.volume is None


@pytest.mark.parametrize(
    ("part", "unit", "message"),
    [
        (Part(name="p", source=_source()), _unit_box((1, 1, 1)), "no mass source"),
        (Part(name="p", source=_source(), mass=Density(density=PETG)), None, "no geometry"),
        (
            Part(name="p", source=_source(), mass=Density(density=0.0)),
            _unit_box((1, 1, 1)),
            "density",
        ),
        (
            Part(name="p", source=_source(), mass=MeasuredMass(mass=0.005)),
            _unit_box((1, 1, 1)),
            "task 17",
        ),
        (
            Part(name="p", source=_source(), mass=Density(density=PETG)),
            UnitMassProperties(0.0, np.zeros(3), np.eye(3), InertiaMethod.BREP_EXACT),
            "volume",
        ),
        (
            Part(
                name="p",
                source=_source(),
                inertial_override=InertialOverride(
                    mass=1.0, com=(0, 0, 0), inertia=InertiaTensor(ixx=1, iyy=1, izz=3)
                ),
            ),
            None,
            "triangle_inequality",
        ),
    ],
)
def test_part_errors(part: Part, unit: UnitMassProperties | None, message: str) -> None:
    with pytest.raises(MassPropertiesError, match=message):
        part_mass_properties(part, unit)


def _two_part_model() -> tuple[RobotModel, Link, list[Part]]:
    link = Link(name="arm")
    rot = _rot_z(30.0)
    parts = [
        Part(name="a", source=_source(), link=link.id, mass=Density(density=PETG)),
        Part(
            name="b",
            source=_source(assembly_path=["b_1"]),
            link=link.id,
            placement=Pose(xyz=(0.0, 0.0, 0.1), quat=quat_from_rotation(rot)),
            mass=Density(density=PETG),
        ),
        Part(name="loose", source=_source(assembly_path=["c_1"])),
    ]
    return RobotModel(name="r", links=[link], parts=parts), link, parts


def test_link_merges_placed_parts() -> None:
    model, link, parts = _two_part_model()
    unit = _unit_box((0.02, 0.01, 0.03))
    results = {p.id: part_mass_properties(p, unit) for p in parts[:2]}
    got = link_mass_properties(model, link.id, results)
    single = unit.at_density(PETG)
    expected = combine([single, single.transformed(_rot_z(30.0), np.array([0.0, 0.0, 0.1]))])
    assert got.props.allclose(expected, rtol=1e-12, atol=1e-18)
    assert got.props.mass == pytest.approx(2 * PETG * unit.volume)
    np.testing.assert_allclose(got.props.com, (0.0, 0.0, 0.05), atol=1e-15)
    assert [r.part for r in got.parts] == [parts[0].id, parts[1].id]
    assert got.override is None


def test_link_override_wins() -> None:
    model, link, _ = _two_part_model()
    link.inertial_override = InertialOverride(
        mass=0.2, com=(0, 0, 0.01), inertia=InertiaTensor(ixx=1e-4, iyy=1e-4, izz=1e-4)
    )
    got = link_mass_properties(model, link.id, {})
    assert got.props.mass == 0.2
    assert got.parts == ()
    assert got.override is not None and got.override.source is MassSourceKind.OVERRIDE


def test_link_errors() -> None:
    model, link, parts = _two_part_model()
    with pytest.raises(MassPropertiesError, match="no mass properties for part 'b'"):
        unit = _unit_box((0.02, 0.01, 0.03))
        link_mass_properties(model, link.id, {parts[0].id: part_mass_properties(parts[0], unit)})
    empty = Link(name="empty")
    model.links.append(empty)
    with pytest.raises(MassPropertiesError, match="no parts"):
        link_mass_properties(model, empty.id, {})
    with pytest.raises(KeyError):
        link_mass_properties(model, uuid4(), {})


def test_link_merge_matches_compound_cylinder() -> None:
    """Two half-length cylinders stacked along z equal one full cylinder."""
    r, h = 0.01, 0.15
    half = solid_cylinder(np.pi * r * r * h / 2, r, h / 2)
    unit = UnitMassProperties(half.mass, half.com, half.inertia, InertiaMethod.BREP_EXACT)
    link = Link(name="rod")
    parts = [
        Part(
            name=f"half{i}",
            source=_source(),
            link=link.id,
            placement=Pose(xyz=(0.0, 0.0, z)),
            mass=Density(density=PETG),
        )
        for i, z in enumerate((-h / 4, h / 4))
    ]
    model = RobotModel(name="r", links=[link], parts=parts)
    got = link_mass_properties(model, link.id, {p.id: part_mass_properties(p, unit) for p in parts})
    full = solid_cylinder(PETG * np.pi * r * r * h, r, h)
    assert got.props.allclose(full, rtol=1e-12, atol=1e-18)


def test_part_mass_properties_type() -> None:
    part = Part(name="p", source=_source(), mass=Density(density=1.0))
    result = part_mass_properties(part, _unit_box((1.0, 1.0, 1.0)))
    assert isinstance(result, PartMassProperties)
    assert isinstance(result.props, MassProperties)


def test_method_for_source_kind() -> None:
    assert method_for(_source()) is InertiaMethod.BREP_EXACT
    assert method_for(_source(kind=SourceKind.MESH, asset="a.stl")) is InertiaMethod.MESH


def test_geometry_cache_key() -> None:
    base = _source(assembly_path=["x_1"], scale_to_si=0.001)
    key = geometry_cache_key(base)
    assert len(key) == 64
    assert key == geometry_cache_key(_source(assembly_path=["x_1"], scale_to_si=0.001))
    # Unrelated metadata does not change it.
    assert key == geometry_cache_key(base.model_copy(update={"original_path": "/elsewhere"}))
    changed = [
        _source(assembly_path=["x_2"], scale_to_si=0.001),
        _source(assembly_path=["x_1"], scale_to_si=0.0254),
        _source(assembly_path=["x_1"], scale_to_si=0.001, sha256="cd" * 32),
        _source(kind=SourceKind.MESH, assembly_path=["x_1"], scale_to_si=0.001),
    ]
    keys = {geometry_cache_key(s) for s in changed}
    assert key not in keys and len(keys) == len(changed)
    assert geometry_cache_key(base, MirrorPlane(normal=(0.0, 1.0, 0.0))) != key
