"""Tests for instancing an imported servo assembly per link (M1 task 16)."""

import math
from pathlib import Path
from typing import Any
from uuid import UUID

import numpy as np
import pytest

from rigidforge.core.geometry import InertiaTensor, Pose
from rigidforge.core.instancing import instantiate_assembly
from rigidforge.core.model import (
    Density,
    InertialOverride,
    Link,
    Massless,
    MirrorPlane,
    Part,
    RobotModel,
)
from rigidforge.core.project import add_asset
from rigidforge.core.transforms import matrix_from_pose, pose_from_matrix
from rigidforge.importers.step import candidate_parts, read_step
from rigidforge.physics.brep import brep_unit_mass_properties
from rigidforge.physics.inertia import MassProperties, combine
from rigidforge.pipeline import compute_link_masses

HEXAPOD = Path(__file__).parent.parent / "data" / "hexapod-leg"
DENSITY = 1100.0


def _rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _pose(rot: np.ndarray, t: tuple[float, float, float]) -> Pose:
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = t
    return pose_from_matrix(m)


def _setup(tmp_path: Path) -> tuple[RobotModel, list[Part], list[UUID], tuple[Path, Any]]:
    project = tmp_path / "p.rforge"
    asset, sha = add_asset(project, HEXAPOD / "xl330-Body.step")
    imported = read_step(HEXAPOD / "xl330-Body.step")
    base = candidate_parts(imported, asset, sha)
    links = [Link(name="a"), Link(name="b")]
    return RobotModel(name="r", links=links), base, [lk.id for lk in links], (project, imported)


def _reference(imported: Any, placement: Pose) -> MassProperties:
    """Servo mass properties in the link frame, built straight from the geometry."""
    root = matrix_from_pose(placement)
    bodies = []
    for leaf in imported.leaves:
        unit = brep_unit_mass_properties(imported.product_of(leaf).shape, "ref")
        m = root @ leaf.transform
        bodies.append(unit.at_density(DENSITY).transformed(m[:3, :3], m[:3, 3]))
    return combine(bodies)


def test_instances_share_sources_and_compose_placements(tmp_path: Path) -> None:
    _, base, (a, b), _ = _setup(tmp_path)
    pa = _pose(_rot_z(0.3), (0.01, 0.02, 0.03))
    pb = _pose(np.eye(3), (0.1, 0.0, 0.0))
    first = instantiate_assembly(base, pa, link=a, prefix="a/", mass=Density(density=DENSITY))
    second = instantiate_assembly(base, pb, link=b, prefix="b/", mass=Density(density=DENSITY))
    assert len(first) == len(second) == len(base) == 16
    assert len({p.id for p in first + second + base}) == 48
    for orig, x, y in zip(base, first, second, strict=True):
        assert x.source == y.source == orig.source
        assert x.name == f"a/{orig.name}" and y.name == f"b/{orig.name}"
        assert x.link == a and y.link == b
        np.testing.assert_allclose(
            matrix_from_pose(x.placement),
            matrix_from_pose(pa) @ matrix_from_pose(orig.placement),
            atol=1e-12,
        )
    assert all(p.mass is None for p in base)


def test_existing_mass_is_kept_and_mirrored_rejected(tmp_path: Path) -> None:
    _, base, (a, _), _ = _setup(tmp_path)
    pre = base[0].model_copy(update={"mass": Density(density=7800.0)})
    (out,) = instantiate_assembly([pre], Pose(), link=a, prefix="", mass=Density(density=DENSITY))
    assert out.mass == Density(density=7800.0)
    mirrored = base[0].model_copy(update={"mirror": MirrorPlane(normal=(1.0, 0.0, 0.0))})
    with pytest.raises(ValueError, match="mirrored"):
        instantiate_assembly([mirrored], Pose(), link=a, prefix="", mass=None)


def test_link_mass_is_parallel_axis_merge_of_servo_and_other_part(tmp_path: Path) -> None:
    model, base, (a, b), (project, imported) = _setup(tmp_path)
    pa = _pose(_rot_z(0.7), (0.04, -0.01, 0.02))
    pb = _pose(_rot_z(-1.2), (0.0, 0.05, 0.0))
    parts = instantiate_assembly(base, pa, link=a, prefix="a/", mass=Density(density=DENSITY))
    parts += instantiate_assembly(base, pb, link=b, prefix="b/", mass=Density(density=DENSITY))
    # A second servo in link a, so the link holds two instances of one assembly.
    pc = _pose(np.eye(3), (-0.05, 0.0, 0.0))
    parts += instantiate_assembly(base, pc, link=a, prefix="c/", mass=Density(density=DENSITY))
    result = compute_link_masses(model.model_copy(update={"parts": parts}), project)

    ref_a = combine([_reference(imported, pa), _reference(imported, pc)])
    ref_b = _reference(imported, pb)
    for link, ref in ((a, ref_a), (b, ref_b)):
        got = result[link]
        assert got.mass == pytest.approx(ref.mass, rel=1e-9)
        np.testing.assert_allclose(got.com, ref.com, atol=1e-12)
        np.testing.assert_allclose(got.inertia, ref.inertia, rtol=1e-6, atol=1e-15)
    # The same servo placed identically (up to the frame) has the same mass in both links.
    assert result[b].mass == pytest.approx(ref_a.mass / 2.0, rel=1e-9)
    assert result[a].mass == pytest.approx(2.0 * result[b].mass, rel=1e-9)
    # About the CoM: the pair's tensor is not just the sum of the two servo tensors.
    solo = _reference(imported, pa)
    assert not np.allclose(result[a].inertia, 2.0 * solo.inertia, rtol=1e-3)


# --- Manufacturer inertia override (M1 task 18) ---------------------------------------------

SERVO_MASS = 0.018


def _servo_override(com: tuple[float, float, float] = (-0.007, 0.0, 0.0)) -> InertialOverride:
    return InertialOverride(
        mass=SERVO_MASS,
        com=com,
        inertia=InertiaTensor(ixx=2.0e-6, iyy=3.0e-6, izz=2.5e-6),
        source="test datasheet",
    )


def _bounds(imported: Any) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for leaf in imported.leaves:
        box = brep_unit_mass_properties(imported.product_of(leaf).shape, "b")
        c = leaf.transform[:3, 3]
        lo, hi = np.minimum(lo, c - 0.03), np.maximum(hi, c + 0.03)
        del box
    return (float(lo[0]), float(lo[1]), float(lo[2])), (float(hi[0]), float(hi[1]), float(hi[2]))


def test_override_gives_link_the_datasheet_properties(tmp_path: Path) -> None:
    model, base, (a, _), (project, imported) = _setup(tmp_path)
    placement = _pose(_rot_z(0.4), (0.01, -0.02, 0.03))
    parts = instantiate_assembly(
        base,
        placement,
        link=a,
        prefix="s/",
        override=_servo_override(),
        com_bounds=_bounds(imported),
    )
    assert sum(p.inertial_override is not None for p in parts) == 1
    assert sum(isinstance(p.mass, Massless) for p in parts) == 15
    model.links.pop()
    model.parts.extend(parts)
    props = compute_link_masses(model, project)[a]
    assert props.mass == pytest.approx(SERVO_MASS)
    root = matrix_from_pose(placement)
    np.testing.assert_allclose(
        props.com, root[:3, :3] @ np.array([-0.007, 0, 0]) + root[:3, 3], atol=1e-12
    )
    expected = root[:3, :3] @ np.diag([2.0e-6, 3.0e-6, 2.5e-6]) @ root[:3, :3].T
    np.testing.assert_allclose(props.inertia, expected, atol=1e-15)


def test_override_on_a_named_part_with_a_rotated_frame(tmp_path: Path) -> None:
    model, base, (a, _), (project, _) = _setup(tmp_path)
    ov = _servo_override().model_copy(
        update={"orientation": (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))}
    )
    target = base[3].name
    parts = instantiate_assembly(base, Pose(), link=a, prefix="", override=ov, override_on=target)
    assert sum(p.inertial_override is not None for p in parts) == 1
    model.links.pop()
    model.parts.extend(parts)
    props = compute_link_masses(model, project)[a]
    rz = _rot_z(math.pi / 2)
    np.testing.assert_allclose(
        props.inertia, rz @ np.diag([2.0e-6, 3.0e-6, 2.5e-6]) @ rz.T, atol=1e-15
    )


def test_override_is_validated(tmp_path: Path) -> None:
    _, base, (a, _), (_, imported) = _setup(tmp_path)
    bad = _servo_override().model_copy(
        update={"inertia": InertiaTensor(ixx=1.0e-6, iyy=1.0e-6, izz=5.0e-6)}
    )
    with pytest.raises(ValueError, match="triangle"):
        instantiate_assembly(base, Pose(), link=a, prefix="", override=bad)
    with pytest.raises(ValueError, match="mass"):
        instantiate_assembly(
            base,
            Pose(),
            link=a,
            prefix="",
            override=_servo_override().model_copy(update={"mass": 0.0}),
        )
    # CoM given in millimetres by mistake lands far outside the geometry
    with pytest.raises(ValueError, match="outside"):
        instantiate_assembly(
            base,
            Pose(),
            link=a,
            prefix="",
            override=_servo_override((-7.0, 0.0, 0.0)),
            com_bounds=_bounds(imported),
        )
    with pytest.raises(ValueError, match="not a part"):
        instantiate_assembly(
            base, Pose(), link=a, prefix="", override=_servo_override(), override_on="nope"
        )
    with pytest.raises(ValueError, match="either"):
        instantiate_assembly(
            base, Pose(), link=a, prefix="", override=_servo_override(), mass=Density(density=1.0)
        )
