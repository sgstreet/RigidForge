"""Tests for mirroring geometry and for part instancing (M1 task 15)."""

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from rigidforge.core.geometry import Pose
from rigidforge.core.instancing import mirrored_copy, rigid_copy
from rigidforge.core.model import (
    Density,
    Link,
    MirrorPlane,
    Part,
    PartSource,
    RobotModel,
    SourceKind,
)
from rigidforge.core.project import add_asset, save_project
from rigidforge.core.validation import validate
from rigidforge.importers.step import read_step
from rigidforge.physics.brep import brep_unit_mass_properties
from rigidforge.physics.mesh import closed_mesh_problems, mesh_unit_mass_properties
from rigidforge.physics.mirror import MirrorError, mirror_mesh, mirror_shape, reflection_matrix
from rigidforge.pipeline import BuildError, compute_link_masses
from rigidforge.testing.mesh_fixtures import box_mesh, write_stl_box

HEXAPOD = Path(__file__).parent.parent / "data" / "hexapod-leg"
PLANES = [
    MirrorPlane(normal=(1.0, 0.0, 0.0)),
    MirrorPlane(point=(0.01, -0.02, 0.003), normal=(0.0, 2.0, 0.0)),
    MirrorPlane(point=(0.004, 0.0, 0.01), normal=(1.0, 2.0, -3.0)),
]


def _expected(unit: Any, plane: MirrorPlane) -> tuple[Any, Any]:
    m = reflection_matrix(plane)
    com = m[:3, :3] @ unit.com + m[:3, 3]
    return com, m[:3, :3] @ unit.inertia @ m[:3, :3].T


def test_reflection_matrix_properties():
    for plane in PLANES:
        m = reflection_matrix(plane)
        assert np.allclose(m @ m, np.eye(4), atol=1e-12)
        assert np.linalg.det(m[:3, :3]) == pytest.approx(-1.0)
        p = np.append(plane.point, 1.0)
        assert np.allclose(m @ p, p)


def test_degenerate_plane_is_rejected():
    with pytest.raises(MirrorError):
        reflection_matrix(MirrorPlane(normal=(0.0, 0.0, 0.0)))
    with pytest.raises(MirrorError):
        reflection_matrix(MirrorPlane(point=(float("nan"), 0, 0), normal=(1.0, 0.0, 0.0)))


@pytest.mark.parametrize("plane", PLANES)
@pytest.mark.parametrize("name", ["coxa-frame", "femur-frame", "tibia-frame"])
def test_brep_mirror_matches_reflected_properties(name: str, plane: MirrorPlane):
    shape = read_step(HEXAPOD / f"{name}.step")
    leaf = shape.leaves[0]
    original = shape.product_of(leaf).shape
    unit = brep_unit_mass_properties(original)
    mirrored = brep_unit_mass_properties(mirror_shape(original, plane))
    com, inertia = _expected(unit, plane)
    assert mirrored.volume == pytest.approx(unit.volume, rel=1e-9)
    assert np.allclose(mirrored.com, com, rtol=0, atol=1e-9 * max(1.0, np.abs(com).max()))
    assert np.allclose(mirrored.inertia, inertia, rtol=1e-6, atol=1e-6 * np.abs(inertia).max())


def test_mirrored_brep_is_a_valid_solid():
    from OCP.BRepCheck import BRepCheck_Analyzer

    shape = read_step(HEXAPOD / "femur-frame.step")
    mirrored = mirror_shape(shape.product_of(shape.leaves[0]).shape, PLANES[0])
    assert BRepCheck_Analyzer(mirrored).IsValid()


@pytest.mark.parametrize("plane", PLANES)
def test_mesh_mirror_keeps_outward_winding_and_matches(plane: MirrorPlane):
    vertices, faces = box_mesh((0.02, 0.03, 0.04), centred=False)
    unit = mesh_unit_mass_properties(vertices, faces)
    v2, f2 = mirror_mesh(vertices, faces, plane)
    assert closed_mesh_problems(v2, f2) == []
    mirrored = mesh_unit_mass_properties(v2, f2)
    com, inertia = _expected(unit, plane)
    assert mirrored.volume == pytest.approx(unit.volume, rel=1e-12)
    assert np.allclose(mirrored.com, com, atol=1e-12)
    assert np.allclose(mirrored.inertia, inertia, rtol=1e-9, atol=1e-18)


def test_mirror_of_symmetric_part_about_its_centre_is_unchanged():
    vertices, faces = box_mesh((0.02, 0.03, 0.04))
    unit = mesh_unit_mass_properties(vertices, faces)
    v2, f2 = mirror_mesh(vertices, faces, MirrorPlane(normal=(0.0, 1.0, 0.0)))
    mirrored = mesh_unit_mass_properties(v2, f2)
    assert np.allclose(mirrored.com, unit.com, atol=1e-15)
    assert np.allclose(mirrored.inertia, unit.inertia, rtol=1e-12)


# --- Instancing --------------------------------------------------------------------------


def _part() -> Part:
    return Part(
        name="frame",
        source=PartSource(kind=SourceKind.STEP, asset="a.step", sha256="0" * 64, scale_to_si=1e-3),
        placement=Pose(xyz=(0.1, 0.0, 0.0)),
        mass=Density(density=1270.0),
    )


def test_rigid_copy_shares_source_and_gets_a_new_id():
    original = _part()
    placement = Pose(xyz=(0.0, 0.2, 0.0), quat=(0.0, 1.0, 0.0, 0.0))
    copy = rigid_copy(original, name="frame2", placement=placement)
    assert copy.id != original.id
    assert copy.name == "frame2"
    assert copy.placement == placement
    assert copy.source == original.source
    assert copy.mirror is None and copy.mass == original.mass
    assert original.placement.xyz == (0.1, 0.0, 0.0)


def test_mirrored_copy_sets_plane_and_refuses_double_mirror():
    original = _part()
    plane = MirrorPlane(normal=(0.0, 1.0, 0.0))
    copy = mirrored_copy(original, plane)
    assert copy.mirror == plane and original.mirror is None
    assert copy.source == original.source
    with pytest.raises(ValueError, match="already mirrored"):
        mirrored_copy(copy, plane)


# --- Through the pipeline ----------------------------------------------------------------


def test_pipeline_masses_for_rigid_and_mirrored_instances(tmp_path: Path):
    project = tmp_path / "p.rforge"
    project.mkdir()
    stl = tmp_path / "box.stl"
    size = (0.02, 0.03, 0.04)
    write_stl_box(stl, size)
    asset, sha = add_asset(project, stl)
    link = Link(name="base")
    source = PartSource(kind=SourceKind.MESH, asset=asset, sha256=sha, assembly_path=["box"])
    base = Part(name="box", source=source, link=link.id, mass=Density(density=1000.0))
    # Probe the real object name from the importer so the test does not hard-code it.
    from rigidforge.importers.mesh import read_mesh

    name = read_mesh(project / "assets" / asset).objects[0].name
    base = base.model_copy(update={"source": source.model_copy(update={"assembly_path": [name]})})
    right = rigid_copy(base, name="right", placement=Pose(xyz=(0.0, 0.1, 0.0)))
    left = mirrored_copy(
        base,
        MirrorPlane(normal=(0.0, 1.0, 0.0), point=(0.0, 0.01, 0.0)),
        name="left",
        placement=Pose(xyz=(0.0, -0.1, 0.0)),
    )
    model = RobotModel(name="r", links=[link], parts=[base, right, left])
    assert validate(model).ok
    save_project(model, project)
    props = compute_link_masses(model, project)[link.id]
    single = 1000.0 * size[0] * size[1] * size[2]
    assert props.mass == pytest.approx(3 * single, rel=1e-9)
    # The centred box mirrored about y = 0.01 sits at y = 0.02; the placements add -0.1.
    assert np.allclose(props.com, (0.0, (0.0 + 0.1 - 0.08) / 3, 0.0), atol=1e-12)


def test_pipeline_rejects_degenerate_mirror_plane_cleanly(tmp_path: Path):
    project = tmp_path / "p.rforge"
    project.mkdir()
    stl = tmp_path / "box.stl"
    write_stl_box(stl)
    asset, sha = add_asset(project, stl)
    from rigidforge.importers.mesh import read_mesh

    name = read_mesh(project / "assets" / asset).objects[0].name
    link = Link(name="base")
    part = Part(
        name="box",
        source=PartSource(kind=SourceKind.MESH, asset=asset, sha256=sha, assembly_path=[name]),
        link=link.id,
        mass=Density(density=1000.0),
        mirror=MirrorPlane(normal=(0.0, 0.0, 0.0)),
    )
    model = RobotModel(name="r", links=[link], parts=[part])
    with pytest.raises(BuildError, match="degenerate mirror plane"):
        compute_link_masses(model, project)
