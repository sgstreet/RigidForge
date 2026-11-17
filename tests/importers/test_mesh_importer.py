"""Tests for the mesh importer and mesh mass properties (M1 task 8).

Synthetic boxes (``rigidforge.testing.mesh_fixtures``) give analytic volume, CoM and inertia
in SI; the real Bambu Studio 3MF check runs only when the files are present locally.
"""

from pathlib import Path

import numpy as np
import pytest

from rigidforge.core.model import SourceKind
from rigidforge.core.transforms import matrix_from_pose
from rigidforge.importers.mesh import MeshImportError, candidate_parts, read_mesh
from rigidforge.physics.inertia import solid_box
from rigidforge.physics.mass import MassPropertiesError
from rigidforge.physics.mesh import closed_mesh_problems, mesh_unit_mass_properties
from rigidforge.testing.mesh_fixtures import (
    box_mesh,
    write_3mf_box,
    write_dae_box,
    write_obj_box,
    write_stl_box,
)

SIZE_MM = (20.0, 30.0, 40.0)
SIZE_M = (0.02, 0.03, 0.04)


def _transform(rz_deg: float, t: tuple[float, float, float]) -> np.ndarray:
    a = np.radians(rz_deg)
    m = np.eye(4)
    m[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    m[:3, 3] = t
    return m


def _check_box(imp_obj, size_m=SIZE_M):  # type: ignore[no-untyped-def]
    unit = mesh_unit_mass_properties(imp_obj.vertices, imp_obj.faces, imp_obj.name)
    assert unit.volume == pytest.approx(np.prod(size_m), rel=1e-12)
    np.testing.assert_allclose(unit.com, 0.0, atol=1e-15)
    expected = solid_box(float(np.prod(size_m)), size_m).inertia
    np.testing.assert_allclose(unit.inertia, expected, rtol=1e-10, atol=1e-18)
    return unit


def test_box_mesh_is_closed() -> None:
    v, f = box_mesh(SIZE_M)
    assert closed_mesh_problems(v, f) == []


def test_3mf_millimetres_scaled_to_si(tmp_path: Path) -> None:
    p = tmp_path / "box.3mf"
    write_3mf_box(p, SIZE_MM)
    imp = read_mesh(p)
    assert imp.file_unit == "millimeter"
    assert imp.scale_to_si == 1e-3
    assert len(imp.objects) == 1
    obj = imp.objects[0]
    assert obj.watertight
    np.testing.assert_allclose(np.ptp(obj.vertices, axis=0), SIZE_M, rtol=1e-12)
    _check_box(obj)


def test_3mf_component_and_build_transforms(tmp_path: Path) -> None:
    p = tmp_path / "box.3mf"
    comp = _transform(0.0, (5.0, 0.0, 0.0))
    build = _transform(90.0, (100.0, 200.0, 300.0))
    write_3mf_box(p, SIZE_MM, component=comp, build=build)
    obj = read_mesh(p).objects[0]
    expected = _transform(90.0, (100.0, 200.0, 300.0)) @ _transform(0.0, (5.0, 0.0, 0.0))
    expected[:3, 3] *= 1e-3
    np.testing.assert_allclose(obj.transform, expected, atol=1e-12)
    # World-frame CoM of the box is the placement translation (box is centred).
    unit = _check_box(obj)
    world_com = obj.transform[:3, :3] @ unit.com + obj.transform[:3, 3]
    np.testing.assert_allclose(world_com, expected[:3, 3], atol=1e-12)


def test_3mf_multiple_objects_become_candidate_parts(tmp_path: Path) -> None:
    p = tmp_path / "plate.3mf"
    write_3mf_box(p, SIZE_MM, extra_objects=2)
    imp = read_mesh(p)
    assert len(imp.objects) == 3
    assert len({o.name for o in imp.objects}) == 3
    parts = candidate_parts(imp, "assets/plate.3mf", "0" * 64)
    assert [pt.source.kind for pt in parts] == [SourceKind.MESH] * 3
    assert {pt.source.assembly_path[0] for pt in parts} == {o.name for o in imp.objects}
    assert all(pt.source.scale_to_si == 1e-3 for pt in parts)
    xs = sorted(matrix_from_pose(pt.placement)[0, 3] for pt in parts)
    np.testing.assert_allclose(xs, [0.0, 0.04, 0.08], atol=1e-12)


def test_3mf_inch_unit(tmp_path: Path) -> None:
    p = tmp_path / "box.3mf"
    write_3mf_box(p, (1.0, 2.0, 3.0), unit="inch")
    imp = read_mesh(p)
    assert imp.scale_to_si == 0.0254
    unit = mesh_unit_mass_properties(imp.objects[0].vertices, imp.objects[0].faces)
    assert unit.volume == pytest.approx(6.0 * 0.0254**3, rel=1e-12)


def test_3mf_mirror_is_baked_and_stays_outward(tmp_path: Path) -> None:
    p = tmp_path / "box.3mf"
    mirror = np.diag([-1.0, 1.0, 1.0, 1.0])
    mirror[:3, 3] = (10.0, 0.0, 0.0)
    write_3mf_box(p, SIZE_MM, build=mirror)
    obj = read_mesh(p).objects[0]
    np.testing.assert_allclose(obj.transform, np.eye(4))
    assert obj.watertight
    unit = mesh_unit_mass_properties(obj.vertices, obj.faces)
    assert unit.volume == pytest.approx(np.prod(SIZE_M), rel=1e-12)
    np.testing.assert_allclose(unit.com, (0.01, 0.0, 0.0), atol=1e-12)


@pytest.mark.parametrize("writer", [write_stl_box, write_obj_box])
def test_unitless_formats_use_scale_override(tmp_path: Path, writer) -> None:  # type: ignore[no-untyped-def]
    p = tmp_path / f"box.{writer.__name__.split('_')[1]}"
    writer(p, SIZE_MM)  # written in mm numbers
    default = read_mesh(p)
    assert default.file_unit is None
    assert default.scale_to_si == 1.0
    imp = read_mesh(p, scale_to_si=1e-3)
    assert imp.scale_to_si == 1e-3
    _check_box(imp.objects[0])


def test_stl_in_metres(tmp_path: Path) -> None:
    p = tmp_path / "box.stl"
    write_stl_box(p)
    _check_box(read_mesh(p).objects[0])


def test_dae_unit_is_read_from_file(tmp_path: Path) -> None:
    p = tmp_path / "box.dae"
    write_dae_box(p, SIZE_MM, unit_meter=0.001)
    imp = read_mesh(p)
    assert imp.scale_to_si == pytest.approx(1e-3)
    _check_box(imp.objects[0])


def test_dae_unit_not_applied_twice(tmp_path: Path) -> None:
    # Whether or not the loader applies <unit>, the result must be 20 mm, not 20 µm or 20 m.
    p = tmp_path / "box.dae"
    write_dae_box(p, SIZE_MM, unit_meter=0.001)
    obj = read_mesh(p).objects[0]
    np.testing.assert_allclose(np.ptp(obj.vertices, axis=0), SIZE_M, rtol=1e-9)


def test_open_mesh_is_imported_but_flagged(tmp_path: Path) -> None:
    p = tmp_path / "open.obj"
    v, f = box_mesh(SIZE_M)
    lines = ["o open"] + [f"v {x:.9g} {y:.9g} {z:.9g}" for x, y, z in v]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in f[:-2]]  # drop one face quad
    p.write_text("\n".join(lines) + "\n")
    obj = read_mesh(p).objects[0]
    assert not obj.watertight
    assert any("open edges" in s for s in obj.problems)
    with pytest.raises(MassPropertiesError, match="not a closed solid"):
        mesh_unit_mass_properties(obj.vertices, obj.faces, obj.name)


def test_inward_wound_mesh_is_rejected() -> None:
    v, f = box_mesh(SIZE_M)
    assert any("inward" in s for s in closed_mesh_problems(v, f[:, ::-1]))


def test_inconsistent_winding_is_flagged() -> None:
    v, f = box_mesh(SIZE_M)
    f = f.copy()
    f[0] = f[0, ::-1]
    assert closed_mesh_problems(v, f)


def test_far_from_origin_mesh_keeps_precision() -> None:
    v, f = box_mesh(SIZE_M)
    unit = mesh_unit_mass_properties(v + np.array([1e3, -2e3, 5e2]), f)
    np.testing.assert_allclose(
        unit.inertia, solid_box(float(np.prod(SIZE_M)), SIZE_M).inertia, rtol=1e-6, atol=1e-15
    )
    np.testing.assert_allclose(unit.com, (1e3, -2e3, 5e2), atol=1e-9)


def test_errors(tmp_path: Path) -> None:
    with pytest.raises(MeshImportError, match="no such file"):
        read_mesh(tmp_path / "missing.stl")
    with pytest.raises(MeshImportError, match="unsupported"):
        read_mesh(tmp_path / "x.ply")
    bad = tmp_path / "bad.stl"
    bad.write_text("not an stl")
    with pytest.raises(MeshImportError):
        read_mesh(bad)
    ok = tmp_path / "ok.stl"
    write_stl_box(ok)
    with pytest.raises(MeshImportError, match="scale_to_si"):
        read_mesh(ok, scale_to_si=-1.0)


_REAL = Path.home() / "Downloads" / "shin_tibias_example.3mf"


@pytest.mark.skipif(not _REAL.is_file(), reason="real 3MF sample not present")
def test_real_slicer_3mf() -> None:
    imp = read_mesh(_REAL)
    assert imp.scale_to_si == 1e-3
    assert len(imp.objects) == 2
    for obj in imp.objects:
        assert obj.watertight
        unit = mesh_unit_mass_properties(obj.vertices, obj.faces)
        assert 5e-5 < unit.volume < 1.5e-4  # tens of cm³, in m³
