"""Tests for the STEP importer (M1 task 6).

Synthetic fixtures (``rigidforge.testing.step_fixtures``) give exact expected placements,
volumes and bounding boxes; the hexapod leg files check a real CAD export (an XL330 servo
assembly with nested connector sub-assemblies and repeated screws).
"""

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from rigidforge.core.model import RobotModel, SourceKind
from rigidforge.core.transforms import matrix_from_pose
from rigidforge.importers.step import (
    StepImport,
    StepImportError,
    StepLeaf,
    candidate_parts,
    read_step,
    unit_metres,
)
from rigidforge.testing.step_fixtures import FIXTURES, gripper_arm, two_link_arm, write_step

HEXAPOD = Path(__file__).parent.parent / "data" / "hexapod-leg"


def _world_props(imported: StepImport, leaf: StepLeaf) -> tuple[float, np.ndarray, np.ndarray]:
    """Volume (m^3) and tight bounding box (m) of a leaf placed in the root frame."""
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepGProp import BRepGProp
    from OCP.gp import gp_Trsf
    from OCP.GProp import GProp_GProps
    from OCP.TopLoc import TopLoc_Location

    m = leaf.transform
    trsf = gp_Trsf()
    trsf.SetValues(*(float(m[r, c]) for r in range(3) for c in range(4)))
    shape: Any = imported.product_of(leaf).shape.Moved(TopLoc_Location(trsf))
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    lo, hi = box.CornerMin(), box.CornerMax()
    return (
        props.Mass(),
        np.array([lo.X(), lo.Y(), lo.Z()]),
        np.array([hi.X(), hi.Y(), hi.Z()]),
    )


@pytest.mark.parametrize("fixture", sorted(FIXTURES))
@pytest.mark.parametrize("file_unit", ["MM", "M"])
def test_fixture_imports_in_si(tmp_path: Path, fixture: str, file_unit: str) -> None:
    spec = FIXTURES[fixture]()
    path = write_step(spec, tmp_path / f"{fixture}.step", file_unit)  # type: ignore[arg-type]
    imported = read_step(path)

    assert imported.root_name == spec.root
    assert imported.file_unit == {"MM": "millimetre", "M": "metre"}[file_unit]
    assert imported.scale_to_si == {"MM": 1e-3, "M": 1.0}[file_unit]
    expected = spec.leaves()
    assert [leaf.path for leaf in imported.leaves] == [e.path for e in expected]
    for got, want in zip(imported.leaves, expected, strict=True):
        product = imported.product_of(got)
        assert product.name == want.part.name
        assert product.colour is not None
        np.testing.assert_allclose(product.colour, want.part.colour, atol=1e-6)
        np.testing.assert_allclose(got.transform[:3, :3], want.transform[:3, :3], atol=1e-12)
        np.testing.assert_allclose(got.transform[:3, 3], want.transform[:3, 3] * 1e-3, atol=1e-12)
        volume, lo, hi = _world_props(imported, got)
        assert volume == pytest.approx(want.part.volume_mm3() * 1e-9, rel=1e-6)
        want_lo, want_hi = want.bbox_mm()
        np.testing.assert_allclose(lo, want_lo * 1e-3, atol=1e-9)
        np.testing.assert_allclose(hi, want_hi * 1e-3, atol=1e-9)


def test_repeated_part_is_one_product(tmp_path: Path) -> None:
    imported = read_step(write_step(gripper_arm(), tmp_path / "g.step"))
    assert sorted(p.name for p in imported.products.values()) == ["base", "finger", "link", "tip"]
    fingers = [leaf for leaf in imported.leaves if imported.product_of(leaf).name == "finger"]
    assert [f.path for f in fingers] == [("wrist_1", "finger_1"), ("wrist_1", "finger_2")]
    assert fingers[0].product == fingers[1].product


@pytest.mark.parametrize("prior", ["MM", "M", "CM"])
def test_reader_unit_is_independent_of_and_restores_global(tmp_path: Path, prior: str) -> None:
    from OCP.Interface import Interface_Static
    from OCP.STEPControl import STEPControl_Controller

    path = write_step(two_link_arm(), tmp_path / "a.step")
    STEPControl_Controller.Init_s()
    assert Interface_Static.SetCVal_s("xstep.cascade.unit", prior)
    imported = read_step(path)
    assert Interface_Static.CVal_s("xstep.cascade.unit") == prior
    volume, _, _ = _world_props(imported, imported.leaves[0])
    assert volume == pytest.approx(0.1 * 0.08 * 0.04, rel=1e-9)


def test_scale_override_scales_the_brep(tmp_path: Path) -> None:
    """A model drawn in inches but exported with a mm header: override to 0.0254 m/unit."""
    path = write_step(two_link_arm(), tmp_path / "a.step")
    imported = read_step(path, scale_to_si=0.0254)
    assert imported.file_unit_metres == 1e-3
    assert imported.scale_to_si == 0.0254
    base, arm = imported.leaves
    volume, lo, hi = _world_props(imported, base)
    s = 0.0254
    assert volume == pytest.approx(100.0 * 80.0 * 40.0 * s**3, rel=1e-9)
    np.testing.assert_allclose(lo, np.array([-50.0, -40.0, -20.0]) * s, atol=1e-9)
    np.testing.assert_allclose(hi, np.array([50.0, 40.0, 20.0]) * s, atol=1e-9)
    np.testing.assert_allclose(arm.transform[:3, 3], [0.0, 0.0, 95.0 * s], atol=1e-12)
    arm_volume, _, _ = _world_props(imported, arm)
    assert arm_volume == pytest.approx(np.pi * 10.0**2 * 150.0 * s**3, rel=1e-6)


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_rejects_bad_scale(tmp_path: Path, bad: float) -> None:
    path = write_step(two_link_arm(), tmp_path / "a.step")
    with pytest.raises(StepImportError, match="scale_to_si"):
        read_step(path, scale_to_si=bad)


def test_missing_and_garbage_files(tmp_path: Path) -> None:
    with pytest.raises(StepImportError, match="no such file"):
        read_step(tmp_path / "nope.step")
    junk = tmp_path / "junk.step"
    junk.write_text("this is not a STEP file\n")
    with pytest.raises(StepImportError):
        read_step(junk)


@pytest.mark.parametrize(
    ("name", "metres"),
    [("millimetre", 1e-3), ("METRE", 1.0), ("FOOT", 0.3048), ("inch", 0.0254), ("furlong", None)],
)
def test_unit_names(name: str, metres: float | None) -> None:
    assert unit_metres(name) == metres


def test_candidate_parts(tmp_path: Path) -> None:
    spec = gripper_arm()
    path = write_step(spec, tmp_path / "g.step")
    imported = read_step(path)
    parts = candidate_parts(imported, "g-abc123.step", "abc123", str(path))

    assert [p.name for p in parts] == [e.part.name for e in spec.leaves()]
    assert len({p.id for p in parts}) == len(parts)
    for part, leaf in zip(parts, imported.leaves, strict=True):
        assert part.link is None
        assert part.mass is None
        assert part.source.kind is SourceKind.STEP
        assert part.source.asset == "g-abc123.step"
        assert part.source.sha256 == "abc123"
        assert part.source.scale_to_si == 1e-3
        assert part.source.assembly_path == list(leaf.path)
        assert part.source.original_path == str(path)
        np.testing.assert_allclose(matrix_from_pose(part.placement), leaf.transform, atol=1e-12)
    model = RobotModel(name="g", parts=parts)
    assert RobotModel.model_validate_json(model.model_dump_json()) == model


def test_hexapod_single_solid_frame() -> None:
    imported = read_step(HEXAPOD / "coxa-frame.step")
    assert imported.file_unit == "millimetre"
    assert imported.root_name == "coxa-frame"
    (leaf,) = imported.leaves
    assert leaf.path == ()
    np.testing.assert_array_equal(leaf.transform, np.eye(4))
    volume, lo, hi = _world_props(imported, leaf)
    # Metres, not millimetres: the frame is a few centimetres across and a few cm^3.
    assert 1e-7 < volume < 1e-4
    assert lo[0] == pytest.approx(-0.0085, abs=1e-6)
    assert hi[0] == pytest.approx(0.055, abs=1e-6)


def test_hexapod_servo_assembly() -> None:
    imported = read_step(HEXAPOD / "xl330-Body.step")
    assert imported.file_unit == "millimetre"
    assert len(imported.products) == 7
    assert len(imported.leaves) == 16
    screws = [leaf for leaf in imported.leaves if "PHS" in imported.product_of(leaf).name]
    assert len(screws) == 4
    assert len({s.product for s in screws}) == 1
    nested = [leaf.path for leaf in imported.leaves if len(leaf.path) == 2]
    assert ("XL330_B3B_EH:1", "shell:1") in nested
    assert ("XL330_B3B_EH:2", "pin:3") in nested
    for leaf in imported.leaves:
        # Every part of a 34 x 20 x 23 mm servo sits within a few cm of its origin.
        assert np.all(np.abs(leaf.transform[:3, 3]) < 0.05)
