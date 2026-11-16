"""Tests for the synthetic STEP fixture generator (M1 task 5).

Each fixture is written and read back through ``STEPCAFControl_Reader`` on an XCAF document,
checking the assembly tree, product and instance names, colours, placements, volumes and
bounding boxes against the spec, in both mm and m files.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from rigidforge.testing.step_fixtures import (
    FIXED_TIMESTAMP,
    FIXTURES,
    AssemblyDef,
    Component,
    FixtureSpec,
    PartDef,
    Placement,
    gripper_arm,
    two_link_arm,
    write_step,
)


@dataclass
class ReadLeaf:
    path: tuple[str, ...]
    product: str
    colour: tuple[float, float, float] | None
    transform: np.ndarray  # world, in the reader's unit
    volume: float
    bbox: tuple[np.ndarray, np.ndarray]
    prototype: Any  # TopoDS_Shape of the referred product, without the placement


def _name(label) -> str:
    from OCP.TDataStd import TDataStd_Name

    attr = TDataStd_Name()
    assert label.FindAttribute(TDataStd_Name.GetID_s(), attr)
    return attr.Get().ToExtString()


def _matrix(location) -> np.ndarray:
    trsf = location.Transformation()
    out = np.eye(4)
    for i in range(3):
        for j in range(4):
            out[i, j] = trsf.Value(i + 1, j + 1)
    return out


def read_step(path: Path, unit: str) -> tuple[str, list[ReadLeaf]]:
    """Read a STEP file into (root product name, flattened leaves) with geometry in ``unit``."""
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepGProp import BRepGProp
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
    from OCP.gp import gp_Trsf
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_ReturnStatus
    from OCP.Interface import Interface_Static
    from OCP.Quantity import Quantity_Color, Quantity_TypeOfColor
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.STEPControl import STEPControl_Controller
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_Label
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import (
        XCAFDoc_ColorTool,
        XCAFDoc_ColorType,
        XCAFDoc_DocumentTool,
        XCAFDoc_ShapeTool,
    )

    STEPControl_Controller.Init_s()
    assert Interface_Static.SetCVal_s("xstep.cascade.unit", unit)
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    XCAFApp_Application.GetApplication_s().InitDocument(doc)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    assert reader.ReadFile(str(path)) == IFSelect_ReturnStatus.IFSelect_RetDone
    assert reader.Transfer(doc)

    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    free = TDF_LabelSequence()
    shapes.GetFreeShapes(free)
    assert free.Length() == 1
    root = free.Value(1)

    leaves: list[ReadLeaf] = []

    def walk(label: TDF_Label, path: tuple[str, ...], world: np.ndarray) -> None:
        comps = TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(label, comps, False)
        for i in range(1, comps.Length() + 1):
            comp = comps.Value(i)
            ref = TDF_Label()
            assert XCAFDoc_ShapeTool.GetReferredShape_s(comp, ref)
            here = world @ _matrix(XCAFDoc_ShapeTool.GetLocation_s(comp))
            sub = (*path, _name(comp))
            if XCAFDoc_ShapeTool.IsAssembly_s(ref):
                walk(ref, sub, here)
                continue
            proto = XCAFDoc_ShapeTool.GetShape_s(ref)
            trsf = gp_Trsf()
            trsf.SetValues(*(float(here[r, c]) for r in range(3) for c in range(4)))
            world_shape = proto.Moved(TopLoc_Location(trsf))
            props = GProp_GProps()
            BRepGProp.VolumeProperties_s(world_shape, props)
            box = Bnd_Box()
            BRepBndLib.AddOptimal_s(world_shape, box, False, False)
            lo, hi = box.CornerMin(), box.CornerMax()
            colour = Quantity_Color()
            rgb = None
            if XCAFDoc_ColorTool.GetColor_s(ref, XCAFDoc_ColorType.XCAFDoc_ColorSurf, colour):
                rgb = colour.Values(Quantity_TypeOfColor.Quantity_TOC_RGB)
            leaves.append(
                ReadLeaf(
                    sub,
                    _name(ref),
                    rgb,
                    here,
                    props.Mass(),
                    (np.array([lo.X(), lo.Y(), lo.Z()]), np.array([hi.X(), hi.Y(), hi.Z()])),
                    proto,
                )
            )

    walk(root, (), np.eye(4))
    return _name(root), leaves


SCALE = {"MM": 1.0, "M": 1e-3}


@pytest.mark.parametrize("fixture", sorted(FIXTURES))
@pytest.mark.parametrize("file_unit", ["MM", "M"])
@pytest.mark.parametrize("read_unit", ["MM", "M"])
def test_round_trip(tmp_path: Path, fixture: str, file_unit: str, read_unit: str) -> None:
    spec = FIXTURES[fixture]()
    path = write_step(spec, tmp_path / f"{fixture}.step", file_unit)  # type: ignore[arg-type]
    root, leaves = read_step(path, read_unit)
    s = SCALE[read_unit]

    assert root == spec.root
    expected = spec.leaves()
    assert [leaf.path for leaf in leaves] == [e.path for e in expected]
    for got, want in zip(leaves, expected, strict=True):
        assert got.product == want.part.name
        assert got.colour is not None
        np.testing.assert_allclose(got.colour, want.part.colour, atol=1e-6)
        np.testing.assert_allclose(got.transform[:3, :3], want.transform[:3, :3], atol=1e-12)
        np.testing.assert_allclose(got.transform[:3, 3], want.transform[:3, 3] * s, atol=1e-9 * s)
        # BRepGProp on these analytic surfaces is exact to ~1e-9 relative.
        assert got.volume == pytest.approx(want.part.volume_mm3() * s**3, rel=1e-6)
        lo, hi = want.bbox_mm()
        np.testing.assert_allclose(got.bbox[0], lo * s, atol=1e-6 * s)
        np.testing.assert_allclose(got.bbox[1], hi * s, atol=1e-6 * s)


def test_repeated_part_is_one_shared_product(tmp_path: Path) -> None:
    """The two finger instances refer to one product, not two copies of the geometry."""
    _, leaves = read_step(write_step(gripper_arm(), tmp_path / "g.step"), "MM")
    fingers = [leaf for leaf in leaves if leaf.product == "finger"]
    assert [f.path[-1] for f in fingers] == ["finger_1", "finger_2"]
    assert fingers[0].prototype.IsPartner(fingers[1].prototype)
    assert not fingers[0].prototype.IsPartner(leaves[0].prototype)


def test_rewrite_gives_same_content(tmp_path: Path) -> None:
    """Writes are not byte-reproducible (OCCT entity order), but the content is."""
    a = write_step(gripper_arm(), tmp_path / "a.step")
    b = write_step(gripper_arm(), tmp_path / "b.step")
    assert FIXED_TIMESTAMP in a.read_text()
    (root_a, leaves_a), (root_b, leaves_b) = read_step(a, "MM"), read_step(b, "MM")
    assert root_a == root_b
    for x, y in zip(leaves_a, leaves_b, strict=True):
        assert (x.path, x.product, x.colour, x.volume) == (y.path, y.product, y.colour, y.volume)
        np.testing.assert_array_equal(x.transform, y.transform)


def test_file_declares_requested_unit(tmp_path: Path) -> None:
    mm = write_step(two_link_arm(), tmp_path / "mm.step", "MM").read_text()
    m = write_step(two_link_arm(), tmp_path / "m.step", "M").read_text()
    assert "SI_UNIT(.MILLI.,.METRE.)" in mm
    assert "SI_UNIT($,.METRE.)" in m


def test_writer_ignores_earlier_reader_unit(tmp_path: Path) -> None:
    """A reader set to metres earlier in the process must not shrink the next fixture."""
    from OCP.Interface import Interface_Static

    read_step(write_step(two_link_arm(), tmp_path / "a.step"), "M")
    assert Interface_Static.CVal_s("xstep.cascade.unit") == "M"
    _, leaves = read_step(write_step(two_link_arm(), tmp_path / "b.step"), "MM")
    assert leaves[0].volume == pytest.approx(100.0 * 80.0 * 40.0, rel=1e-9)


def test_spec_rejects_bad_references() -> None:
    box = PartDef("box", "box", (1.0, 1.0, 1.0), (1.0, 1.0, 1.0))
    with pytest.raises(ValueError, match="unknown"):
        FixtureSpec("a", (box,), (AssemblyDef("a", (Component("c", "nope"),)),))
    with pytest.raises(ValueError, match="duplicate"):
        FixtureSpec("box", (box,), (AssemblyDef("box", ()),))
    with pytest.raises(ValueError, match="not an assembly"):
        FixtureSpec("box", (box,), ())


def test_placement_matrix_rotates_then_translates() -> None:
    m = Placement((1.0, 2.0, 3.0), (0.0, 0.0, 1.0), 90.0).matrix()
    np.testing.assert_allclose(m @ [1.0, 0.0, 0.0, 1.0], [1.0, 3.0, 3.0, 1.0], atol=1e-12)
