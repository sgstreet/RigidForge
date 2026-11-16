"""Synthetic STEP assembly fixtures (M1 task 5).

Builds small XCAF assemblies from analytic primitives (box, cylinder, sphere) with product
names, instance names and surface colours, and writes them with ``STEPCAFControl_Writer``.
The same spec also gives the expected result (world placements, volumes, bounding boxes),
so importer and mass-property tests can compare against exact values without committing
binary fixtures.

Conventions:

* Specs are in millimetres, the unit most CAD exports use. ``write_step`` can write the file
  in ``"MM"`` (default) or ``"M"``; the geometry is the same either way, so importer tests can
  check that the unit is honoured at the import boundary.
* Every primitive is centred on its own origin (box and sphere centre, cylinder mid-height,
  axis along z), so each part's CoM is at its local origin.
* A part used by more than one component is a shared product with several instances, which
  is how CAD tools encode repeated parts.
* Output is not byte-reproducible. OCCT numbers assembly occurrences from a process-global
  counter and emits colour styles in a pointer-dependent order, so two writes of one spec can
  differ in entity order. The header timestamp is fixed, and the content (tree, names,
  colours, geometry) is always the same. Tests compare content, never fixture bytes.
"""

from __future__ import annotations

import math
import re
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

import numpy as np
from numpy.typing import NDArray

PrimitiveKind = Literal["box", "cylinder", "sphere"]
StepUnit = Literal["MM", "M"]

FIXED_TIMESTAMP = "2026-01-01T00:00:00"
"""Timestamp written into every fixture's STEP header."""


@dataclass(frozen=True)
class Placement:
    """Rigid placement in mm: rotate about ``axis`` by ``angle_deg``, then translate."""

    translation_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    angle_deg: float = 0.0

    def matrix(self) -> NDArray[np.float64]:
        """The 4x4 homogeneous transform (translation in mm)."""
        a = np.asarray(self.axis, dtype=np.float64)
        a = a / np.linalg.norm(a)
        t = math.radians(self.angle_deg)
        k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
        rot = np.eye(3) + math.sin(t) * k + (1.0 - math.cos(t)) * (k @ k)
        out = np.eye(4)
        out[:3, :3] = rot
        out[:3, 3] = self.translation_mm
        return out


@dataclass(frozen=True)
class PartDef:
    """A leaf product: one primitive solid centred on its origin.

    ``dims_mm`` is ``(x, y, z)`` for a box, ``(radius, height)`` for a cylinder along z, and
    ``(radius,)`` for a sphere. ``colour`` is linear RGB in 0..1.
    """

    name: str
    kind: PrimitiveKind
    dims_mm: tuple[float, ...]
    colour: tuple[float, float, float]

    def volume_mm3(self) -> float:
        """Exact volume in mm^3."""
        if self.kind == "box":
            x, y, z = self.dims_mm
            return x * y * z
        if self.kind == "cylinder":
            r, h = self.dims_mm
            return math.pi * r * r * h
        (r,) = self.dims_mm
        return 4.0 / 3.0 * math.pi * r**3

    def half_extents_mm(self) -> tuple[float, float, float]:
        """Half extents of the local axis-aligned bounding box."""
        if self.kind == "box":
            x, y, z = self.dims_mm
            return (x / 2, y / 2, z / 2)
        if self.kind == "cylinder":
            r, h = self.dims_mm
            return (r, r, h / 2)
        (r,) = self.dims_mm
        return (r, r, r)


@dataclass(frozen=True)
class Component:
    """An instance of a part or sub-assembly (``ref`` names either) inside an assembly."""

    name: str
    ref: str
    placement: Placement = field(default_factory=Placement)


@dataclass(frozen=True)
class AssemblyDef:
    """An assembly product holding placed components."""

    name: str
    components: tuple[Component, ...]


@dataclass(frozen=True)
class LeafInstance:
    """A part as it appears in the flattened assembly, with its world transform in mm."""

    path: tuple[str, ...]
    part: PartDef
    transform: NDArray[np.float64]

    def bbox_mm(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """World axis-aligned bounding box (min, max) in mm."""
        h = np.asarray(self.part.half_extents_mm())
        rot = self.transform[:3, :3]
        centre = self.transform[:3, 3]
        if self.part.kind == "sphere":
            reach = h
        elif self.part.kind == "cylinder":
            # A disc of radius r with normal n reaches r * sqrt(1 - n_i^2) along axis i.
            r, half_h = h[0], h[2]
            n = rot[:, 2]
            reach = r * np.sqrt(np.clip(1.0 - n * n, 0.0, None)) + half_h * np.abs(n)
        else:
            reach = np.abs(rot) @ h
        return centre - reach, centre + reach


@dataclass(frozen=True)
class FixtureSpec:
    """A complete fixture: parts, assemblies and the root assembly's name."""

    root: str
    parts: tuple[PartDef, ...]
    assemblies: tuple[AssemblyDef, ...]

    def __post_init__(self) -> None:
        names = [p.name for p in self.parts] + [a.name for a in self.assemblies]
        if len(names) != len(set(names)):
            raise ValueError(f"duplicate product names in fixture: {names}")
        if self.root not in {a.name for a in self.assemblies}:
            raise ValueError(f"root {self.root!r} is not an assembly")
        for asm in self.assemblies:
            for comp in asm.components:
                if comp.ref not in names:
                    raise ValueError(f"{asm.name}/{comp.name} refers to unknown {comp.ref!r}")

    def part(self, name: str) -> PartDef:
        return next(p for p in self.parts if p.name == name)

    def assembly(self, name: str) -> AssemblyDef:
        return next(a for a in self.assemblies if a.name == name)

    def leaves(self) -> list[LeafInstance]:
        """Flatten the tree depth-first in component order."""
        return list(self._walk(self.root, (), np.eye(4)))

    def _walk(
        self, name: str, path: tuple[str, ...], world: NDArray[np.float64]
    ) -> Iterator[LeafInstance]:
        for comp in self.assembly(name).components:
            here = world @ comp.placement.matrix()
            sub = (*path, comp.name)
            if comp.ref in {a.name for a in self.assemblies}:
                yield from self._walk(comp.ref, sub, here)
            else:
                yield LeafInstance(sub, self.part(comp.ref), here)


# --- Ready-made fixtures ---------------------------------------------------------------------

RED = (0.8, 0.1, 0.1)
GREEN = (0.1, 0.7, 0.2)
BLUE = (0.1, 0.2, 0.8)
GREY = (0.5, 0.5, 0.5)


def two_link_arm() -> FixtureSpec:
    """Two parts: a box base and a cylindrical arm standing on it (the M1 done definition).

    The intended joint is a revolute about world z at the base top face (z = 20 mm).
    """
    base = PartDef("base", "box", (100.0, 80.0, 40.0), GREY)
    arm = PartDef("arm", "cylinder", (10.0, 150.0), RED)
    root = AssemblyDef(
        "two_link_arm",
        (
            Component("base_1", "base"),
            Component("arm_1", "arm", Placement((0.0, 0.0, 20.0 + 75.0))),
        ),
    )
    return FixtureSpec("two_link_arm", (base, arm), (root,))


def gripper_arm() -> FixtureSpec:
    """Three parts with a nested sub-assembly and a part instanced twice.

    Root ``gripper_arm`` holds the ``base`` box and the ``wrist`` sub-assembly. The wrist holds
    a ``link`` cylinder lying along x, a ``tip`` sphere and two instances of ``finger``, the
    second rotated 180 degrees about x (no mirror).
    """
    base = PartDef("base", "box", (60.0, 60.0, 20.0), GREY)
    link = PartDef("link", "cylinder", (8.0, 120.0), BLUE)
    tip = PartDef("tip", "sphere", (12.0,), GREEN)
    finger = PartDef("finger", "box", (30.0, 6.0, 10.0), RED)
    wrist = AssemblyDef(
        "wrist",
        (
            Component("link_1", "link", Placement((60.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)),
            Component("tip_1", "tip", Placement((120.0, 0.0, 0.0))),
            Component("finger_1", "finger", Placement((145.0, 8.0, 0.0))),
            Component("finger_2", "finger", Placement((145.0, -8.0, 0.0), (1.0, 0.0, 0.0), 180.0)),
        ),
    )
    root = AssemblyDef(
        "gripper_arm",
        (
            Component("base_1", "base"),
            Component("wrist_1", "wrist", Placement((0.0, 0.0, 10.0 + 8.0))),
        ),
    )
    return FixtureSpec("gripper_arm", (base, link, tip, finger), (wrist, root))


FIXTURES = {"two_link_arm": two_link_arm, "gripper_arm": gripper_arm}


# --- OCP construction and writing ------------------------------------------------------------


def make_shape(part: PartDef):  # -> TopoDS_Shape (OCP types kept out of the public signature)
    """Build the part's solid in mm, centred on its local origin."""
    from OCP.BRepPrimAPI import (
        BRepPrimAPI_MakeBox,
        BRepPrimAPI_MakeCylinder,
        BRepPrimAPI_MakeSphere,
    )
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    if part.kind == "box":
        x, y, z = part.dims_mm
        return BRepPrimAPI_MakeBox(gp_Pnt(-x / 2, -y / 2, -z / 2), x, y, z).Shape()
    if part.kind == "cylinder":
        r, h = part.dims_mm
        ax = gp_Ax2(gp_Pnt(0.0, 0.0, -h / 2), gp_Dir(0.0, 0.0, 1.0))
        return BRepPrimAPI_MakeCylinder(ax, r, h).Shape()
    (r,) = part.dims_mm
    return BRepPrimAPI_MakeSphere(gp_Pnt(0.0, 0.0, 0.0), r).Shape()


def _location(placement: Placement):  # -> TopLoc_Location
    from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
    from OCP.TopLoc import TopLoc_Location

    rot = gp_Trsf()
    if placement.angle_deg != 0.0:
        rot.SetRotation(
            gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(*placement.axis)),
            math.radians(placement.angle_deg),
        )
    move = gp_Trsf()
    move.SetTranslation(gp_Vec(*placement.translation_mm))
    return TopLoc_Location(move.Multiplied(rot))


def _set_name(label, name: str) -> None:
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name

    TDataStd_Name.Set_s(label, TCollection_ExtendedString(name))


def build_document(spec: FixtureSpec):  # -> TDocStd_Document
    """Build an XCAF document for ``spec`` (geometry in mm)."""
    from OCP.Quantity import Quantity_Color, Quantity_TypeOfColor
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_Label
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    XCAFApp_Application.GetApplication_s().InitDocument(doc)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colours = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    labels: dict[str, TDF_Label] = {}
    for part in spec.parts:
        label = shapes.AddShape(make_shape(part), False)
        _set_name(label, part.name)
        r, g, b = part.colour
        colour = Quantity_Color(r, g, b, Quantity_TypeOfColor.Quantity_TOC_RGB)
        colours.SetColor(label, colour, XCAFDoc_ColorType.XCAFDoc_ColorSurf)
        labels[part.name] = label

    # Sub-assemblies before their parents: add in dependency order.
    pending = list(spec.assemblies)
    while pending:
        ready = [a for a in pending if all(c.ref in labels for c in a.components)]
        if not ready:
            raise ValueError("assembly references form a cycle")
        for asm in ready:
            label = shapes.NewShape()
            _set_name(label, asm.name)
            for comp in asm.components:
                inst = shapes.AddComponent(label, labels[comp.ref], _location(comp.placement))
                _set_name(inst, comp.name)
            labels[asm.name] = label
            pending.remove(asm)
    shapes.UpdateAssemblies()
    return doc


def write_step(spec: FixtureSpec, path: Path | str, unit: StepUnit = "MM") -> Path:
    """Write ``spec`` as an AP214 STEP file with names and colours; return the path.

    The geometry is built in mm (OCCT's session unit); ``unit`` sets the unit declared and
    used in the file. The process-global ``xstep.cascade.unit`` is reset to ``MM`` first so a
    reader configured for metres earlier in the process does not rescale the fixture.
    """
    from OCP.IFSelect import IFSelect_ReturnStatus
    from OCP.Interface import Interface_Static
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_Controller, STEPControl_StepModelType

    path = Path(path)
    STEPControl_Controller.Init_s()
    for key, value in (
        ("xstep.cascade.unit", "MM"),
        ("write.step.unit", unit),
        ("write.step.schema", "AP214IS"),
    ):
        if not Interface_Static.SetCVal_s(key, value):
            raise RuntimeError(f"could not set {key}={value}")

    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    writer.SetColorMode(True)
    if not writer.Transfer(build_document(spec), STEPControl_StepModelType.STEPControl_AsIs):
        raise RuntimeError("STEP transfer failed")
    status = writer.Write(str(path))
    if status != IFSelect_ReturnStatus.IFSelect_RetDone:
        raise RuntimeError(f"STEP write failed: {status}")
    _fix_header(path)
    return path


_HEADER_TIME = re.compile(r"(FILE_NAME\('[^']*',')[^']*(')")


def _fix_header(path: Path) -> None:
    """Replace the header timestamp so repeated writes differ as little as possible."""
    text = path.read_text(encoding="utf-8")
    fixed, n = _HEADER_TIME.subn(rf"\g<1>{FIXED_TIMESTAMP}\g<2>", text, count=1)
    if n != 1:
        raise RuntimeError(f"unexpected STEP header in {path}")
    path.write_text(fixed, encoding="utf-8")
