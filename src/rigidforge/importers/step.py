"""STEP importer (M1 task 6): XCAF assembly tree to candidate parts, in SI.

``read_step`` reads a STEP file with ``STEPCAFControl_Reader`` on an XCAF document, so the
assembly tree, product and instance names and colours survive. Geometry is converted to
metres at this boundary and nowhere else:

* OCCT converts from the file's declared length unit to ``xstep.cascade.unit``, which is set
  to ``"M"`` after ``STEPControl_Controller.Init_s()`` (set before it, the value is silently
  ignored; design doc section 6, spike 1). The setting is process-global, so the previous
  value is restored afterwards, and reads must not run concurrently in one process.
* The unit check: the declared unit is read from the file, and the XCAF document's length
  unit must come back as 1 m after transfer. Either failing raises ``StepImportError``
  rather than returning geometry at the wrong scale.
* ``scale_to_si`` (metres per file unit) overrides a wrongly declared unit, for example an
  inch model exported with a millimetre header. The extra factor is applied to the B-Rep
  with ``gp_Trsf`` scaling, so mass properties are later computed on scaled geometry and no
  tensor is ever rescaled.

``candidate_parts`` turns the result into unassigned model ``Part``s, one per leaf instance,
with placements in the import root's frame. A product used several times (a repeated screw,
say) appears once in ``StepImport.products`` and once per instance in ``leaves``.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from rigidforge.core.model import Part, PartSource, SourceKind
from rigidforge.core.transforms import NotRigidError, pose_from_matrix

# Lower-case unit names as returned by ``STEPControl_Reader.FileUnits``, in metres.
_UNIT_METRES: dict[str, float] = {
    "metre": 1.0,
    "meter": 1.0,
    "millimetre": 1e-3,
    "millimeter": 1e-3,
    "centimetre": 1e-2,
    "centimeter": 1e-2,
    "decimetre": 1e-1,
    "decimeter": 1e-1,
    "micrometre": 1e-6,
    "micrometer": 1e-6,
    "micron": 1e-6,
    "kilometre": 1e3,
    "kilometer": 1e3,
    "inch": 0.0254,
    "foot": 0.3048,
    "feet": 0.3048,
    "mil": 2.54e-5,
    "yard": 0.9144,
    "mile": 1609.344,
}


class StepImportError(RuntimeError):
    """The file could not be read, or its unit or structure cannot be trusted."""


@dataclass(frozen=True)
class StepProduct:
    """A leaf product (a part definition), with its shape in metres and no instance placement.

    ``shape`` is an OCP ``TopoDS_Shape``; it is not picklable (see CLAUDE.md, threading).
    ``colour`` is linear RGB in 0..1, or ``None`` when the file gives none.
    """

    key: str
    name: str
    colour: tuple[float, float, float] | None
    shape: Any = field(repr=False, compare=False)


@dataclass(frozen=True)
class StepLeaf:
    """One placed instance of a product.

    ``path`` is the chain of instance names from the root (empty for a single-shape file);
    names are unique among siblings (a repeat gets ``#2``, ``#3``, ...). ``transform`` is the
    4x4 rigid transform from the product's frame to the import root frame, in metres.
    """

    path: tuple[str, ...]
    product: str
    transform: NDArray[np.float64] = field(compare=False)


@dataclass(frozen=True)
class StepImport:
    """The result of reading one STEP file."""

    path: Path
    root_name: str
    file_unit: str
    """Length unit declared in the file, as OCCT names it (for example ``"millimetre"``)."""
    file_unit_metres: float
    """Metres per declared file unit."""
    scale_to_si: float
    """Metres per file unit actually used: the declared unit or the caller's override."""
    products: dict[str, StepProduct]
    leaves: list[StepLeaf]

    def product_of(self, leaf: StepLeaf) -> StepProduct:
        return self.products[leaf.product]


def unit_metres(name: str) -> float | None:
    """Metres per unit for an OCCT/STEP length unit name, or ``None`` if unknown."""
    return _UNIT_METRES.get(name.strip().lower())


def read_step(path: Path | str, *, scale_to_si: float | None = None) -> StepImport:
    """Read ``path`` into products and placed leaves with all geometry in metres.

    ``scale_to_si`` overrides the declared unit (metres per file unit). Raises
    ``StepImportError`` if the file cannot be read, declares no usable length unit, comes
    back in a unit other than metres, or places an instance with a non-rigid transform.
    """
    from OCP.IFSelect import IFSelect_ReturnStatus
    from OCP.Interface import Interface_Static
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.STEPControl import STEPControl_Controller
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFApp import XCAFApp_Application

    path = Path(path)
    if not path.is_file():
        raise StepImportError(f"no such file: {path}")
    if scale_to_si is not None and not (np.isfinite(scale_to_si) and scale_to_si > 0.0):
        raise StepImportError(f"scale_to_si must be positive and finite, got {scale_to_si}")

    STEPControl_Controller.Init_s()
    previous = Interface_Static.CVal_s("xstep.cascade.unit")
    if not Interface_Static.SetCVal_s("xstep.cascade.unit", "M"):
        raise StepImportError("could not set xstep.cascade.unit to M")
    try:
        reader = STEPCAFControl_Reader()
        reader.SetNameMode(True)
        reader.SetColorMode(True)
        if reader.ReadFile(str(path)) != IFSelect_ReturnStatus.IFSelect_RetDone:
            raise StepImportError(f"could not read STEP file: {path}")
        file_unit = _declared_unit(reader, path)
        file_unit_metres = unit_metres(file_unit)
        if file_unit_metres is None:
            raise StepImportError(f"{path}: unknown length unit {file_unit!r}")

        doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
        XCAFApp_Application.GetApplication_s().InitDocument(doc)
        if not reader.Transfer(doc):
            raise StepImportError(f"STEP transfer failed: {path}")
        _check_document_unit(doc, path)
    finally:
        Interface_Static.SetCVal_s("xstep.cascade.unit", previous or "MM")

    used = file_unit_metres if scale_to_si is None else scale_to_si
    extra = used / file_unit_metres
    root_name, products, leaves = _walk_document(doc, path, extra)
    return StepImport(
        path=path,
        root_name=root_name,
        file_unit=file_unit,
        file_unit_metres=file_unit_metres,
        scale_to_si=used,
        products=products,
        leaves=leaves,
    )


def candidate_parts(
    imported: StepImport, asset: str, sha256: str, original_path: str | None = None
) -> list[Part]:
    """One unassigned ``Part`` per leaf, placed in the import root frame (metres).

    ``asset`` is the file's path relative to the project's ``assets/`` directory. Mass is
    left unset; it comes from density or a measured mass later (tasks 7 and 17).
    """
    parts: list[Part] = []
    for leaf in imported.leaves:
        source = PartSource(
            kind=SourceKind.STEP,
            asset=asset,
            sha256=sha256,
            scale_to_si=imported.scale_to_si,
            assembly_path=list(leaf.path),
            original_path=original_path,
        )
        parts.append(
            Part(
                name=imported.product_of(leaf).name,
                source=source,
                placement=pose_from_matrix(leaf.transform),
            )
        )
    return parts


# --- Internals ---------------------------------------------------------------------------


def _declared_unit(reader: Any, path: Path) -> str:
    from OCP.collections import Sequence_TCollection_AsciiString

    lengths = Sequence_TCollection_AsciiString()
    angles = Sequence_TCollection_AsciiString()
    solids = Sequence_TCollection_AsciiString()
    reader.ChangeReader().FileUnits(lengths, angles, solids)
    names = {lengths.Value(i).ToCString() for i in range(1, lengths.Length() + 1)}
    if not names:
        raise StepImportError(f"{path}: the file declares no length unit")
    if len(names) > 1:
        raise StepImportError(f"{path}: the file declares several length units {sorted(names)}")
    return names.pop()


def _check_document_unit(doc: Any, path: Path) -> None:
    from OCP.XCAFDoc import XCAFDoc_LengthUnit

    attr = XCAFDoc_LengthUnit()
    if not doc.Main().Root().FindAttribute(XCAFDoc_LengthUnit.GetID_s(), attr):
        raise StepImportError(f"{path}: XCAF document has no length unit after transfer")
    value = attr.GetUnitValue()
    if abs(value - 1.0) > 1e-12:
        raise StepImportError(
            f"{path}: geometry came back in {attr.GetUnitName().ToCString()!r} "
            f"({value} m), not metres; xstep.cascade.unit was not honoured"
        )


def _name(label: Any) -> str:
    from OCP.TDataStd import TDataStd_Name

    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def _entry(label: Any) -> str:
    from OCP.TCollection import TCollection_AsciiString
    from OCP.TDF import TDF_Tool

    out = TCollection_AsciiString()
    TDF_Tool.Entry_s(label, out)
    return out.ToCString()


def _matrix(location: Any) -> NDArray[np.float64]:
    trsf = location.Transformation()
    out = np.eye(4)
    for i in range(3):
        for j in range(4):
            out[i, j] = trsf.Value(i + 1, j + 1)
    return out


def _colour(label: Any) -> tuple[float, float, float] | None:
    from OCP.Quantity import Quantity_Color, Quantity_TypeOfColor
    from OCP.XCAFDoc import XCAFDoc_ColorTool, XCAFDoc_ColorType

    colour = Quantity_Color()
    for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        if XCAFDoc_ColorTool.GetColor_s(label, kind, colour):
            r, g, b = colour.Values(Quantity_TypeOfColor.Quantity_TOC_RGB)
            return (float(r), float(g), float(b))
    return None


def _scaled(shape: Any, factor: float) -> Any:
    if factor == 1.0:
        return shape
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Pnt, gp_Trsf

    trsf = gp_Trsf()
    trsf.SetScale(gp_Pnt(0.0, 0.0, 0.0), factor)
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def _unique(names: list[str]) -> list[str]:
    """Make sibling names unique: the second ``a`` becomes ``a#2``, and so on."""
    seen: Counter[str] = Counter()
    out: list[str] = []
    for name in names:
        seen[name] += 1
        out.append(name if seen[name] == 1 else f"{name}#{seen[name]}")
    return out


def _walk_document(
    doc: Any, path: Path, extra: float
) -> tuple[str, dict[str, StepProduct], list[StepLeaf]]:
    from OCP.collections import Sequence_TDF_Label
    from OCP.TDF import TDF_Label
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool

    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    free = Sequence_TDF_Label()
    shapes.GetFreeShapes(free)
    roots = [free.Value(i) for i in range(1, free.Length() + 1)]
    if not roots:
        raise StepImportError(f"{path}: the file contains no shapes")

    products: dict[str, StepProduct] = {}
    leaves: list[StepLeaf] = []

    def product(label: Any) -> str:
        key = _entry(label)
        if key not in products:
            shape = XCAFDoc_ShapeTool.GetShape_s(label)
            products[key] = StepProduct(
                key, _name(label) or path.stem, _colour(label), _scaled(shape, extra)
            )
        return key

    def add_leaf(label: Any, sub: tuple[str, ...], world: NDArray[np.float64]) -> None:
        try:
            pose_from_matrix(world)
        except NotRigidError as exc:
            raise StepImportError(f"{path}: instance {'/'.join(sub)} is not rigid") from exc
        leaves.append(StepLeaf(sub, product(label), world))

    def walk(label: Any, sub: tuple[str, ...], world: NDArray[np.float64]) -> None:
        comps = Sequence_TDF_Label()
        XCAFDoc_ShapeTool.GetComponents_s(label, comps, False)
        items: list[tuple[str, Any, NDArray[np.float64]]] = []
        for i in range(1, comps.Length() + 1):
            comp = comps.Value(i)
            ref = TDF_Label()
            if not XCAFDoc_ShapeTool.GetReferredShape_s(comp, ref):
                raise StepImportError(f"{path}: component without a referred shape")
            local = _matrix(XCAFDoc_ShapeTool.GetLocation_s(comp))
            local[:3, 3] *= extra
            items.append((_name(comp) or _name(ref), ref, local))
        for name, (_, ref, local) in zip(_unique([n for n, _, _ in items]), items, strict=True):
            here = world @ local
            if XCAFDoc_ShapeTool.IsAssembly_s(ref):
                walk(ref, (*sub, name), here)
            else:
                add_leaf(ref, (*sub, name), here)

    if len(roots) == 1:
        root = roots[0]
        root_name = _name(root) or path.stem
        if XCAFDoc_ShapeTool.IsAssembly_s(root):
            walk(root, (), np.eye(4))
        else:
            add_leaf(root, (), np.eye(4))
    else:
        root_name = path.stem
        names = _unique([_name(r) or path.stem for r in roots])
        for name, root in zip(names, roots, strict=True):
            if XCAFDoc_ShapeTool.IsAssembly_s(root):
                walk(root, (name,), np.eye(4))
            else:
                add_leaf(root, (name,), np.eye(4))
    return root_name, products, leaves
