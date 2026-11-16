"""Tests for exact B-Rep mass properties (M1 task 7).

Analytic primitives built in metres check ``brep_unit_mass_properties`` directly; the
synthetic STEP fixtures check the whole chain (STEP in mm or m, scaled at the import
boundary, density applied, parts merged into a link); the hexapod coxa frame checks a real
CAD export against the spike's numbers.
"""

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from rigidforge.core.model import Density, Link, RobotModel
from rigidforge.core.transforms import matrix_from_pose
from rigidforge.importers.step import candidate_parts, read_step
from rigidforge.physics.brep import brep_unit_mass_properties
from rigidforge.physics.inertia import (
    MassProperties,
    combine,
    solid_box,
    solid_cylinder,
    solid_sphere,
)
from rigidforge.physics.mass import (
    InertiaMethod,
    MassPropertiesError,
    link_mass_properties,
    part_mass_properties,
)
from rigidforge.testing.step_fixtures import FIXTURES, PartDef, write_step

HEXAPOD = Path(__file__).parent.parent / "data" / "hexapod-leg"
PETG = 1270.0


def _centred_box(x: float, y: float, z: float) -> Any:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt

    return BRepPrimAPI_MakeBox(gp_Pnt(-x / 2, -y / 2, -z / 2), x, y, z).Shape()


def _centred_cylinder(r: float, h: float) -> Any:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -h / 2), gp_Dir(0, 0, 1)), r, h).Shape()


def _sphere(r: float) -> Any:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeSphere

    return BRepPrimAPI_MakeSphere(r).Shape()


def _assert_matches(shape: Any, expected: MassProperties, rtol: float = 1e-9) -> None:
    unit = brep_unit_mass_properties(shape)
    assert unit.method is InertiaMethod.BREP_EXACT
    got = unit.at_density(PETG)
    scale = float(np.abs(expected.inertia).max())
    assert got.mass == pytest.approx(expected.mass, rel=rtol)
    np.testing.assert_allclose(got.com, expected.com, atol=1e-12)
    np.testing.assert_allclose(got.inertia, expected.inertia, rtol=rtol, atol=rtol * scale)


@pytest.mark.parametrize(
    ("make", "expected"),
    [
        (
            lambda: _centred_box(0.1, 0.08, 0.04),
            solid_box(PETG * 0.1 * 0.08 * 0.04, (0.1, 0.08, 0.04)),
        ),
        (
            lambda: _centred_cylinder(0.01, 0.15),
            solid_cylinder(PETG * np.pi * 0.01**2 * 0.15, 0.01, 0.15),
        ),
        (lambda: _sphere(0.012), solid_sphere(PETG * 4 / 3 * np.pi * 0.012**3, 0.012)),
    ],
    ids=["box", "cylinder", "sphere"],
)
def test_primitives_match_analytic(make: Any, expected: MassProperties) -> None:
    _assert_matches(make(), expected)


def test_box_with_through_hole() -> None:
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut

    x, y, z, r = 0.04, 0.03, 0.02, 0.005
    shape = BRepAlgoAPI_Cut(_centred_box(x, y, z), _centred_cylinder(r, 2 * z)).Shape()
    box = solid_box(PETG * x * y * z, (x, y, z))
    hole = solid_cylinder(PETG * np.pi * r * r * z, r, z)
    expected = MassProperties.of(box.mass - hole.mass, (0, 0, 0), box.inertia - hole.inertia)
    _assert_matches(shape, expected)


def test_located_shape_is_in_its_frame() -> None:
    """A moved shape gives the CoM and tensor of the moved body (R I Rᵀ about the new CoM)."""
    from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
    from OCP.TopLoc import TopLoc_Location

    size = (0.05, 0.02, 0.01)
    rot = gp_Trsf()
    rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 1, 0)), 0.7)
    move = gp_Trsf()
    move.SetTranslation(gp_Vec(0.1, -0.2, 0.3))
    trsf = move.Multiplied(rot)
    shape = _centred_box(*size).Moved(TopLoc_Location(trsf))
    r = np.array([[trsf.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    expected = solid_box(PETG * size[0] * size[1] * size[2], size).transformed(r, (0.1, -0.2, 0.3))
    _assert_matches(shape, expected)


def test_rejects_shapes_without_volume() -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.TopoDS import TopoDS_Shape

    with pytest.raises(MassPropertiesError, match="null shape"):
        brep_unit_mass_properties(TopoDS_Shape())
    shell = BRepPrimAPI_MakeBox(0.01, 0.01, 0.01).Shell()
    with pytest.raises(MassPropertiesError, match="no solids"):
        brep_unit_mass_properties(shell, "shell")


def test_rejects_reversed_solid() -> None:
    reversed_box = _centred_box(0.01, 0.01, 0.01).Reversed()
    with pytest.raises(MassPropertiesError, match="negative volume"):
        brep_unit_mass_properties(reversed_box)


def _expected_link(leaves: list[tuple[PartDef, np.ndarray]]) -> MassProperties:
    bodies: list[MassProperties] = []
    for part, transform in leaves:
        d = tuple(v / 1000.0 for v in part.dims_mm)
        volume = part.volume_mm3() * 1e-9
        if part.kind == "box":
            ref = solid_box(PETG * volume, (d[0], d[1], d[2]))
        elif part.kind == "cylinder":
            ref = solid_cylinder(PETG * volume, d[0], d[1])
        else:
            ref = solid_sphere(PETG * volume, d[0])
        bodies.append(ref.transformed(transform[:3, :3], transform[:3, 3] / 1000.0))
    return combine(bodies)


@pytest.mark.parametrize("fixture", sorted(FIXTURES))
@pytest.mark.parametrize("file_unit", ["MM", "M"])
def test_step_fixture_link_mass(tmp_path: Path, fixture: str, file_unit: str) -> None:
    """Import, apply density per part, merge every part into one link: matches analytic."""
    spec = FIXTURES[fixture]()
    path = write_step(spec, tmp_path / f"{fixture}.step", file_unit)  # type: ignore[arg-type]
    imported = read_step(path)
    link = Link(name="all")
    parts = candidate_parts(imported, asset=path.name, sha256="0" * 64)
    for part in parts:
        part.link = link.id
        part.mass = Density(density=PETG)
    model = RobotModel(name=fixture, links=[link], parts=parts)

    results = {}
    for part in parts:
        product = imported.product_of(imported.leaf_at(part.source.assembly_path))
        results[part.id] = part_mass_properties(
            part, brep_unit_mass_properties(product.shape, part.name)
        )
        # The part frame is the product frame, so the CoM of a centred primitive is 0.
        np.testing.assert_allclose(results[part.id].props.com, 0.0, atol=1e-12)
    got = link_mass_properties(model, link.id, results).props

    expected = _expected_link([(leaf.part, leaf.transform) for leaf in spec.leaves()])
    assert got.mass == pytest.approx(expected.mass, rel=1e-9)
    np.testing.assert_allclose(got.com, expected.com, atol=1e-12)
    scale = float(np.abs(expected.inertia).max())
    np.testing.assert_allclose(got.inertia, expected.inertia, rtol=1e-9, atol=1e-9 * scale)
    assert not got.problems()
    # Placements came through the model, not the raw leaf transforms.
    for part, leaf in zip(parts, imported.leaves, strict=True):
        np.testing.assert_allclose(matrix_from_pose(part.placement), leaf.transform, atol=1e-12)


def test_hexapod_coxa_frame() -> None:
    """4475.244 mm³ and 5.68 g as solid PETG; the tensor is valid.

    The spike's 4475.23 mm³ came from OCCT's default fixed-order integration, which is
    2.3e-6 low on this part; the adaptive scheme converges to 4475.244 mm³.
    """
    imported = read_step(HEXAPOD / "coxa-frame.step")
    (leaf,) = imported.leaves
    unit = brep_unit_mass_properties(imported.product_of(leaf).shape, "coxa-frame")
    assert unit.volume == pytest.approx(4475.244e-9, rel=1e-7)
    props = unit.at_density(PETG)
    assert props.mass == pytest.approx(5.68e-3, rel=1e-3)
    assert not props.problems()


def test_hexapod_xl330_body_products() -> None:
    """Every product of the servo assembly (case, screws, connectors) has a valid tensor."""
    imported = read_step(HEXAPOD / "xl330-Body.step")
    for product in imported.products.values():
        unit = brep_unit_mass_properties(product.shape, product.name)
        assert unit.volume > 0.0
        assert not [p for p in unit.at_density(PETG).problems() if p != "extreme_ratio"]
