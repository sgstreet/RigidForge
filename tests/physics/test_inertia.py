"""Tests for analytic inertia oracles, mass-property algebra and checks (M1 task 4)."""

import math
from typing import TYPE_CHECKING

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from rigidforge.core import InertiaTensor
from rigidforge.core.inertia import (
    EXTREME_RATIO,
    InertiaProblem,
    check_inertia,
    matrix_to_tensor,
    tensor_to_matrix,
)
from rigidforge.physics.inertia import (
    MassProperties,
    combine,
    parallel_axis,
    quat_to_matrix,
    rotate_inertia,
    solid_box,
    solid_cylinder,
    solid_sphere,
)

if TYPE_CHECKING:
    from OCP.TopoDS import TopoDS_Shape

RTOL = 1e-6  # milestones.md: analytic comparisons at 1e-6 relative

# --- Oracles against OCCT's exact B-Rep integration -----------------------------------------


def _brep_props(shape: "TopoDS_Shape", density: float) -> MassProperties:
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    volume = props.Mass()
    c = props.CentreOfMass()
    m = props.MatrixOfInertia()  # about the CoM, for unit density
    inertia = np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    return MassProperties.of(density * volume, (c.X(), c.Y(), c.Z()), density * inertia)


DENSITY = 1270.0  # PETG, kg/m³


def test_box_matches_brep() -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt

    x, y, z = 0.04, 0.025, 0.012
    shape = BRepPrimAPI_MakeBox(gp_Pnt(-x / 2, -y / 2, -z / 2), x, y, z).Shape()
    brep = _brep_props(shape, DENSITY)
    oracle = solid_box(DENSITY * x * y * z, (x, y, z))
    assert oracle.allclose(brep, rtol=RTOL, atol=1e-15)


def test_cylinder_matches_brep() -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    r, h = 0.011, 0.034
    shape = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -h / 2), gp_Dir(0, 0, 1)), r, h).Shape()
    brep = _brep_props(shape, DENSITY)
    oracle = solid_cylinder(DENSITY * math.pi * r * r * h, r, h)
    assert oracle.allclose(brep, rtol=RTOL, atol=1e-15)


def test_sphere_matches_brep() -> None:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeSphere

    r = 0.015
    brep = _brep_props(BRepPrimAPI_MakeSphere(r).Shape(), DENSITY)
    oracle = solid_sphere(DENSITY * 4.0 / 3.0 * math.pi * r**3, r)
    assert oracle.allclose(brep, rtol=RTOL, atol=1e-15)


@pytest.mark.parametrize(
    "props",
    [
        solid_box(0.2, (0.1, 0.05, 0.02)),
        solid_cylinder(0.3, 0.02, 0.1),
        solid_cylinder(0.3, 0.1, 0.001),  # a disc
        solid_sphere(1.5, 0.04),
    ],
    ids=["box", "cylinder", "disc", "sphere"],
)
def test_oracles_are_physically_valid(props: MassProperties) -> None:
    assert props.problems() == ()


# --- Algebra ---------------------------------------------------------------------------------


def test_two_boxes_combine_into_one() -> None:
    """Two 1x1x1 boxes side by side along x equal one 2x1x1 box of twice the mass."""
    half = solid_box(1.0, (1.0, 1.0, 1.0))
    left = half.transformed(np.eye(3), (-0.5, 0.0, 0.0))
    right = half.transformed(np.eye(3), (0.5, 0.0, 0.0))
    assert combine([left, right]).allclose(solid_box(2.0, (2.0, 1.0, 1.0)), atol=1e-15)


def test_stacked_cylinders_combine_into_one() -> None:
    a = solid_cylinder(1.0, 0.02, 0.06).transformed(np.eye(3), (0.0, 0.0, 0.03))
    b = solid_cylinder(2.0, 0.02, 0.12).transformed(np.eye(3), (0.0, 0.0, -0.06))
    whole = solid_cylinder(3.0, 0.02, 0.18).transformed(np.eye(3), (0.0, 0.0, -0.03))
    assert combine([a, b]).allclose(whole, atol=1e-15)


def test_parallel_axis_point_mass() -> None:
    # A point mass m at distance d along x: I about origin has Iyy = Izz = m d², Ixx = 0.
    i = parallel_axis(np.zeros((3, 3)), 2.0, (3.0, 0.0, 0.0))
    assert np.allclose(i, np.diag([0.0, 18.0, 18.0]))


def test_about_point_shifts_away_from_com() -> None:
    rod = solid_cylinder(1.0, 0.0, 2.0)  # thin rod along z, length 2
    tip = rod.about_point((0.0, 0.0, 1.0))
    assert np.allclose(tip, np.diag([4.0 / 3.0, 4.0 / 3.0, 0.0]))  # m L² / 3


def test_rotation_permutes_box_axes() -> None:
    box = solid_box(1.0, (0.3, 0.2, 0.1))
    r = quat_to_matrix((math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)))  # 90° about z
    rotated = rotate_inertia(box.inertia, r)
    assert np.allclose(rotated, np.diag([box.inertia[1, 1], box.inertia[0, 0], box.inertia[2, 2]]))


def test_quat_to_matrix_is_orthonormal_and_normalises() -> None:
    r = quat_to_matrix((2.0, 0.4, -1.0, 0.6))
    assert np.allclose(r @ r.T, np.eye(3))
    assert math.isclose(float(np.linalg.det(r)), 1.0)
    with pytest.raises(ValueError):
        quat_to_matrix((0.0, 0.0, 0.0, 0.0))


def test_transformed_moves_com() -> None:
    sphere = solid_sphere(1.0, 0.1).transformed(np.eye(3), (1.0, 2.0, 3.0))
    assert sphere.com_tuple == (1.0, 2.0, 3.0)


def test_combine_rejects_empty_and_massless() -> None:
    with pytest.raises(ValueError):
        combine([])
    with pytest.raises(ValueError):
        combine([MassProperties.of(0.0, (0, 0, 0), np.zeros((3, 3)))])


def test_tensor_round_trip_and_ordering() -> None:
    t = InertiaTensor(ixx=1.0, iyy=2.0, izz=3.0, ixy=0.1, ixz=0.2, iyz=0.3)
    m = tensor_to_matrix(t)
    assert m[0, 1] == m[1, 0] == 0.1
    assert m[0, 2] == m[2, 0] == 0.2
    assert m[1, 2] == m[2, 1] == 0.3
    assert matrix_to_tensor(m) == t
    with pytest.raises(ValueError):
        matrix_to_tensor(np.zeros((2, 2)))


# --- Checks ----------------------------------------------------------------------------------


def test_flat_plate_passes_triangle_inequality_at_equality() -> None:
    plate = solid_box(1.0, (0.1, 0.07, 0.0))  # Ixx + Iyy == Izz exactly
    assert InertiaProblem.TRIANGLE_INEQUALITY not in plate.problems()


@pytest.mark.parametrize(
    ("mass", "inertia", "problem"),
    [
        (0.0, np.eye(3), InertiaProblem.NON_POSITIVE_MASS),
        (-1.0, np.eye(3), InertiaProblem.NON_POSITIVE_MASS),
        (math.nan, np.eye(3), InertiaProblem.NON_POSITIVE_MASS),
        (1.0, np.diag([1.0, math.inf, 1.0]), InertiaProblem.NOT_FINITE),
        (1.0, [[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], InertiaProblem.NOT_SYMMETRIC),
        (1.0, np.diag([1.0, 1.0, 0.0]), InertiaProblem.NOT_POSITIVE_DEFINITE),
        (1.0, np.diag([1.0, 1.0, -1.0]), InertiaProblem.NOT_POSITIVE_DEFINITE),
        (1.0, np.diag([1.0, 1.0, 2.1]), InertiaProblem.TRIANGLE_INEQUALITY),
        (1.0, np.diag([1.0, 1.0, 1e-6]), InertiaProblem.EXTREME_RATIO),
    ],
    ids=[
        "zero mass",
        "negative mass",
        "NaN mass",
        "inf tensor",
        "asymmetric",
        "singular",
        "indefinite",
        "triangle",
        "extreme ratio",
    ],
)
def test_check_inertia_flags(mass: float, inertia: object, problem: InertiaProblem) -> None:
    assert problem in check_inertia(mass, inertia)  # type: ignore[arg-type]


def test_check_inertia_accepts_tensor_model_and_rejects_bad_shape() -> None:
    assert check_inertia(1.0, InertiaTensor(ixx=1.0, iyy=1.0, izz=1.0)) == ()
    with pytest.raises(ValueError):
        check_inertia(1.0, np.eye(2))


def test_extreme_ratio_threshold() -> None:
    assert check_inertia(1.0, np.diag([1.0, 1.0, 1.0 / (EXTREME_RATIO * 0.9)])) == ()


# --- Property tests --------------------------------------------------------------------------

_positive = st.floats(min_value=1e-3, max_value=1e3, allow_nan=False, allow_infinity=False)
_coord = st.floats(min_value=-1.0, max_value=1.0, allow_nan=False, allow_infinity=False)
_quat = st.tuples(_coord, _coord, _coord, _coord).filter(lambda q: sum(v * v for v in q) > 1e-3)


@st.composite
def _bodies(draw: st.DrawFn) -> MassProperties:
    mass = draw(_positive)
    kind = draw(st.sampled_from(["box", "cylinder", "sphere"]))
    a, b, c = (draw(_positive) * 1e-3 for _ in range(3))
    match kind:
        case "box":
            body = solid_box(mass, (a, b, c))
        case "cylinder":
            body = solid_cylinder(mass, a, b)
        case _:
            body = solid_sphere(mass, a)
    rotation = quat_to_matrix(draw(_quat))
    offset = (draw(_coord), draw(_coord), draw(_coord))
    return body.transformed(rotation, offset)


def _errors(props: MassProperties) -> set[InertiaProblem]:
    return set(props.problems()) - {InertiaProblem.EXTREME_RATIO}


@settings(max_examples=200, deadline=None)
@given(_bodies())
def test_rotated_bodies_stay_valid(body: MassProperties) -> None:
    assert _errors(body) == set()


@settings(max_examples=200, deadline=None)
@given(st.lists(_bodies(), min_size=1, max_size=5))
def test_combined_bodies_stay_valid(bodies: list[MassProperties]) -> None:
    merged = combine(bodies)
    assert _errors(merged) == set()
    assert math.isclose(merged.mass, sum(b.mass for b in bodies))


@settings(max_examples=100, deadline=None)
@given(_bodies(), _quat, st.tuples(_coord, _coord, _coord))
def test_combine_commutes_with_frame_change(
    body: MassProperties, quat: tuple[float, float, float, float], t: tuple[float, float, float]
) -> None:
    """Merging then moving equals moving then merging."""
    other = solid_sphere(0.5, 0.01).transformed(np.eye(3), (0.1, -0.2, 0.05))
    r = quat_to_matrix(quat)
    a = combine([body, other]).transformed(r, t)
    b = combine([body.transformed(r, t), other.transformed(r, t)])
    assert a.allclose(b, rtol=1e-9, atol=1e-12)
