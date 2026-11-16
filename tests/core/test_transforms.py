"""Tests for Pose <-> matrix conversions."""

import math

import numpy as np
import pytest
from hypothesis import given
from hypothesis import strategies as st

from rigidforge.core.geometry import Pose
from rigidforge.core.transforms import (
    NotRigidError,
    is_rigid,
    matrix_from_pose,
    pose_from_matrix,
    quat_from_rotation,
    rotation_from_quat,
)

unit = st.floats(-1.0, 1.0, allow_nan=False)


@given(st.tuples(unit, unit, unit, unit), st.tuples(unit, unit, unit))
def test_round_trip(q: tuple[float, float, float, float], t: tuple[float, float, float]) -> None:
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-3:
        return
    quat = (q[0] / n, q[1] / n, q[2] / n, q[3] / n)
    m = matrix_from_pose(Pose(xyz=t, quat=quat))
    assert is_rigid(m)
    pose = pose_from_matrix(m)
    assert pose.quat[0] >= 0.0
    np.testing.assert_allclose(matrix_from_pose(pose), m, atol=1e-12)
    sign = 1.0 if quat[0] >= 0.0 else -1.0
    if abs(quat[0]) > 1e-9:
        np.testing.assert_allclose(pose.quat, [sign * c for c in quat], atol=1e-9)


@pytest.mark.parametrize(
    ("axis", "angle"),
    [((1, 0, 0), math.pi), ((0, 1, 0), math.pi), ((0, 0, 1), math.pi), ((1, 1, 0), 2.5)],
)
def test_large_rotations(axis: tuple[int, int, int], angle: float) -> None:
    a = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    quat = (math.cos(angle / 2), *(a * math.sin(angle / 2)))
    rot = rotation_from_quat(quat)  # type: ignore[arg-type]
    np.testing.assert_allclose(rotation_from_quat(quat_from_rotation(rot)), rot, atol=1e-12)


def test_identity() -> None:
    assert pose_from_matrix(np.eye(4)) == Pose()


@pytest.mark.parametrize(
    "bad",
    [
        np.diag([2.0, 2.0, 2.0, 1.0]),  # scale
        np.diag([-1.0, 1.0, 1.0, 1.0]),  # mirror
        np.array([[1, 0.1, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]]),  # shear
        np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 1, 1.0]]),  # projective
        np.full((4, 4), np.nan),
        np.eye(3),
    ],
)
def test_rejects_non_rigid(bad: np.ndarray) -> None:
    assert not is_rigid(bad)
    with pytest.raises(NotRigidError):
        pose_from_matrix(bad)
