"""Conversions between ``Pose`` and 4x4 homogeneous matrices (SI, scalar-first quaternions)."""

import math

import numpy as np
from numpy.typing import NDArray

from rigidforge.core.geometry import Pose, Quat

RIGID_TOLERANCE = 1e-9
"""Largest deviation from an orthonormal, right-handed rotation accepted as rigid."""


class NotRigidError(ValueError):
    """A transform has scale, shear or a reflection, so it is not a rigid placement."""


def is_rigid(matrix: NDArray[np.float64], tol: float = RIGID_TOLERANCE) -> bool:
    """True if ``matrix`` is a 4x4 rotation plus translation (no scale, shear or mirror)."""
    m = np.asarray(matrix, dtype=np.float64)
    if m.shape != (4, 4) or not np.all(np.isfinite(m)):
        return False
    if not np.allclose(m[3], (0.0, 0.0, 0.0, 1.0), rtol=0.0, atol=tol):
        return False
    rot = m[:3, :3]
    orthonormal = np.allclose(rot.T @ rot, np.eye(3), rtol=0.0, atol=tol)
    return bool(orthonormal and float(np.linalg.det(rot)) > 0.0)


def quat_from_rotation(rot: NDArray[np.float64]) -> Quat:
    """Scalar-first unit quaternion for a proper rotation matrix (Shepperd's method).

    The sign is fixed so that ``w >= 0``, which keeps saved files stable.
    """
    r = np.asarray(rot, dtype=np.float64)
    trace = float(r[0, 0] + r[1, 1] + r[2, 2])
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * s
        x = float(r[2, 1] - r[1, 2]) / s
        y = float(r[0, 2] - r[2, 0]) / s
        z = float(r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + float(r[0, 0] - r[1, 1] - r[2, 2]))
        w = float(r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = float(r[0, 1] + r[1, 0]) / s
        z = float(r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + float(r[1, 1] - r[0, 0] - r[2, 2]))
        w = float(r[0, 2] - r[2, 0]) / s
        x = float(r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = float(r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + float(r[2, 2] - r[0, 0] - r[1, 1]))
        w = float(r[1, 0] - r[0, 1]) / s
        x = float(r[0, 2] + r[2, 0]) / s
        y = float(r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z)
    sign = -1.0 if w < 0.0 else 1.0
    return (sign * w / n, sign * x / n, sign * y / n, sign * z / n)


def rotation_from_quat(quat: Quat) -> NDArray[np.float64]:
    """Rotation matrix for a scalar-first quaternion (normalised first)."""
    w, x, y, z = quat
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose_from_matrix(matrix: NDArray[np.float64]) -> Pose:
    """The ``Pose`` of a rigid 4x4 transform. Raises ``NotRigidError`` otherwise."""
    m = np.asarray(matrix, dtype=np.float64)
    if not is_rigid(m):
        raise NotRigidError(f"transform is not rigid:\n{m}")
    t = m[:3, 3]
    return Pose(xyz=(float(t[0]), float(t[1]), float(t[2])), quat=quat_from_rotation(m[:3, :3]))


def matrix_from_pose(pose: Pose) -> NDArray[np.float64]:
    """The 4x4 homogeneous transform of a ``Pose``."""
    out = np.eye(4)
    out[:3, :3] = rotation_from_quat(pose.quat)
    out[:3, 3] = pose.xyz
    return out
