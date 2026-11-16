"""Analytic inertia oracles and rigid-body mass-property algebra (SI units throughout).

* Oracles: solid box, cylinder and sphere, used as test references for the B-Rep and mesh
  mass-property code.
* Algebra: rotate a tensor, move mass properties into another frame, shift a CoM tensor to
  another point (parallel-axis theorem), and merge several bodies into one link.

Following design doc section 4, inertia is always held about the centre of mass. The
parallel-axis shift is only used when merging parts into one link, where each part's tensor
moves to the combined CoM. Tensors are never rescaled for unit changes: geometry is
converted to SI before mass properties are computed.
"""

from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from rigidforge.core.geometry import InertiaTensor, Quat, Vec3
from rigidforge.core.inertia import (
    InertiaProblem,
    Matrix3,
    check_inertia,
    matrix_to_tensor,
    principal_moments,
    tensor_to_matrix,
)

__all__ = [
    "InertiaProblem",
    "MassProperties",
    "check_inertia",
    "combine",
    "parallel_axis",
    "principal_moments",
    "quat_to_matrix",
    "rotate_inertia",
    "solid_box",
    "solid_cylinder",
    "solid_sphere",
]

type Vector3 = npt.NDArray[np.float64]


def _vec(v: npt.ArrayLike) -> Vector3:
    a = np.asarray(v, dtype=np.float64)
    if a.shape != (3,):
        raise ValueError(f"expected a 3-vector, got shape {a.shape}")
    return a


def _mat(m: npt.ArrayLike) -> Matrix3:
    a = np.asarray(m, dtype=np.float64)
    if a.shape != (3, 3):
        raise ValueError(f"expected a 3x3 matrix, got shape {a.shape}")
    return a


@dataclass(frozen=True, slots=True, eq=False)
class MassProperties:
    """Mass (kg), centre of mass (m) and the 3x3 inertia tensor about the CoM (kg·m²).

    ``com`` and the tensor axes are expressed in the same reference frame.
    """

    mass: float
    com: Vector3
    inertia: Matrix3

    @classmethod
    def of(cls, mass: float, com: npt.ArrayLike, inertia: npt.ArrayLike) -> "MassProperties":
        return cls(float(mass), _vec(com).copy(), _mat(inertia).copy())

    @property
    def com_tuple(self) -> Vec3:
        return (float(self.com[0]), float(self.com[1]), float(self.com[2]))

    def tensor(self) -> InertiaTensor:
        return matrix_to_tensor(self.inertia)

    def problems(self) -> tuple[InertiaProblem, ...]:
        return check_inertia(self.mass, self.inertia)

    def transformed(self, rotation: npt.ArrayLike, translation: npt.ArrayLike) -> "MassProperties":
        """Express these properties in a parent frame where ``x_parent = R x + t``."""
        r = _mat(rotation)
        return MassProperties.of(
            self.mass, r @ self.com + _vec(translation), rotate_inertia(self.inertia, r)
        )

    def about_point(self, point: npt.ArrayLike) -> Matrix3:
        """The inertia tensor about ``point`` (same frame), by the parallel-axis theorem."""
        return parallel_axis(self.inertia, self.mass, self.com - _vec(point))

    def allclose(self, other: "MassProperties", rtol: float = 1e-9, atol: float = 0.0) -> bool:
        return (
            bool(np.isclose(self.mass, other.mass, rtol=rtol, atol=atol))
            and bool(np.allclose(self.com, other.com, rtol=rtol, atol=atol))
            and bool(np.allclose(self.inertia, other.inertia, rtol=rtol, atol=atol))
        )


# --- Oracles --------------------------------------------------------------------------------


def solid_box(mass: float, size: Vec3) -> MassProperties:
    """A uniform box with edge lengths ``size`` (x, y, z), centred on the origin."""
    x, y, z = size
    k = mass / 12.0
    return MassProperties.of(
        mass,
        (0.0, 0.0, 0.0),
        np.diag([k * (y * y + z * z), k * (x * x + z * z), k * (x * x + y * y)]),
    )


def solid_cylinder(mass: float, radius: float, length: float) -> MassProperties:
    """A uniform cylinder along z, centred on the origin."""
    side = mass * (3.0 * radius * radius + length * length) / 12.0
    return MassProperties.of(
        mass, (0.0, 0.0, 0.0), np.diag([side, side, 0.5 * mass * radius * radius])
    )


def solid_sphere(mass: float, radius: float) -> MassProperties:
    """A uniform sphere centred on the origin."""
    i = 0.4 * mass * radius * radius
    return MassProperties.of(mass, (0.0, 0.0, 0.0), np.diag([i, i, i]))


# --- Algebra --------------------------------------------------------------------------------


def quat_to_matrix(quat: Quat) -> Matrix3:
    """Rotation matrix of a scalar-first quaternion (w, x, y, z). The input is normalised."""
    q = np.asarray(quat, dtype=np.float64)
    n = float(np.linalg.norm(q))
    if not n > 0.0:
        raise ValueError("quaternion has zero length")
    w, x, y, z = q / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotate_inertia(inertia: npt.ArrayLike | InertiaTensor, rotation: npt.ArrayLike) -> Matrix3:
    """Re-express a tensor in a rotated frame: R I Rᵀ, where R maps body axes to the new frame."""
    i = tensor_to_matrix(inertia) if isinstance(inertia, InertiaTensor) else _mat(inertia)
    r = _mat(rotation)
    return r @ i @ r.T


def parallel_axis(inertia_com: npt.ArrayLike, mass: float, offset: npt.ArrayLike) -> Matrix3:
    """Shift a CoM tensor to a point: I_P = I_com + m (|d|² E - d dᵀ), with d = com - P.

    The sign of ``offset`` does not matter. Only valid for a tensor about the CoM.
    """
    d = _vec(offset)
    return _mat(inertia_com) + mass * (float(d @ d) * np.eye(3) - np.outer(d, d))


def combine(bodies: Iterable[MassProperties]) -> MassProperties:
    """Merge bodies (all in one frame) into one: total mass, combined CoM, tensor about it."""
    items = list(bodies)
    if not items:
        raise ValueError("cannot combine zero bodies")
    mass = sum(b.mass for b in items)
    if not mass > 0.0:
        raise ValueError("combined mass must be > 0")
    com = sum((b.mass * b.com for b in items), np.zeros(3)) / mass
    inertia = sum((b.about_point(com) for b in items), np.zeros((3, 3)))
    return MassProperties.of(mass, com, inertia)
