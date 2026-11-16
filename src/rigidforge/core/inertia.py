"""Physical plausibility checks for mass properties (design doc section 4, item 4).

These checks are shared by model validation (for inertial overrides) and the physics code
(for computed mass properties), so they live in the headless core and depend only on numpy.

A rigid body's inertia tensor about its centre of mass must be:

* finite and symmetric,
* positive definite (every principal moment > 0), and
* satisfy the triangle inequality on its principal moments: each moment is at most the sum
  of the other two. MuJoCo rejects tensors that violate it.

The triangle inequality holds with equality for an ideal flat plate (Ia + Ib = Ic), so the
check allows a small relative slack for rounding.
"""

import math
from enum import StrEnum

import numpy as np
import numpy.typing as npt

from rigidforge.core.geometry import InertiaTensor

type Matrix3 = npt.NDArray[np.float64]
"""A 3x3 float64 array."""

SYMMETRY_RTOL = 1e-9
"""Allowed asymmetry, relative to the largest absolute tensor component."""

TRIANGLE_RTOL = 1e-9
"""Allowed triangle-inequality violation, relative to the trace."""

EXTREME_RATIO = 1e5
"""Largest/smallest principal moment ratio above which a warning is raised."""


class InertiaProblem(StrEnum):
    """What is wrong with a set of mass properties. Values are stable identifiers."""

    NON_POSITIVE_MASS = "non_positive_mass"
    NOT_FINITE = "not_finite"
    NOT_SYMMETRIC = "not_symmetric"
    NOT_POSITIVE_DEFINITE = "not_positive_definite"
    TRIANGLE_INEQUALITY = "triangle_inequality"
    EXTREME_RATIO = "extreme_ratio"
    """Not an error: principal moments differ by more than :data:`EXTREME_RATIO`."""


def tensor_to_matrix(tensor: InertiaTensor) -> Matrix3:
    return np.array(tensor.as_matrix(), dtype=np.float64)


def matrix_to_tensor(matrix: npt.ArrayLike) -> InertiaTensor:
    """Convert a 3x3 array to an :class:`InertiaTensor`, symmetrising it.

    The off-diagonal terms are averaged, so call :func:`check_inertia` on the array first
    if asymmetry must be reported rather than hidden.
    """
    m = np.asarray(matrix, dtype=np.float64)
    if m.shape != (3, 3):
        raise ValueError(f"expected a 3x3 matrix, got shape {m.shape}")
    s = 0.5 * (m + m.T)
    return InertiaTensor(
        ixx=float(s[0, 0]),
        iyy=float(s[1, 1]),
        izz=float(s[2, 2]),
        ixy=float(s[0, 1]),
        ixz=float(s[0, 2]),
        iyz=float(s[1, 2]),
    )


def principal_moments(inertia: npt.ArrayLike) -> npt.NDArray[np.float64]:
    """Principal moments in ascending order (eigenvalues of the symmetrised tensor)."""
    m = np.asarray(inertia, dtype=np.float64)
    return np.linalg.eigvalsh(0.5 * (m + m.T)).astype(np.float64)


def check_inertia(
    mass: float, inertia: InertiaTensor | npt.ArrayLike
) -> tuple[InertiaProblem, ...]:
    """Return every problem with ``mass`` and the CoM ``inertia`` tensor, errors first.

    An empty tuple means the mass properties are physically valid. Only
    :attr:`InertiaProblem.EXTREME_RATIO` is a warning; everything else is an error.
    """
    problems: list[InertiaProblem] = []
    if not (math.isfinite(mass) and mass > 0.0):
        problems.append(InertiaProblem.NON_POSITIVE_MASS)

    m = (
        tensor_to_matrix(inertia)
        if isinstance(inertia, InertiaTensor)
        else np.asarray(inertia, dtype=np.float64)
    )
    if m.shape != (3, 3):
        raise ValueError(f"expected a 3x3 inertia tensor, got shape {m.shape}")
    if not bool(np.all(np.isfinite(m))):
        problems.append(InertiaProblem.NOT_FINITE)
        return tuple(problems)

    scale = float(np.max(np.abs(m)))
    if scale > 0.0 and float(np.max(np.abs(m - m.T))) > SYMMETRY_RTOL * scale:
        problems.append(InertiaProblem.NOT_SYMMETRIC)
        return tuple(problems)

    moments = principal_moments(m)
    smallest, middle, largest = (float(v) for v in moments)
    if smallest <= 0.0:
        problems.append(InertiaProblem.NOT_POSITIVE_DEFINITE)
        return tuple(problems)

    # With the moments sorted, only largest <= smallest + middle can fail.
    if largest - (smallest + middle) > TRIANGLE_RTOL * (smallest + middle + largest):
        problems.append(InertiaProblem.TRIANGLE_INEQUALITY)

    if largest / smallest > EXTREME_RATIO:
        problems.append(InertiaProblem.EXTREME_RATIO)
    return tuple(problems)
