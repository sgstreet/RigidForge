"""Small geometric value types used by the model. All lengths are metres (SI)."""

from typing import Self

from pydantic import BaseModel, ConfigDict

type Vec3 = tuple[float, float, float]
"""A 3-vector in metres (positions) or unitless (directions)."""

type Quat = tuple[float, float, float, float]
"""A rotation quaternion, scalar first: (w, x, y, z)."""

IDENTITY_QUAT: Quat = (1.0, 0.0, 0.0, 0.0)
ZERO_VEC3: Vec3 = (0.0, 0.0, 0.0)


class Pose(BaseModel):
    """A rigid transform: translation in metres, then rotation as a scalar-first quaternion.

    The model stores numbers as given. Checking that the quaternion is unit length and the
    values are finite belongs to validation, so a bad file can still be loaded and reported.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    xyz: Vec3 = ZERO_VEC3
    quat: Quat = IDENTITY_QUAT

    @classmethod
    def identity(cls) -> Self:
        return cls()


class InertiaTensor(BaseModel):
    """A symmetric 3x3 inertia tensor in kg·m², about the centre of mass.

    Components follow the usual matrix layout: the tensor is
    [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]. Exporters reorder these as each
    format requires (MJCF fullinertia is ixx iyy izz ixy ixz iyz).
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    ixx: float
    iyy: float
    izz: float
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0

    def as_matrix(self) -> tuple[Vec3, Vec3, Vec3]:
        return (
            (self.ixx, self.ixy, self.ixz),
            (self.ixy, self.iyy, self.iyz),
            (self.ixz, self.iyz, self.izz),
        )
