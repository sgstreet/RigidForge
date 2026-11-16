"""Simulator-specific extension blocks (design doc section 7).

Each simulator gets its own namespaced pydantic model. Every block allows extra keys, and
so does the container, so data written by a newer RigidForge (or by hand) round-trips
instead of being dropped.
"""

from typing import Any

from pydantic import BaseModel, ConfigDict


class _ExtensionBlock(BaseModel):
    model_config = ConfigDict(extra="allow")

    def unknown_keys(self) -> dict[str, Any]:
        """Keys this version does not model, kept for round-tripping."""
        return dict(self.__pydantic_extra__ or {})


class UrdfExtension(_ExtensionBlock):
    """URDF-only data. Nothing is modelled yet."""


class MjcfPositionActuator(BaseModel):
    """An MJCF position actuator on the owning joint. Values in SI (N·m, N·m/rad)."""

    model_config = ConfigDict(extra="allow")

    kp: float
    kv: float | None = None
    forcerange: tuple[float, float] | None = None
    ctrlrange: tuple[float, float] | None = None


class MjcfExtension(_ExtensionBlock):
    """MJCF-only data, for example actuators and solver parameters."""

    position_actuator: MjcfPositionActuator | None = None


class SdfExtension(_ExtensionBlock):
    """SDF-only data, for example friction and plugins. Nothing is modelled yet."""


class UsdExtension(_ExtensionBlock):
    """USD-only data, for example drive stiffness. Nothing is modelled yet."""


class Extensions(BaseModel):
    """Container for the per-simulator blocks. Unknown namespaces round-trip as raw data."""

    model_config = ConfigDict(extra="allow")

    urdf: UrdfExtension | None = None
    mjcf: MjcfExtension | None = None
    sdf: SdfExtension | None = None
    usd: UsdExtension | None = None

    def is_empty(self) -> bool:
        return (
            self.urdf is None
            and self.mjcf is None
            and self.sdf is None
            and self.usd is None
            and not self.__pydantic_extra__
        )
