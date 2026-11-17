"""Helpers shared by the exporters. Each format is still generated directly from the model."""

import math
from dataclasses import dataclass, field

from rigidforge.core.geometry import Pose

__all__ = ["MeshRef", "format_number"]


@dataclass(frozen=True, slots=True)
class MeshRef:
    """A mesh file for a link's visual or collision geometry, in metres.

    ``uri`` is written as given (for example ``meshes/coxa.stl`` or ``package://...``).
    ``origin`` places the mesh file's frame in the link frame.
    """

    uri: str
    origin: Pose = field(default_factory=Pose)
    rgba: tuple[float, float, float, float] | None = None


def format_number(x: float) -> str:
    """Deterministic float text: ``repr``, with ``-0.0`` written as ``0.0``.

    Raises ``ValueError`` for NaN and infinity.
    """
    if not math.isfinite(x):
        raise ValueError(f"non-finite number {x!r}")
    return repr(0.0 if x == 0.0 else float(x))
