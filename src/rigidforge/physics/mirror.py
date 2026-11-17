"""Mirroring geometry about a plane (M1 task 15, design doc section 6, spike 3).

A part's ``MirrorPlane`` is given in the part file's own coordinates (metres). The mirror is
baked into the geometry before mass properties are computed, and never expressed as a
negative scale:

* B-Rep: ``gp_Trsf.SetMirror`` with ``BRepBuilderAPI_Transform(copy=True)``, which rebuilds
  the solid with positive volume.
* Mesh: reflect the vertices and reverse each triangle's winding, so normals still point
  outwards.

The result is a new shape in the same (file) frame; the part's placement is applied after.
``reflection_matrix`` gives the same mirror as a 4x4 matrix, which tests and the unit-property
shortcut use: the CoM is reflected and the tensor becomes ``R I Rᵀ``.
"""

from typing import Any

import numpy as np
import numpy.typing as npt

from rigidforge.core.model import MirrorPlane

__all__ = ["mirror_mesh", "mirror_shape", "reflection_matrix"]

type Vertices = npt.NDArray[np.float64]
type Faces = npt.NDArray[np.int64]


class MirrorError(ValueError):
    """The mirror plane is degenerate."""


def _unit_normal(plane: MirrorPlane) -> npt.NDArray[np.float64]:
    n = np.asarray(plane.normal, dtype=np.float64)
    p = np.asarray(plane.point, dtype=np.float64)
    length = float(np.linalg.norm(n))
    if not (np.all(np.isfinite(n)) and np.all(np.isfinite(p)) and length > 1e-12):
        raise MirrorError(f"degenerate mirror plane: point {plane.point}, normal {plane.normal}")
    return n / length


def reflection_matrix(plane: MirrorPlane) -> npt.NDArray[np.float64]:
    """The 4x4 reflection about ``plane``: ``I - 2 n nᵀ`` on the rotation part."""
    n = _unit_normal(plane)
    p = np.asarray(plane.point, dtype=np.float64)
    out = np.eye(4)
    out[:3, :3] = np.eye(3) - 2.0 * np.outer(n, n)
    out[:3, 3] = 2.0 * float(n @ p) * n
    return out


def mirror_shape(shape: Any, plane: MirrorPlane) -> Any:
    """A mirrored copy of an OCP ``TopoDS_Shape`` (metres), about ``plane``."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf

    n = _unit_normal(plane)
    trsf = gp_Trsf()
    trsf.SetMirror(gp_Ax2(gp_Pnt(*plane.point), gp_Dir(float(n[0]), float(n[1]), float(n[2]))))
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def mirror_mesh(vertices: Vertices, faces: Faces, plane: MirrorPlane) -> tuple[Vertices, Faces]:
    """Reflect ``vertices`` about ``plane`` and reverse the winding of every face."""
    m = reflection_matrix(plane)
    v = np.asarray(vertices, dtype=np.float64) @ m[:3, :3].T + m[:3, 3]
    f = np.asarray(faces, dtype=np.int64)[:, ::-1].copy()
    return v, f
