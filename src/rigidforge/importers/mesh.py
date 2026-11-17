"""Mesh importer (M1 task 8): STL, OBJ, DAE and 3MF files to candidate parts, in SI.

``read_mesh`` loads a file with trimesh as a scene and returns one :class:`MeshObject` per
placed mesh, with vertices in metres in the object's own frame and a rigid placement in the
file's root frame:

* **3MF** declares a model unit (``unit="millimeter"``, default millimetre). Build-item and
  component transforms are composed by trimesh into the scene graph; this module takes them
  as the object's placement. Every object becomes a candidate part, so a slicer plate with
  three printed pieces gives three parts (spike on real Bambu Studio files, design doc
  section 6, item 8).
* **STL and OBJ** carry no unit. The caller gives ``scale_to_si`` (metres per file unit);
  the default is 1.0, which is correct only for metre files, and the result records
  ``file_unit = None`` so the UI can ask.
* **DAE** declares ``<unit meter=...>``; it is read here rather than trusting the loader.

Geometry is converted to metres at this boundary and never rescaled afterwards. Translation
in a placement is scaled with the geometry. A placement with scale, shear or a mirror is not
rigid, so it is baked into the vertices instead (with the winding flipped for a mirror) and
the object gets an identity placement.

Vertices are welded before the closed-mesh check. A mesh that is not a closed, outward-wound
solid is still imported, with ``problems`` set, so the UI can report it; mass properties
refuse such a mesh (``rigidforge.physics.mesh``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from rigidforge.core.model import Part, PartSource, SourceKind
from rigidforge.core.transforms import pose_from_matrix
from rigidforge.importers.step import unit_metres
from rigidforge.physics.mesh import closed_mesh_problems

__all__ = ["MeshImport", "MeshImportError", "MeshObject", "candidate_parts", "read_mesh"]

MESH_SUFFIXES = (".stl", ".obj", ".dae", ".3mf")

_RIGID_TOL = 1e-6
"""Looser than ``core.transforms``: file transforms are text with 7 to 9 significant digits."""


class MeshImportError(RuntimeError):
    """The file could not be read, or its unit or contents cannot be trusted."""


@dataclass(frozen=True)
class MeshObject:
    """One placed mesh. ``vertices`` are metres in the object frame; ``transform`` is the
    4x4 rigid transform from the object frame to the file root frame, in metres."""

    name: str
    vertices: NDArray[np.float64] = field(repr=False, compare=False)
    faces: NDArray[np.int64] = field(repr=False, compare=False)
    transform: NDArray[np.float64] = field(compare=False)
    problems: tuple[str, ...] = ()
    """Empty when the mesh is a closed, outward-wound solid."""

    @property
    def watertight(self) -> bool:
        return not self.problems


@dataclass(frozen=True)
class MeshImport:
    """The result of reading one mesh file."""

    path: Path
    format: str
    file_unit: str | None
    """Unit declared in the file (as trimesh names it), or ``None`` if the format has none."""
    scale_to_si: float
    """Metres per file unit actually used."""
    objects: list[MeshObject]

    def object_named(self, name: str) -> MeshObject:
        for obj in self.objects:
            if obj.name == name:
                return obj
        raise KeyError(name)


def read_mesh(path: Path | str, *, scale_to_si: float | None = None) -> MeshImport:
    """Read ``path`` into objects in metres.

    ``scale_to_si`` overrides the declared (or missing) unit. Raises ``MeshImportError`` for
    an unreadable file, an unknown declared unit, or a file with no triangles.
    """
    import trimesh

    path = Path(path)
    suffix = path.suffix.lower()
    if suffix not in MESH_SUFFIXES:
        raise MeshImportError(f"unsupported mesh format {suffix!r} (supported: {MESH_SUFFIXES})")
    if not path.is_file():
        raise MeshImportError(f"no such file: {path}")
    if scale_to_si is not None and not (np.isfinite(scale_to_si) and scale_to_si > 0.0):
        raise MeshImportError(f"scale_to_si must be positive and finite, got {scale_to_si}")

    try:
        loaded = trimesh.load(path, force="scene")
    except Exception as exc:  # trimesh raises many types for malformed files
        raise MeshImportError(f"cannot read {path}: {exc}") from exc
    if not isinstance(loaded, trimesh.Scene):  # pragma: no cover - force="scene" guarantees it
        raise MeshImportError(f"{path}: expected a scene")
    scene: Any = loaded

    file_unit = _declared_unit(scene, path)
    if scale_to_si is None:
        if file_unit is None:
            scale = 1.0
        else:
            declared = unit_metres(file_unit)
            if declared is None:
                raise MeshImportError(f"{path}: unknown unit {file_unit!r}; pass scale_to_si")
            scale = declared
    else:
        scale = scale_to_si

    objects: list[MeshObject] = []
    names: set[str] = set()
    for node in sorted(scene.graph.nodes_geometry):
        matrix, geom_name = scene.graph[node]
        geom = scene.geometry[geom_name]
        if len(geom.faces) == 0:
            continue
        name = _unique(_object_name(node, geom_name, suffix), names)
        objects.append(_object(name, geom, np.asarray(matrix, dtype=np.float64), scale))
    if not objects:
        raise MeshImportError(f"{path}: no triangle meshes found")
    return MeshImport(path, suffix.lstrip("."), file_unit, scale, objects)


def candidate_parts(
    imported: MeshImport, asset: str, sha256: str, original_path: str | None = None
) -> list[Part]:
    """One unassigned ``Part`` per object, placed in the file root frame (metres).

    ``asset`` is the file's path relative to the project's ``assets/`` directory. The part's
    ``assembly_path`` is the object name. Mass is left unset.
    """
    return [
        Part(
            name=obj.name,
            source=PartSource(
                kind=SourceKind.MESH,
                asset=asset,
                sha256=sha256,
                scale_to_si=imported.scale_to_si,
                assembly_path=[obj.name],
                original_path=original_path,
            ),
            placement=pose_from_matrix(obj.transform),
        )
        for obj in imported.objects
    ]


# --- Internals ---------------------------------------------------------------------------


def _declared_unit(scene: Any, path: Path) -> str | None:
    suffix = path.suffix.lower()
    if suffix == ".3mf":
        units = getattr(scene, "units", None)
        return str(units) if units else "millimeter"  # 3MF's default model unit
    if suffix == ".dae":
        return _collada_unit(path)
    return None


def _collada_unit(path: Path) -> str | None:
    """The ``<unit>`` of a COLLADA file as a name ``unit_metres`` knows, or ``None``."""
    import re

    head = path.read_bytes()[:65536].decode("utf-8", errors="ignore")
    m = re.search(r"<unit\b[^>]*\bmeter\s*=\s*[\"']([^\"']+)[\"']", head)
    if m is None:
        return None  # COLLADA's default unit is 1 m
    value = float(m.group(1))
    for name in ("meter", "centimeter", "millimeter", "inch", "foot"):
        known = unit_metres(name)
        if known is not None and abs(known - value) <= 1e-9 * max(known, value):
            return name
    raise MeshImportError(f"{path}: unsupported COLLADA unit meter={value}; pass scale_to_si")


def _object_name(node: str, geom_name: str, suffix: str) -> str:
    # 3MF and DAE nodes are named by the file; STL and OBJ geometry carries the file or
    # group name. Prefer the node name, fall back to the geometry.
    return str(node if node and node != "world" else geom_name)


def _unique(name: str, taken: set[str]) -> str:
    candidate, n = name, 1
    while candidate in taken:
        n += 1
        candidate = f"{name}#{n}"
    taken.add(candidate)
    return candidate


def _object(name: str, geom: Any, matrix: NDArray[np.float64], scale: float) -> MeshObject:
    vertices = np.asarray(geom.vertices, dtype=np.float64) * scale
    faces = np.asarray(geom.faces, dtype=np.int64)
    transform = matrix.copy()
    transform[:3, 3] *= scale

    rot = transform[:3, :3]
    if np.allclose(rot @ rot.T, np.eye(3), atol=_RIGID_TOL) and np.linalg.det(rot) > 0.0:
        u, _, vt = np.linalg.svd(rot)  # remove text-rounding drift: nearest rotation
        transform[:3, :3] = u @ vt
    else:
        # Scale, shear or mirror: bake into the geometry and place the object at identity.
        vertices = vertices @ rot.T + transform[:3, 3]
        if np.linalg.det(rot) < 0.0:
            faces = faces[:, ::-1]
        transform = np.eye(4)

    vertices, faces = _weld(vertices, faces)
    problems = tuple(closed_mesh_problems(vertices, faces))
    return MeshObject(name, vertices, faces, transform, problems)


def _weld(
    vertices: NDArray[np.float64], faces: NDArray[np.int64]
) -> tuple[NDArray[np.float64], NDArray[np.int64]]:
    """Merge vertices closer than 1e-9 of the bounding diagonal (plus a floor of 1 nm).

    Scale-relative so that a part in metres and the same part in millimetres weld alike.
    Degenerate faces are kept; ``closed_mesh_problems`` reports them.
    """
    diag = float(np.linalg.norm(np.ptp(vertices, axis=0))) if len(vertices) else 0.0
    tol = max(diag * 1e-9, 1e-9)
    keys = np.round(vertices / tol).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    order = np.argsort(first)  # keep first-appearance order for stable output
    rank = np.empty_like(order)
    rank[order] = np.arange(len(order))
    return vertices[first[order]], rank[inverse.reshape(-1)][faces]
