"""Synthetic mesh fixtures (M1 task 8): boxes written as 3MF, STL, OBJ and DAE.

A box has exact volume, CoM and inertia, so importer and mass-property tests can compare
against analytic values. The 3MF writer builds the package by hand (not through trimesh) so
the file's unit, component transforms and build-item transforms are exactly what the test
says, independent of the loader under test.
"""

from __future__ import annotations

import zipfile
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

__all__ = ["box_mesh", "write_3mf_box", "write_dae_box", "write_obj_box", "write_stl_box"]

_CONTENT_TYPES = (
    '<?xml version="1.0" encoding="UTF-8"?>'
    '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
    '<Default Extension="rels" '
    'ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
    '<Default Extension="model" '
    'ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
    "</Types>"
)
_RELS = (
    '<?xml version="1.0" encoding="UTF-8"?>'
    '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
    '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
    'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>'
)


def box_mesh(
    size: tuple[float, float, float], *, centred: bool = True
) -> tuple[NDArray[np.float64], NDArray[np.int64]]:
    """Vertices and outward-wound triangles of a box; centred on the origin by default."""
    sx, sy, sz = size
    v = np.array(
        [[x, y, z] for z in (0.0, sz) for y in (0.0, sy) for x in (0.0, sx)], dtype=np.float64
    )
    if centred:
        v -= np.array(size) / 2.0
    # Index = x + 2y + 4z. Quads listed counter-clockwise seen from outside.
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    f = [t for a, b, c, d in quads for t in ((a, b, c), (a, c, d))]
    return v, np.array(f, dtype=np.int64)


def _matrix_text(m: NDArray[np.float64]) -> str:
    """3MF's 12 numbers: the 3x3 block row by row (row-vector convention), then translation."""
    rows = np.vstack([m[:3, :3].T, m[:3, 3]])
    return " ".join(f"{x:.9g}" for x in rows.ravel())


def write_3mf_box(
    path: Path,
    size: tuple[float, float, float] = (20.0, 30.0, 40.0),
    *,
    unit: str = "millimeter",
    component: NDArray[np.float64] | None = None,
    build: NDArray[np.float64] | None = None,
    extra_objects: int = 0,
) -> None:
    """Write a 3MF whose single object is a centred box of ``size`` in ``unit``.

    The object is referenced through a component (``component`` transform) from a wrapper
    object, which the build item places with ``build``; both are 4x4 matrices in the file's
    unit (column-vector convention, written in 3MF's row-vector layout). ``extra_objects``
    adds more build items of the same box, offset along x, to test multiple objects.
    """
    verts, faces = box_mesh(size)
    vx = "".join(f'<vertex x="{x:.9g}" y="{y:.9g}" z="{z:.9g}"/>' for x, y, z in verts)
    tx = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in faces)
    comp = np.eye(4) if component is None else component
    bld = np.eye(4) if build is None else build
    items = [f'<item objectid="2" transform="{_matrix_text(bld)}"/>']
    for k in range(extra_objects):
        shift = np.eye(4)
        shift[0, 3] = (k + 1) * (size[0] * 2.0)
        items.append(f'<item objectid="2" transform="{_matrix_text(shift @ bld)}"/>')
    model = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        f'<model unit="{unit}" xml:lang="en-US" '
        'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
        f'<resources><object id="1" type="model"><mesh><vertices>{vx}</vertices>'
        f"<triangles>{tx}</triangles></mesh></object>"
        '<object id="2" type="model"><components>'
        f'<component objectid="1" transform="{_matrix_text(comp)}"/>'
        "</components></object></resources>"
        f"<build>{''.join(items)}</build></model>"
    )
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", _CONTENT_TYPES)
        z.writestr("_rels/.rels", _RELS)
        z.writestr("3D/3dmodel.model", model)


def write_stl_box(path: Path, size: tuple[float, float, float] = (0.02, 0.03, 0.04)) -> None:
    """Write an ASCII STL of a centred box."""
    verts, faces = box_mesh(size)
    lines = ["solid box"]
    for a, b, c in faces:
        n = np.cross(verts[b] - verts[a], verts[c] - verts[a])
        n = n / np.linalg.norm(n)
        lines.append(f"facet normal {n[0]:.9g} {n[1]:.9g} {n[2]:.9g}\n outer loop")
        lines += [f"  vertex {x:.9g} {y:.9g} {z:.9g}" for x, y, z in verts[[a, b, c]]]
        lines.append(" endloop\nendfacet")
    lines.append("endsolid box")
    path.write_text("\n".join(lines) + "\n")


def write_obj_box(path: Path, size: tuple[float, float, float] = (0.02, 0.03, 0.04)) -> None:
    """Write an OBJ of a centred box."""
    verts, faces = box_mesh(size)
    lines = ["o box"] + [f"v {x:.9g} {y:.9g} {z:.9g}" for x, y, z in verts]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in faces]
    path.write_text("\n".join(lines) + "\n")


def write_dae_box(
    path: Path, size: tuple[float, float, float] = (20.0, 30.0, 40.0), *, unit_meter: float = 0.001
) -> None:
    """Write a COLLADA box with ``<unit meter=...>``; ``size`` is in that unit."""
    verts, faces = box_mesh(size)
    pos = " ".join(f"{x:.9g}" for x in verts.ravel())
    idx = " ".join(str(i) for i in faces.ravel())
    path.write_text(
        '<?xml version="1.0" encoding="utf-8"?>'
        '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">'
        f'<asset><unit name="unit" meter="{unit_meter}"/><up_axis>Z_UP</up_axis></asset>'
        '<library_geometries><geometry id="box-mesh" name="box"><mesh>'
        f'<source id="pos"><float_array id="pos-array" count="{verts.size}">{pos}</float_array>'
        f'<technique_common><accessor source="#pos-array" count="{len(verts)}" stride="3">'
        '<param name="X" type="float"/><param name="Y" type="float"/>'
        '<param name="Z" type="float"/></accessor></technique_common></source>'
        '<vertices id="verts"><input semantic="POSITION" source="#pos"/></vertices>'
        f'<triangles count="{len(faces)}"><input semantic="VERTEX" source="#verts" offset="0"/>'
        f"<p>{idx}</p></triangles></mesh></geometry></library_geometries>"
        '<library_visual_scenes><visual_scene id="scene"><node id="box" name="box">'
        '<instance_geometry url="#box-mesh"/></node></visual_scene></library_visual_scenes>'
        '<scene><instance_visual_scene url="#scene"/></scene></COLLADA>'
    )
