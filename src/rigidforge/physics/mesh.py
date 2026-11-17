"""Mass properties of a closed triangle mesh (M1 task 8).

``mesh_unit_mass_properties`` integrates over a closed, consistently wound triangle mesh in
metres: each triangle forms a signed tetrahedron with a reference point, and the volume,
first moments and second moments are summed exactly (the integrals of polynomials over a
tetrahedron are closed-form). The result is the volume, CoM and inertia tensor about the
CoM at unit density, like ``rigidforge.physics.brep``. The only error is the mesh's own
tessellation error (design doc section 6, spike 4: about 1e-3 on curved parts).

The reference point is the vertex mean, which keeps the sums well conditioned for a part
far from its frame origin.

``closed_mesh_problems`` reports what makes a mesh unfit for this: degenerate faces, open
edges, edges shared by more than two faces, inconsistent winding, and inside-out meshes.
"""

import numpy as np
import numpy.typing as npt

from rigidforge.physics.mass import InertiaMethod, MassPropertiesError, UnitMassProperties

__all__ = ["MIN_MESH_VOLUME", "closed_mesh_problems", "mesh_unit_mass_properties"]

MIN_MESH_VOLUME = 1e-15
"""Smallest accepted volume in m³ (a 10 µm cube). Anything smaller is treated as empty."""

type Vertices = npt.NDArray[np.float64]
type Faces = npt.NDArray[np.int64]


def _signed_volume(vertices: Vertices, faces: Faces) -> float:
    ref = vertices.mean(axis=0)
    a = vertices[faces[:, 0]] - ref
    b = vertices[faces[:, 1]] - ref
    c = vertices[faces[:, 2]] - ref
    return float(np.sum(np.einsum("ij,ij->i", a, np.cross(b, c))) / 6.0)


def closed_mesh_problems(vertices: Vertices, faces: Faces) -> list[str]:
    """Reasons ``vertices`` and ``faces`` are not a closed, consistently wound solid.

    The list is empty for a watertight, outward-wound mesh. Vertices are expected to be
    welded already, so that adjacent faces share vertex indices.
    """
    problems: list[str] = []
    if vertices.ndim != 2 or vertices.shape[1] != 3 or faces.ndim != 2 or faces.shape[1] != 3:
        return ["vertices must be Nx3 and faces Mx3"]
    if len(faces) < 4:
        return ["fewer than 4 faces"]
    if not np.all(np.isfinite(vertices)):
        return ["non-finite vertex coordinates"]
    if faces.min() < 0 or faces.max() >= len(vertices):
        return ["face index out of range"]

    degenerate = (
        (faces[:, 0] == faces[:, 1]) | (faces[:, 1] == faces[:, 2]) | (faces[:, 0] == faces[:, 2])
    )
    if np.any(degenerate):
        problems.append(f"{int(degenerate.sum())} degenerate faces")
        faces = faces[~degenerate]

    directed = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    n = len(vertices)
    keys = directed[:, 0].astype(np.int64) * n + directed[:, 1]
    reverse = directed[:, 1].astype(np.int64) * n + directed[:, 0]
    uniq, counts = np.unique(keys, return_counts=True)
    if np.any(counts > 1):
        # Same directed edge used twice: either inconsistent winding or a non-manifold edge.
        problems.append("inconsistent winding or non-manifold edges")
    missing = ~np.isin(uniq, reverse)
    if np.any(missing):
        problems.append(f"{int(missing.sum())} open edges")

    if not problems:
        volume = _signed_volume(vertices, faces)
        if volume < 0.0:
            problems.append("faces wound inward (negative volume)")
        elif volume == 0.0:
            problems.append("zero volume")
    return problems


def mesh_unit_mass_properties(
    vertices: Vertices, faces: Faces, name: str = "mesh"
) -> UnitMassProperties:
    """Unit-density volume, CoM and CoM inertia of a closed mesh in metres.

    Raises :class:`MassPropertiesError` if the mesh is not closed and outward wound, or its
    volume is below :data:`MIN_MESH_VOLUME`.
    """
    v = np.asarray(vertices, dtype=np.float64)
    f = np.asarray(faces, dtype=np.int64)
    problems = closed_mesh_problems(v, f)
    if problems:
        raise MassPropertiesError(f"{name}: mesh is not a closed solid: {'; '.join(problems)}")

    ref = v.mean(axis=0)
    a = v[f[:, 0]] - ref
    b = v[f[:, 1]] - ref
    c = v[f[:, 2]] - ref
    tet = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
    volume = float(tet.sum())
    if volume < MIN_MESH_VOLUME:
        raise MassPropertiesError(f"{name}: volume {volume} m³ is ~0 (is the mesh in metres?)")

    centroid_sum = a + b + c
    com_rel = (tet[:, None] * centroid_sum / 4.0).sum(axis=0) / volume

    # Second moment about the reference point: sum over tetrahedra (with the reference as
    # the fourth vertex) of V/20 * (a aᵀ + b bᵀ + c cᵀ + s sᵀ), s = a + b + c.
    outer = (
        np.einsum("ni,nj->nij", a, a)
        + np.einsum("ni,nj->nij", b, b)
        + np.einsum("ni,nj->nij", c, c)
        + np.einsum("ni,nj->nij", centroid_sum, centroid_sum)
    )
    second = (tet[:, None, None] / 20.0 * outer).sum(axis=0)
    second_com = second - volume * np.outer(com_rel, com_rel)
    inertia = np.trace(second_com) * np.eye(3) - second_com
    com = ref + com_rel
    if not (np.all(np.isfinite(com)) and np.all(np.isfinite(inertia))):
        raise MassPropertiesError(f"{name}: CoM or inertia is not finite")
    return UnitMassProperties(volume, com, 0.5 * (inertia + inertia.T), InertiaMethod.MESH)
