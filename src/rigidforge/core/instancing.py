"""Part instancing (M1 task 15): rigid copies and mirrored copies of a part.

A copy shares its source file and assembly path with the original, so no geometry is
duplicated in the project; only the placement (and a mirror plane, if any) differs.

* ``rigid_copy`` places the same geometry elsewhere: rotation plus translation only.
* ``mirrored_copy`` mirrors about a plane in the part file's own coordinates. The mirror is
  baked into the B-Rep or mesh before mass properties are computed (see
  ``rigidforge.physics.mirror``), never a negative scale. The placement is applied after.
"""

from collections.abc import Sequence
from uuid import UUID, uuid4

import numpy as np

from rigidforge.core.geometry import Pose, Vec3
from rigidforge.core.inertia import InertiaProblem, check_inertia
from rigidforge.core.model import (
    InertialOverride,
    Massless,
    MassSource,
    MirrorPlane,
    Part,
)
from rigidforge.core.transforms import (
    matrix_from_pose,
    pose_from_matrix,
    quat_from_rotation,
    rotation_from_quat,
)

__all__ = ["instantiate_assembly", "mirrored_copy", "rigid_copy"]


def rigid_copy(
    part: Part, *, name: str | None = None, placement: Pose | None = None, link: UUID | None = None
) -> Part:
    """A copy of ``part`` with a new ID, the same source and mirror, and a new placement.

    ``placement`` defaults to the original's. ``link`` is unchanged unless given (a UUID, or
    ``None`` to keep the original's).
    """
    update: dict[str, object] = {"id": uuid4()}
    if name is not None:
        update["name"] = name
    if placement is not None:
        update["placement"] = placement
    if link is not None:
        update["link"] = link
    return Part.model_validate(part.model_copy(deep=True, update=update).model_dump())


def mirrored_copy(
    part: Part,
    plane: MirrorPlane,
    *,
    name: str | None = None,
    placement: Pose | None = None,
    link: UUID | None = None,
) -> Part:
    """A copy of ``part`` mirrored about ``plane`` (part file coordinates, metres).

    Raises ``ValueError`` if the part is already mirrored: compose the mirrors by choosing
    the plane in the original file frame instead.
    """
    if part.mirror is not None:
        raise ValueError(f"part {part.name!r} is already mirrored")
    copy = rigid_copy(part, name=name, placement=placement, link=link)
    return copy.model_copy(update={"mirror": plane})


def instantiate_assembly(
    candidates: Sequence[Part],
    placement: Pose,
    *,
    link: UUID,
    prefix: str,
    mass: MassSource | None = None,
    override: InertialOverride | None = None,
    override_on: str | None = None,
    com_bounds: tuple[Vec3, Vec3] | None = None,
) -> list[Part]:
    """Place one imported assembly in a link as rigid copies of its leaf parts.

    ``candidates`` are the parts of a single import (``candidate_parts``), each placed in the
    assembly's root frame. Every copy keeps its source (so the file is stored once and the
    geometry is shared), gets a new ID and the name ``prefix + original name``, belongs to
    ``link``, and has the placement ``placement`` composed with its own (root frame to link
    frame). ``mass`` is applied to parts that have no mass source or override.

    ``override`` is a manufacturer mass, CoM and inertia tensor for the whole assembly, given
    in the assembly's root frame (tensor about the CoM, rotated by ``override.orientation``).
    It is stored on the first part named ``override_on`` (default: the first candidate), converted
    to that part's file frame, and every other part without its own mass source or override
    becomes :class:`Massless`. The override is checked first: mass > 0, symmetric positive
    definite tensor, triangle inequality, and, when ``com_bounds`` (min and max corners of the
    assembly in the root frame, metres) is given, a CoM inside those bounds. A wrong frame or
    unit usually shows up as a CoM far outside the geometry. Raises ``ValueError``.
    """
    target: str | None = None
    if override is not None:
        _check_assembly_override(override, com_bounds)
        if mass is not None:
            raise ValueError("give either mass or override, not both")
        names = [p.name for p in candidates]
        target = override_on if override_on is not None else (names[0] if names else None)
        if target not in names:
            raise ValueError(f"override_on {target!r} is not a part of the assembly")
    root = matrix_from_pose(placement)
    out: list[Part] = []
    carrier = next((p for p in candidates if p.name == target), None)
    for part in candidates:
        if part.mirror is not None:
            raise ValueError(f"part {part.name!r} is mirrored; instantiate unmirrored assemblies")
        composed = pose_from_matrix(root @ matrix_from_pose(part.placement))
        copy = rigid_copy(part, name=f"{prefix}{part.name}", placement=composed, link=link)
        if mass is not None and copy.mass is None and copy.inertial_override is None:
            copy = copy.model_copy(update={"mass": mass})
        if override is not None and copy.mass is None and copy.inertial_override is None:
            if part is carrier:
                copy = copy.model_copy(
                    update={"inertial_override": _into_part_frame(override, part.placement)}
                )
            else:
                copy = copy.model_copy(update={"mass": Massless()})
        out.append(copy)
    return out


def _check_assembly_override(ov: InertialOverride, bounds: tuple[Vec3, Vec3] | None) -> None:
    problems = check_inertia(ov.mass, ov.inertia)
    errors = [p for p in problems if p is not InertiaProblem.EXTREME_RATIO]
    if errors:
        raise ValueError(
            f"assembly inertial override is invalid: {', '.join(e.value for e in errors)}"
        )
    if bounds is not None:
        lo, hi = np.asarray(bounds[0]), np.asarray(bounds[1])
        com = np.asarray(ov.com)
        if not (np.all(com >= lo) and np.all(com <= hi)):
            raise ValueError(
                f"assembly override CoM {tuple(ov.com)} is outside the assembly bounds "
                f"{tuple(bounds[0])} to {tuple(bounds[1])}: check its frame and units"
            )


def _into_part_frame(ov: InertialOverride, placement: Pose) -> InertialOverride:
    """Re-express an override given in the assembly root frame in a part's file frame."""
    inv = np.linalg.inv(matrix_from_pose(placement))
    com = inv[:3, :3] @ np.asarray(ov.com) + inv[:3, 3]
    rot = inv[:3, :3] @ rotation_from_quat(ov.orientation)
    return ov.model_copy(
        update={
            "com": (float(com[0]), float(com[1]), float(com[2])),
            "orientation": quat_from_rotation(rot),
        }
    )
