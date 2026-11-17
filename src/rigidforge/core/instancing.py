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

from rigidforge.core.geometry import Pose
from rigidforge.core.model import MassSource, MirrorPlane, Part
from rigidforge.core.transforms import matrix_from_pose, pose_from_matrix

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
) -> list[Part]:
    """Place one imported assembly in a link as rigid copies of its leaf parts.

    ``candidates`` are the parts of a single import (``candidate_parts``), each placed in the
    assembly's root frame. Every copy keeps its source (so the file is stored once and the
    geometry is shared), gets a new ID and the name ``prefix + original name``, belongs to
    ``link``, and has the placement ``placement`` composed with its own (root frame to link
    frame). ``mass`` is applied to parts that have no mass source or override.
    """
    root = matrix_from_pose(placement)
    out: list[Part] = []
    for part in candidates:
        if part.mirror is not None:
            raise ValueError(f"part {part.name!r} is mirrored; instantiate unmirrored assemblies")
        composed = pose_from_matrix(root @ matrix_from_pose(part.placement))
        copy = rigid_copy(part, name=f"{prefix}{part.name}", placement=composed, link=link)
        if mass is not None and copy.mass is None and copy.inertial_override is None:
            copy = copy.model_copy(update={"mass": mass})
        out.append(copy)
    return out
