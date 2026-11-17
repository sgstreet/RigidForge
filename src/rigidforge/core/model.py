"""The internal robot model (design doc sections 5 and 7).

A robot is a tree of links joined by joints, plus optional loop-closure constraints. Parts
are pieces of geometry placed in a link; a link's mass properties come from its parts unless
the link carries a manual override.

Rules that hold throughout:

* Everything is in SI: metres, kilograms, seconds, radians.
* Links, joints, parts and loop constraints are identified by stable UUIDs and refer to each
  other by ID. Names are user-editable labels, not keys.
* Joint frames are numeric and authoritative. A snap hint records where a frame came from
  and is never used to recompute it.
* Only inputs are stored. Computed mass properties, tessellations and B-Rep shapes are
  derived data and live in the cache.
* The model checks structure (types and shapes). Semantic checks such as a single root,
  positive mass or sane limits are done by ``rigidforge.core.validation``, so that a bad
  file can still be loaded and its problems reported.
"""

from enum import StrEnum
from typing import Annotated, Literal
from uuid import UUID, uuid4

from pydantic import BaseModel, ConfigDict, Field

from rigidforge.core.extensions import Extensions
from rigidforge.core.geometry import IDENTITY_QUAT, InertiaTensor, Pose, Quat, Vec3

SCHEMA_VERSION = 1
"""Current project schema version. Bump it and add a migration when the layout changes."""


class _Model(BaseModel):
    """Base for core models: unknown fields are errors, assignments are validated."""

    model_config = ConfigDict(extra="forbid", validate_assignment=True)


def _new_id() -> UUID:
    return uuid4()


# --- Parts -------------------------------------------------------------------------------


class SourceKind(StrEnum):
    STEP = "step"
    MESH = "mesh"


class PartSource(_Model):
    """Where a part's geometry comes from: a file copied into the project's ``assets/``.

    ``scale_to_si`` is applied to the geometry at the import boundary (0.001 for a file in
    millimetres). ``assembly_path`` locates the part inside a STEP assembly by product name
    and instance index; it is empty for a single-solid file or a mesh.
    """

    kind: SourceKind
    asset: str
    """Path relative to the project's ``assets/`` directory."""
    sha256: str
    scale_to_si: float = 1.0
    assembly_path: list[str] = Field(default_factory=list[str])
    original_path: str | None = None
    """Absolute path of the file when it was imported. A hint for refresh only."""


class MirrorPlane(_Model):
    """A mirror about a plane in the part file's own coordinates, baked into the geometry."""

    point: Vec3 = (0.0, 0.0, 0.0)
    normal: Vec3


class Density(_Model):
    """Mass from geometry at a uniform density in kg/m³."""

    kind: Literal["density"] = "density"
    density: float


class MeasuredMass(_Model):
    """Mass from a scale. Sets an effective density so the computed mass matches.

    Inertia scales with the measured mass at fixed shape; the CoM comes from geometry.
    """

    kind: Literal["measured_mass"] = "measured_mass"
    mass: float


class Massless(_Model):
    """No mass of its own: geometry for display or collision only.

    Used for the other parts of an assembly whose mass properties come from one override.
    """

    kind: Literal["massless"] = "massless"


type MassSource = Annotated[Density | MeasuredMass | Massless, Field(discriminator="kind")]


class InertialOverride(_Model):
    """Mass properties given directly (for example from a datasheet), replacing geometry.

    ``com`` is in the owner's frame (the part file frame for a part, the link frame for a
    link). The tensor is about the CoM, expressed in a frame at ``com`` rotated by
    ``orientation`` (scalar-first quaternion) relative to the owner's frame.
    """

    mass: float
    com: Vec3
    inertia: InertiaTensor
    orientation: Quat = IDENTITY_QUAT
    source: str | None = None
    """Free text recording where the numbers came from."""


class Part(_Model):
    """A piece of geometry placed in a link.

    ``placement`` puts the part file's frame in its link's frame. A part with no link is an
    imported candidate not yet assigned. Several parts may share one source (instancing).
    """

    id: UUID = Field(default_factory=_new_id)
    name: str
    source: PartSource
    link: UUID | None = None
    placement: Pose = Field(default_factory=Pose)
    mirror: MirrorPlane | None = None
    mass: MassSource | None = None
    inertial_override: InertialOverride | None = None
    material: str | None = None


# --- Links, joints and loop constraints ---------------------------------------------------


class Link(_Model):
    """A rigid body. Its mass properties come from its parts unless overridden."""

    id: UUID = Field(default_factory=_new_id)
    name: str
    inertial_override: InertialOverride | None = None
    extensions: Extensions = Field(default_factory=Extensions)


class JointType(StrEnum):
    FIXED = "fixed"
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"


class JointLimits(_Model):
    """Position limits (rad or m), effort (N·m or N) and velocity (rad/s or m/s)."""

    lower: float
    upper: float
    effort: float | None = None
    velocity: float | None = None


class JointDynamics(_Model):
    damping: float = 0.0
    friction: float = 0.0


class SnapHint(_Model):
    """Where a joint frame was snapped from: a hint only, never authoritative.

    B-Rep face indices are not stable across CAD re-exports, so a hint may fail to
    re-resolve; the numeric frame is kept regardless.
    """

    part: UUID
    face_index: int
    feature: str
    """A short signature of the snapped feature, for example ``"cylinder r=0.0035"``."""


class Joint(_Model):
    """Connects a parent link to a child link.

    ``origin`` is the joint frame in the parent link's frame; the child link's frame
    coincides with the joint frame at zero position. ``axis`` is a direction in the joint
    frame (ignored for fixed joints).
    """

    id: UUID = Field(default_factory=_new_id)
    name: str
    type: JointType
    parent: UUID
    child: UUID
    origin: Pose = Field(default_factory=Pose)
    axis: Vec3 = (0.0, 0.0, 1.0)
    limits: JointLimits | None = None
    dynamics: JointDynamics | None = None
    snap_hint: SnapHint | None = None
    extensions: Extensions = Field(default_factory=Extensions)


class LoopConstraintType(StrEnum):
    CONNECT = "connect"
    """Ball-joint style: the two frames' origins coincide."""
    WELD = "weld"
    """The two frames coincide fully."""


class LoopConstraint(_Model):
    """Closes a kinematic loop between two links. URDF cannot express this; MJCF and SDF can.

    Each frame is given in its own link's frame.
    """

    id: UUID = Field(default_factory=_new_id)
    name: str
    type: LoopConstraintType
    link_a: UUID
    link_b: UUID
    frame_a: Pose = Field(default_factory=Pose)
    frame_b: Pose = Field(default_factory=Pose)
    extensions: Extensions = Field(default_factory=Extensions)


# --- Robot -------------------------------------------------------------------------------


class RobotModel(_Model):
    """The whole robot. Lists keep their order, so saved files diff cleanly."""

    schema_version: int = SCHEMA_VERSION
    name: str
    links: list[Link] = Field(default_factory=list[Link])
    joints: list[Joint] = Field(default_factory=list[Joint])
    parts: list[Part] = Field(default_factory=list[Part])
    loop_constraints: list[LoopConstraint] = Field(default_factory=list[LoopConstraint])
    extensions: Extensions = Field(default_factory=Extensions)

    def link(self, link_id: UUID) -> Link:
        """Return the link with this ID. Raises ``KeyError`` if there is none."""
        for link in self.links:
            if link.id == link_id:
                return link
        raise KeyError(link_id)

    def joint(self, joint_id: UUID) -> Joint:
        """Return the joint with this ID. Raises ``KeyError`` if there is none."""
        for joint in self.joints:
            if joint.id == joint_id:
                return joint
        raise KeyError(joint_id)

    def part(self, part_id: UUID) -> Part:
        """Return the part with this ID. Raises ``KeyError`` if there is none."""
        for part in self.parts:
            if part.id == part_id:
                return part
        raise KeyError(part_id)

    def parts_of(self, link_id: UUID) -> list[Part]:
        """Parts assigned to a link, in model order."""
        return [p for p in self.parts if p.link == link_id]

    def child_joints(self, link_id: UUID) -> list[Joint]:
        """Joints whose parent is this link, in model order."""
        return [j for j in self.joints if j.parent == link_id]
