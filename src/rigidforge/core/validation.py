"""Semantic validation of a robot model (design doc sections 4 and 7).

The pydantic model only checks structure, so a bad file still loads. This module reports
what is wrong with it: every problem becomes an :class:`Issue` with a stable code, so tests,
the CLI and later the GUI can act on specific problems rather than on message text.

Checks done here:

* IDs are unique across all links, joints, parts and loop constraints.
* Names are non-empty and unique among links, among joints and among loop constraints.
* Every ID reference points at an existing link.
* The links and joints form a single tree: one root, at most one parent joint per link,
  no self-joints, no cycles, every link reachable from the root.
* Joint limits are present where required, finite and ordered; effort and velocity are
  finite and non-negative; motion axes are finite and non-zero.
* Poses are finite with unit quaternions.
* Mass inputs (density, measured mass, override mass) are finite and positive, and import
  scale factors are finite and positive.
* Inertial override tensors are finite, positive definite and satisfy the triangle
  inequality (see :mod:`rigidforge.core.inertia`); an extreme principal-moment ratio is a
  warning. Computed mass properties are checked with the same function by the physics code.
"""

import math
from collections.abc import Iterable, Iterator
from dataclasses import dataclass, field
from enum import StrEnum
from uuid import UUID

from rigidforge.core.geometry import Pose, Vec3
from rigidforge.core.inertia import InertiaProblem, check_inertia
from rigidforge.core.model import (
    Density,
    InertialOverride,
    Joint,
    JointType,
    MeasuredMass,
    RobotModel,
)

UNIT_QUAT_TOLERANCE = 1e-6
"""Allowed deviation of a quaternion's norm from 1."""


class Severity(StrEnum):
    ERROR = "error"
    """The model cannot be exported."""
    WARNING = "warning"
    """The model exports, but something is probably not what the user meant."""


class IssueCode(StrEnum):
    """Stable identifiers for validation problems. Never renumber or reuse a code."""

    EMPTY_MODEL = "empty_model"
    DUPLICATE_ID = "duplicate_id"
    EMPTY_NAME = "empty_name"
    DUPLICATE_LINK_NAME = "duplicate_link_name"
    DUPLICATE_JOINT_NAME = "duplicate_joint_name"
    DUPLICATE_CONSTRAINT_NAME = "duplicate_constraint_name"
    UNKNOWN_LINK = "unknown_link"
    SELF_JOINT = "self_joint"
    NO_ROOT = "no_root"
    MULTIPLE_ROOTS = "multiple_roots"
    MULTIPLE_PARENTS = "multiple_parents"
    CYCLE = "cycle"
    UNREACHABLE_LINK = "unreachable_link"
    MISSING_LIMITS = "missing_limits"
    LIMITS_NOT_FINITE = "limits_not_finite"
    LIMITS_REVERSED = "limits_reversed"
    NEGATIVE_EFFORT_OR_VELOCITY = "negative_effort_or_velocity"
    UNUSED_LIMITS = "unused_limits"
    BAD_AXIS = "bad_axis"
    POSE_NOT_FINITE = "pose_not_finite"
    QUAT_NOT_UNIT = "quat_not_unit"
    BAD_DYNAMICS = "bad_dynamics"
    NON_POSITIVE_MASS = "non_positive_mass"
    NON_POSITIVE_DENSITY = "non_positive_density"
    BAD_SCALE = "bad_scale"
    BAD_MIRROR_NORMAL = "bad_mirror_normal"
    UNASSIGNED_PART = "unassigned_part"
    LOOP_SAME_LINK = "loop_same_link"
    INERTIA_NOT_FINITE = "inertia_not_finite"
    INERTIA_NOT_POSITIVE_DEFINITE = "inertia_not_positive_definite"
    INERTIA_TRIANGLE_INEQUALITY = "inertia_triangle_inequality"
    INERTIA_EXTREME_RATIO = "inertia_extreme_ratio"


@dataclass(frozen=True, slots=True)
class Issue:
    """One validation problem. ``subject`` is the ID of the offending item, if any."""

    code: IssueCode
    severity: Severity
    message: str
    subject: UUID | None = None


@dataclass(frozen=True, slots=True)
class ValidationReport:
    issues: tuple[Issue, ...] = field(default_factory=tuple[Issue, ...])

    @property
    def errors(self) -> tuple[Issue, ...]:
        return tuple(i for i in self.issues if i.severity is Severity.ERROR)

    @property
    def warnings(self) -> tuple[Issue, ...]:
        return tuple(i for i in self.issues if i.severity is Severity.WARNING)

    @property
    def ok(self) -> bool:
        """True when there are no errors (warnings are allowed)."""
        return not self.errors

    def codes(self) -> set[IssueCode]:
        return {i.code for i in self.issues}


def validate(model: RobotModel) -> ValidationReport:
    """Check a model and return every problem found, in a deterministic order."""
    issues: list[Issue] = []
    issues.extend(_check_ids(model))
    issues.extend(_check_names(model))
    issues.extend(_check_references(model))
    issues.extend(_check_tree(model))
    for joint in model.joints:
        issues.extend(_check_joint(joint))
    issues.extend(_check_mass_inputs(model))
    issues.extend(_check_loop_constraints(model))
    return ValidationReport(tuple(issues))


# --- Helpers -----------------------------------------------------------------------------


def _err(code: IssueCode, message: str, subject: UUID | None = None) -> Issue:
    return Issue(code, Severity.ERROR, message, subject)


def _warn(code: IssueCode, message: str, subject: UUID | None = None) -> Issue:
    return Issue(code, Severity.WARNING, message, subject)


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(v) for v in values)


def _norm(v: Iterable[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


def _usable_direction(v: Vec3) -> bool:
    return _finite(v) and _norm(v) > 0.0


def _check_pose(pose: Pose, what: str, subject: UUID) -> Iterator[Issue]:
    if not (_finite(pose.xyz) and _finite(pose.quat)):
        yield _err(IssueCode.POSE_NOT_FINITE, f"{what} has a non-finite value", subject)
        return
    if abs(_norm(pose.quat) - 1.0) > UNIT_QUAT_TOLERANCE:
        yield _err(IssueCode.QUAT_NOT_UNIT, f"{what} quaternion is not unit length", subject)


def _duplicates(names: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    dups: list[str] = []
    for n in names:
        if n in seen and n not in dups:
            dups.append(n)
        seen.add(n)
    return dups


# --- Checks ------------------------------------------------------------------------------


def _check_ids(model: RobotModel) -> Iterator[Issue]:
    seen: set[UUID] = set()
    items = [*model.links, *model.joints, *model.parts, *model.loop_constraints]
    for item in items:
        if item.id in seen:
            yield _err(IssueCode.DUPLICATE_ID, f"ID {item.id} is used more than once", item.id)
        seen.add(item.id)


def _check_names(model: RobotModel) -> Iterator[Issue]:
    named = [*model.links, *model.joints, *model.parts, *model.loop_constraints]
    for item in named:
        if not item.name.strip():
            yield _err(
                IssueCode.EMPTY_NAME, f"{type(item).__name__} {item.id} has no name", item.id
            )
    groups = (
        (model.links, IssueCode.DUPLICATE_LINK_NAME, "link"),
        (model.joints, IssueCode.DUPLICATE_JOINT_NAME, "joint"),
        (model.loop_constraints, IssueCode.DUPLICATE_CONSTRAINT_NAME, "loop constraint"),
    )
    for items, code, kind in groups:
        for name in _duplicates(i.name for i in items):
            first = next(i for i in items if i.name == name)
            yield _err(code, f"more than one {kind} is named {name!r}", first.id)


def _check_references(model: RobotModel) -> Iterator[Issue]:
    link_ids = {link.id for link in model.links}
    for j in model.joints:
        for role, ref in (("parent", j.parent), ("child", j.child)):
            if ref not in link_ids:
                yield _err(
                    IssueCode.UNKNOWN_LINK, f"joint {j.name!r} {role} {ref} is not a link", j.id
                )
    for p in model.parts:
        if p.link is None:
            yield _warn(IssueCode.UNASSIGNED_PART, f"part {p.name!r} is not in any link", p.id)
        elif p.link not in link_ids:
            yield _err(IssueCode.UNKNOWN_LINK, f"part {p.name!r} link {p.link} is not a link", p.id)
    for c in model.loop_constraints:
        for ref in (c.link_a, c.link_b):
            if ref not in link_ids:
                yield _err(
                    IssueCode.UNKNOWN_LINK,
                    f"loop constraint {c.name!r} link {ref} is not a link",
                    c.id,
                )


def _check_tree(model: RobotModel) -> Iterator[Issue]:
    if not model.links:
        yield _err(IssueCode.EMPTY_MODEL, "the model has no links")
        return
    link_ids = {link.id for link in model.links}
    names = {link.id: link.name for link in model.links}
    # Only joints between known links take part in the tree; others are reported above.
    joints = [j for j in model.joints if j.parent in link_ids and j.child in link_ids]

    parent_joints: dict[UUID, list[Joint]] = {}
    children: dict[UUID, list[UUID]] = {}
    for j in joints:
        if j.parent == j.child:
            yield _err(IssueCode.SELF_JOINT, f"joint {j.name!r} connects a link to itself", j.id)
            continue
        parent_joints.setdefault(j.child, []).append(j)
        children.setdefault(j.parent, []).append(j.child)

    for link in model.links:
        pj = parent_joints.get(link.id, [])
        if len(pj) > 1:
            joint_names = ", ".join(repr(j.name) for j in pj)
            yield _err(
                IssueCode.MULTIPLE_PARENTS,
                f"link {link.name!r} is the child of several joints ({joint_names}); "
                "use a loop constraint to close a loop",
                link.id,
            )

    roots = [link for link in model.links if link.id not in parent_joints]
    if not roots:
        yield _err(IssueCode.NO_ROOT, "every link has a parent joint, so there is no root")
    elif len(roots) > 1:
        root_names = ", ".join(repr(r.name) for r in roots)
        yield _err(
            IssueCode.MULTIPLE_ROOTS, f"the model has several roots ({root_names})", roots[1].id
        )

    # Reachability from the roots; anything left over sits on a cycle or hangs off one.
    reached: set[UUID] = set()
    stack = [r.id for r in roots]
    while stack:
        current = stack.pop()
        if current in reached:
            continue
        reached.add(current)
        stack.extend(children.get(current, []))

    on_cycle = _cycle_members(children, link_ids - reached)
    for link in model.links:
        if link.id in on_cycle:
            yield _err(IssueCode.CYCLE, f"link {names[link.id]!r} is on a joint cycle", link.id)
        elif link.id not in reached:
            yield _err(
                IssueCode.UNREACHABLE_LINK,
                f"link {names[link.id]!r} cannot be reached from the root",
                link.id,
            )


def _cycle_members(children: dict[UUID, list[UUID]], candidates: set[UUID]) -> set[UUID]:
    """Links among ``candidates`` that lie on a directed cycle (Tarjan's SCC, iterative)."""
    index: dict[UUID, int] = {}
    low: dict[UUID, int] = {}
    on_stack: set[UUID] = set()
    stack: list[UUID] = []
    members: set[UUID] = set()
    counter = 0

    for start in sorted(candidates):
        if start in index:
            continue
        work: list[tuple[UUID, Iterator[UUID]]] = [(start, iter(children.get(start, [])))]
        index[start] = low[start] = counter
        counter += 1
        stack.append(start)
        on_stack.add(start)
        while work:
            node, it = work[-1]
            advanced = False
            for nxt in it:
                if nxt not in candidates:
                    continue
                if nxt not in index:
                    index[nxt] = low[nxt] = counter
                    counter += 1
                    stack.append(nxt)
                    on_stack.add(nxt)
                    work.append((nxt, iter(children.get(nxt, []))))
                    advanced = True
                    break
                if nxt in on_stack:
                    low[node] = min(low[node], index[nxt])
            if advanced:
                continue
            work.pop()
            if work:
                parent = work[-1][0]
                low[parent] = min(low[parent], low[node])
            if low[node] == index[node]:
                component: list[UUID] = []
                while True:
                    w = stack.pop()
                    on_stack.discard(w)
                    component.append(w)
                    if w == node:
                        break
                self_loop = node in children.get(node, [])
                if len(component) > 1 or self_loop:
                    members.update(component)
    return members


def _check_joint(j: Joint) -> Iterator[Issue]:
    yield from _check_pose(j.origin, f"joint {j.name!r} origin", j.id)

    moves = j.type is not JointType.FIXED
    if moves and not _usable_direction(j.axis):
        yield _err(IssueCode.BAD_AXIS, f"joint {j.name!r} axis is zero or not finite", j.id)

    needs_limits = j.type in (JointType.REVOLUTE, JointType.PRISMATIC)
    lim = j.limits
    if lim is None:
        if needs_limits:
            yield _err(IssueCode.MISSING_LIMITS, f"{j.type} joint {j.name!r} has no limits", j.id)
    else:
        if not needs_limits:
            yield _warn(
                IssueCode.UNUSED_LIMITS,
                f"{j.type} joint {j.name!r} has position limits that will be ignored",
                j.id,
            )
        rates = [v for v in (lim.effort, lim.velocity) if v is not None]
        if not _finite((lim.lower, lim.upper, *rates)):
            yield _err(IssueCode.LIMITS_NOT_FINITE, f"joint {j.name!r} limits are not finite", j.id)
        else:
            if lim.lower > lim.upper:
                yield _err(
                    IssueCode.LIMITS_REVERSED,
                    f"joint {j.name!r} lower limit {lim.lower} is above upper limit {lim.upper}",
                    j.id,
                )
            if any(v < 0.0 for v in rates):
                yield _err(
                    IssueCode.NEGATIVE_EFFORT_OR_VELOCITY,
                    f"joint {j.name!r} effort or velocity limit is negative",
                    j.id,
                )

    dyn = j.dynamics
    if dyn is not None:
        values = (dyn.damping, dyn.friction)
        if not _finite(values) or any(v < 0.0 for v in values):
            yield _err(
                IssueCode.BAD_DYNAMICS,
                f"joint {j.name!r} damping and friction must be finite and non-negative",
                j.id,
            )


def _check_override(ov: InertialOverride, what: str, subject: UUID) -> Iterator[Issue]:
    if not (math.isfinite(ov.mass) and ov.mass > 0.0):
        yield _err(IssueCode.NON_POSITIVE_MASS, f"{what} override mass must be > 0", subject)
    yield from _check_pose(Pose(xyz=ov.com, quat=ov.orientation), f"{what} override frame", subject)
    for problem in check_inertia(1.0, ov.inertia):  # mass is checked above
        match problem:
            case InertiaProblem.NOT_FINITE:
                yield _err(
                    IssueCode.INERTIA_NOT_FINITE, f"{what} override inertia is not finite", subject
                )
            case InertiaProblem.NOT_POSITIVE_DEFINITE:
                yield _err(
                    IssueCode.INERTIA_NOT_POSITIVE_DEFINITE,
                    f"{what} override inertia is not positive definite",
                    subject,
                )
            case InertiaProblem.TRIANGLE_INEQUALITY:
                yield _err(
                    IssueCode.INERTIA_TRIANGLE_INEQUALITY,
                    f"{what} override inertia violates the triangle inequality",
                    subject,
                )
            case InertiaProblem.EXTREME_RATIO:
                yield _warn(
                    IssueCode.INERTIA_EXTREME_RATIO,
                    f"{what} override principal moments differ by a very large ratio",
                    subject,
                )
            case InertiaProblem.NON_POSITIVE_MASS | InertiaProblem.NOT_SYMMETRIC:
                pass  # mass checked above; InertiaTensor is symmetric by construction


def _check_mass_inputs(model: RobotModel) -> Iterator[Issue]:
    for link in model.links:
        if link.inertial_override is not None:
            yield from _check_override(link.inertial_override, f"link {link.name!r}", link.id)
    for p in model.parts:
        what = f"part {p.name!r}"
        yield from _check_pose(p.placement, f"{what} placement", p.id)
        scale = p.source.scale_to_si
        if not (math.isfinite(scale) and scale > 0.0):
            yield _err(IssueCode.BAD_SCALE, f"{what} scale_to_si must be > 0", p.id)
        if p.mirror is not None and not (
            _usable_direction(p.mirror.normal) and _finite(p.mirror.point)
        ):
            yield _err(IssueCode.BAD_MIRROR_NORMAL, f"{what} mirror plane is degenerate", p.id)
        match p.mass:
            case Density(density=d) if not (math.isfinite(d) and d > 0.0):
                yield _err(IssueCode.NON_POSITIVE_DENSITY, f"{what} density must be > 0", p.id)
            case MeasuredMass(mass=m) if not (math.isfinite(m) and m > 0.0):
                yield _err(IssueCode.NON_POSITIVE_MASS, f"{what} measured mass must be > 0", p.id)
            case _:
                pass
        if p.inertial_override is not None:
            yield from _check_override(p.inertial_override, what, p.id)


def _check_loop_constraints(model: RobotModel) -> Iterator[Issue]:
    for c in model.loop_constraints:
        if c.link_a == c.link_b:
            yield _err(
                IssueCode.LOOP_SAME_LINK, f"loop constraint {c.name!r} uses one link twice", c.id
            )
        yield from _check_pose(c.frame_a, f"loop constraint {c.name!r} frame_a", c.id)
        yield from _check_pose(c.frame_b, f"loop constraint {c.name!r} frame_b", c.id)
