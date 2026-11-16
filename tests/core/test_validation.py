"""Tests for semantic model validation (task 3): table-driven bad models with specific codes."""

import math
import random
from collections.abc import Callable

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from rigidforge.core import (
    Density,
    InertialOverride,
    InertiaTensor,
    IssueCode,
    Joint,
    JointDynamics,
    JointLimits,
    JointType,
    Link,
    LoopConstraint,
    LoopConstraintType,
    MeasuredMass,
    MirrorPlane,
    Part,
    PartSource,
    Pose,
    RobotModel,
    Severity,
    SourceKind,
    validate,
)

NAN = math.nan
INF = math.inf


def leg() -> RobotModel:
    """A valid 3-link chain: base -> coxa (revolute) -> femur (revolute), one part per link."""
    base, coxa, femur = Link(name="base"), Link(name="coxa"), Link(name="femur")
    src = PartSource(kind=SourceKind.STEP, asset="frame.step", sha256="00", scale_to_si=1e-3)
    parts = [
        Part(name=f"{link.name}-frame", source=src, link=link.id, mass=Density(density=1270.0))
        for link in (base, coxa, femur)
    ]
    limits = JointLimits(lower=-1.0, upper=1.0, effort=0.6, velocity=8.0)
    joints = [
        Joint(name="yaw", type=JointType.REVOLUTE, parent=base.id, child=coxa.id, limits=limits),
        Joint(
            name="pitch",
            type=JointType.REVOLUTE,
            parent=coxa.id,
            child=femur.id,
            origin=Pose(xyz=(0.0435, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            limits=limits,
        ),
    ]
    return RobotModel(name="leg", links=[base, coxa, femur], joints=joints, parts=parts)


def test_valid_model_has_no_issues() -> None:
    report = validate(leg())
    assert report.ok
    assert report.issues == ()


def test_single_link_is_valid() -> None:
    assert validate(RobotModel(name="r", links=[Link(name="base")])).ok


def test_validation_is_deterministic() -> None:
    m = leg()
    m.links.append(Link(name="coxa"))
    assert validate(m) == validate(m)


# --- Table of bad models: (mutation, expected code, expected severity) -------------------

type Mutation = Callable[[RobotModel], None]


def _joint(m: RobotModel, i: int = 0) -> Joint:
    return m.joints[i]


def _two_roots(m: RobotModel) -> None:
    m.links.append(Link(name="loose"))


def _cycle(m: RobotModel) -> None:
    # femur -> base closes base -> coxa -> femur -> base; no root remains.
    m.joints.append(
        Joint(name="back", type=JointType.FIXED, parent=m.links[2].id, child=m.links[0].id)
    )


def _cycle_with_root(m: RobotModel) -> None:
    a, b = Link(name="a"), Link(name="b")
    m.links += [a, b]
    m.joints += [
        Joint(name="ab", type=JointType.FIXED, parent=a.id, child=b.id),
        Joint(name="ba", type=JointType.FIXED, parent=b.id, child=a.id),
    ]


def _two_parents(m: RobotModel) -> None:
    m.joints.append(
        Joint(name="extra", type=JointType.FIXED, parent=m.links[0].id, child=m.links[2].id)
    )


def _dup_link_name(m: RobotModel) -> None:
    m.links[2].name = "coxa"


def _dup_joint_name(m: RobotModel) -> None:
    m.joints[1].name = "yaw"


def _empty_name(m: RobotModel) -> None:
    m.links[1].name = "  "


def _dup_id(m: RobotModel) -> None:
    m.parts[1].id = m.links[0].id


def _unknown_child(m: RobotModel) -> None:
    m.joints[1].child = Link(name="ghost").id


def _self_joint(m: RobotModel) -> None:
    m.joints[1].child = m.joints[1].parent


def _part_unknown_link(m: RobotModel) -> None:
    m.parts[0].link = Link(name="ghost").id


def _no_limits(m: RobotModel) -> None:
    _joint(m).limits = None


def _nan_limit(m: RobotModel) -> None:
    _joint(m).limits = JointLimits(lower=NAN, upper=1.0)


def _inf_velocity(m: RobotModel) -> None:
    _joint(m).limits = JointLimits(lower=-1.0, upper=1.0, velocity=INF)


def _reversed_limits(m: RobotModel) -> None:
    _joint(m).limits = JointLimits(lower=1.0, upper=-1.0)


def _negative_effort(m: RobotModel) -> None:
    _joint(m).limits = JointLimits(lower=-1.0, upper=1.0, effort=-0.1)


def _zero_axis(m: RobotModel) -> None:
    _joint(m).axis = (0.0, 0.0, 0.0)


def _nan_origin(m: RobotModel) -> None:
    _joint(m).origin = Pose(xyz=(NAN, 0.0, 0.0))


def _non_unit_quat(m: RobotModel) -> None:
    _joint(m).origin = Pose(quat=(1.0, 1.0, 0.0, 0.0))


def _negative_damping(m: RobotModel) -> None:
    _joint(m).dynamics = JointDynamics(damping=-1.0)


def _zero_density(m: RobotModel) -> None:
    m.parts[0].mass = Density(density=0.0)


def _zero_measured_mass(m: RobotModel) -> None:
    m.parts[0].mass = MeasuredMass(mass=0.0)


def _zero_override_mass(m: RobotModel) -> None:
    m.links[1].inertial_override = InertialOverride(
        mass=0.0, com=(0.0, 0.0, 0.0), inertia=InertiaTensor(ixx=1e-6, iyy=1e-6, izz=1e-6)
    )


def _nan_part_override_com(m: RobotModel) -> None:
    m.parts[1].inertial_override = InertialOverride(
        mass=0.01, com=(NAN, 0.0, 0.0), inertia=InertiaTensor(ixx=1e-6, iyy=1e-6, izz=1e-6)
    )


def _zero_scale(m: RobotModel) -> None:
    m.parts[0].source = m.parts[0].source.model_copy(update={"scale_to_si": 0.0})


def _bad_mirror(m: RobotModel) -> None:
    m.parts[0].mirror = MirrorPlane(normal=(0.0, 0.0, 0.0))


def _loop_same_link(m: RobotModel) -> None:
    link = m.links[1].id
    m.loop_constraints.append(
        LoopConstraint(name="c", type=LoopConstraintType.CONNECT, link_a=link, link_b=link)
    )


def _loop_unknown_link(m: RobotModel) -> None:
    m.loop_constraints.append(
        LoopConstraint(
            name="c",
            type=LoopConstraintType.WELD,
            link_a=m.links[1].id,
            link_b=Link(name="ghost").id,
        )
    )


def _dup_constraint_name(m: RobotModel) -> None:
    a, b = m.links[1].id, m.links[2].id
    for _ in range(2):
        m.loop_constraints.append(
            LoopConstraint(name="c", type=LoopConstraintType.CONNECT, link_a=a, link_b=b)
        )


def _no_links(m: RobotModel) -> None:
    m.links.clear()
    m.joints.clear()
    m.parts.clear()


ERRORS: list[tuple[str, Mutation, IssueCode]] = [
    ("two roots", _two_roots, IssueCode.MULTIPLE_ROOTS),
    ("cycle, no root", _cycle, IssueCode.NO_ROOT),
    ("cycle, no root, links flagged", _cycle, IssueCode.CYCLE),
    ("cycle beside a root", _cycle_with_root, IssueCode.CYCLE),
    ("two parent joints", _two_parents, IssueCode.MULTIPLE_PARENTS),
    ("duplicate link name", _dup_link_name, IssueCode.DUPLICATE_LINK_NAME),
    ("duplicate joint name", _dup_joint_name, IssueCode.DUPLICATE_JOINT_NAME),
    ("duplicate constraint name", _dup_constraint_name, IssueCode.DUPLICATE_CONSTRAINT_NAME),
    ("empty name", _empty_name, IssueCode.EMPTY_NAME),
    ("duplicate id", _dup_id, IssueCode.DUPLICATE_ID),
    ("unknown child link", _unknown_child, IssueCode.UNKNOWN_LINK),
    ("unknown child leaves link unreachable", _unknown_child, IssueCode.MULTIPLE_ROOTS),
    ("self joint", _self_joint, IssueCode.SELF_JOINT),
    ("part in unknown link", _part_unknown_link, IssueCode.UNKNOWN_LINK),
    ("revolute without limits", _no_limits, IssueCode.MISSING_LIMITS),
    ("NaN limit", _nan_limit, IssueCode.LIMITS_NOT_FINITE),
    ("infinite velocity", _inf_velocity, IssueCode.LIMITS_NOT_FINITE),
    ("reversed limits", _reversed_limits, IssueCode.LIMITS_REVERSED),
    ("negative effort", _negative_effort, IssueCode.NEGATIVE_EFFORT_OR_VELOCITY),
    ("zero axis", _zero_axis, IssueCode.BAD_AXIS),
    ("NaN origin", _nan_origin, IssueCode.POSE_NOT_FINITE),
    ("non-unit quaternion", _non_unit_quat, IssueCode.QUAT_NOT_UNIT),
    ("negative damping", _negative_damping, IssueCode.BAD_DYNAMICS),
    ("zero density", _zero_density, IssueCode.NON_POSITIVE_DENSITY),
    ("zero measured mass", _zero_measured_mass, IssueCode.NON_POSITIVE_MASS),
    ("zero override mass", _zero_override_mass, IssueCode.NON_POSITIVE_MASS),
    ("NaN override CoM", _nan_part_override_com, IssueCode.POSE_NOT_FINITE),
    ("zero scale", _zero_scale, IssueCode.BAD_SCALE),
    ("degenerate mirror", _bad_mirror, IssueCode.BAD_MIRROR_NORMAL),
    ("loop on one link", _loop_same_link, IssueCode.LOOP_SAME_LINK),
    ("loop to unknown link", _loop_unknown_link, IssueCode.UNKNOWN_LINK),
    ("no links", _no_links, IssueCode.EMPTY_MODEL),
]


@pytest.mark.parametrize(
    ("mutate", "code"), [(m, c) for _, m, c in ERRORS], ids=[n for n, _, _ in ERRORS]
)
def test_bad_model_reports_error(mutate: Mutation, code: IssueCode) -> None:
    m = leg()
    mutate(m)
    report = validate(m)
    assert not report.ok
    assert code in {i.code for i in report.errors}


def _fixed_with_limits(m: RobotModel) -> None:
    m.joints[0].type = JointType.FIXED


def _continuous_with_limits(m: RobotModel) -> None:
    m.joints[0].type = JointType.CONTINUOUS


def _unassigned_part(m: RobotModel) -> None:
    m.parts[0].link = None


WARNINGS: list[tuple[str, Mutation, IssueCode]] = [
    ("fixed joint with limits", _fixed_with_limits, IssueCode.UNUSED_LIMITS),
    ("continuous joint with limits", _continuous_with_limits, IssueCode.UNUSED_LIMITS),
    ("unassigned part", _unassigned_part, IssueCode.UNASSIGNED_PART),
]


@pytest.mark.parametrize(
    ("mutate", "code"), [(m, c) for _, m, c in WARNINGS], ids=[n for n, _, _ in WARNINGS]
)
def test_suspicious_model_warns_but_passes(mutate: Mutation, code: IssueCode) -> None:
    m = leg()
    mutate(m)
    report = validate(m)
    assert report.ok
    assert [i.code for i in report.warnings] == [code]
    assert all(i.severity is Severity.WARNING for i in report.warnings)


def test_issue_subject_points_at_offending_item() -> None:
    m = leg()
    _reversed_limits(m)
    (issue,) = validate(m).errors
    assert issue.subject == m.joints[0].id


def test_fixed_joint_ignores_zero_axis() -> None:
    m = leg()
    m.joints[0].type = JointType.FIXED
    m.joints[0].limits = None
    m.joints[0].axis = (0.0, 0.0, 0.0)
    assert validate(m).issues == ()


def test_loop_constraint_between_tree_links_is_valid() -> None:
    m = leg()
    m.loop_constraints.append(
        LoopConstraint(
            name="close",
            type=LoopConstraintType.CONNECT,
            link_a=m.links[0].id,
            link_b=m.links[2].id,
        )
    )
    assert validate(m).ok


@settings(max_examples=50, deadline=None)
@given(st.integers(min_value=1, max_value=12), st.randoms(use_true_random=False))
def test_random_trees_are_valid(n: int, rnd: random.Random) -> None:
    links = [Link(name=f"l{i}") for i in range(n)]
    joints = [
        Joint(
            name=f"j{i}", type=JointType.FIXED, parent=links[rnd.randrange(i)].id, child=links[i].id
        )
        for i in range(1, n)
    ]
    rnd.shuffle(joints)
    rnd.shuffle(links)
    assert validate(RobotModel(name="t", links=links, joints=joints)).ok
