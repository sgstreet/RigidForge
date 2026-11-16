"""Tests for the core robot model (task 2)."""

import json
from uuid import UUID, uuid4

import pytest
from pydantic import ValidationError

from rigidforge.core import (
    SCHEMA_VERSION,
    Density,
    Extensions,
    InertialOverride,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    LoopConstraint,
    LoopConstraintType,
    MeasuredMass,
    MirrorPlane,
    MjcfExtension,
    MjcfPositionActuator,
    Part,
    PartSource,
    Pose,
    RobotModel,
    SnapHint,
    SourceKind,
)


def two_link_robot() -> RobotModel:
    """A base, one revolute joint and an arm, with a part on each link."""
    base = Link(name="base")
    arm = Link(name="arm")
    source = PartSource(
        kind=SourceKind.STEP, asset="frame-ab12.step", sha256="ab12", scale_to_si=1e-3
    )
    base_part = Part(name="base-frame", source=source, link=base.id, mass=Density(density=1270.0))
    arm_part = Part(
        name="arm-frame",
        source=source,
        link=arm.id,
        placement=Pose(xyz=(0.0, 0.0, 0.05)),
        mirror=MirrorPlane(normal=(0.0, 1.0, 0.0)),
        mass=MeasuredMass(mass=0.012),
    )
    shoulder = Joint(
        name="shoulder",
        type=JointType.REVOLUTE,
        parent=base.id,
        child=arm.id,
        origin=Pose(xyz=(0.0, 0.0, 0.1), quat=(0.7071067811865476, 0.0, 0.7071067811865476, 0.0)),
        axis=(0.0, 1.0, 0.0),
        limits=JointLimits(lower=-1.5, upper=1.5, effort=0.6, velocity=8.0),
        snap_hint=SnapHint(part=base_part.id, face_index=7, feature="cylinder r=0.0035"),
        extensions=Extensions(mjcf=MjcfExtension(position_actuator=MjcfPositionActuator(kp=5.0))),
    )
    return RobotModel(
        name="two-link", links=[base, arm], joints=[shoulder], parts=[base_part, arm_part]
    )


def test_defaults() -> None:
    robot = RobotModel(name="empty")
    assert robot.schema_version == SCHEMA_VERSION
    assert robot.links == [] and robot.joints == [] and robot.parts == []
    assert robot.extensions.is_empty()
    assert Pose().xyz == (0.0, 0.0, 0.0)
    assert Pose().quat == (1.0, 0.0, 0.0, 0.0)


def test_ids_are_unique_uuids() -> None:
    links = [Link(name="same") for _ in range(100)]
    assert all(isinstance(link.id, UUID) for link in links)
    assert len({link.id for link in links}) == 100


def test_json_round_trip_is_equal() -> None:
    robot = two_link_robot()
    loaded = RobotModel.model_validate_json(robot.model_dump_json())
    assert loaded == robot


def test_json_round_trip_is_stable() -> None:
    text = two_link_robot().model_dump_json()
    assert RobotModel.model_validate_json(text).model_dump_json() == text


def test_ids_survive_round_trip_and_references_resolve() -> None:
    robot = RobotModel.model_validate_json(two_link_robot().model_dump_json())
    joint = robot.joints[0]
    assert robot.link(joint.parent).name == "base"
    assert robot.link(joint.child).name == "arm"
    assert [p.name for p in robot.parts_of(joint.child)] == ["arm-frame"]
    assert robot.child_joints(joint.parent) == [joint]
    assert robot.joint(joint.id) is joint


def test_lookup_by_unknown_id_raises() -> None:
    robot = two_link_robot()
    with pytest.raises(KeyError):
        robot.link(uuid4())
    with pytest.raises(KeyError):
        robot.part(uuid4())


def test_names_are_not_keys() -> None:
    """Renaming a link changes nothing that refers to it."""
    robot = two_link_robot()
    arm_id = robot.joints[0].child
    robot.link(arm_id).name = "upper-arm"
    assert robot.link(robot.joints[0].child).name == "upper-arm"


def test_mass_source_discriminator() -> None:
    data = json.loads(two_link_robot().model_dump_json())
    assert data["parts"][0]["mass"] == {"kind": "density", "density": 1270.0}
    assert data["parts"][1]["mass"] == {"kind": "measured_mass", "mass": 0.012}
    loaded = RobotModel.model_validate(data)
    assert isinstance(loaded.parts[0].mass, Density)
    assert isinstance(loaded.parts[1].mass, MeasuredMass)


def test_unknown_mass_kind_rejected() -> None:
    data = json.loads(two_link_robot().model_dump_json())
    data["parts"][0]["mass"] = {"kind": "guess", "mass": 1.0}
    with pytest.raises(ValidationError):
        RobotModel.model_validate(data)


def test_unknown_core_field_rejected() -> None:
    data = json.loads(two_link_robot().model_dump_json())
    data["links"][0]["colour"] = "red"
    with pytest.raises(ValidationError):
        RobotModel.model_validate(data)


def test_vector_shape_enforced() -> None:
    with pytest.raises(ValidationError):
        Pose(xyz=(1.0, 2.0))  # type: ignore[arg-type]
    with pytest.raises(ValidationError):
        Pose(quat=(1.0, 0.0, 0.0))  # type: ignore[arg-type]


def test_assignment_is_validated() -> None:
    joint = two_link_robot().joints[0]
    with pytest.raises(ValidationError):
        joint.type = "ball"  # type: ignore[assignment]


def test_semantic_problems_still_load() -> None:
    """Bad values are validation's job (task 3), so they must load, not raise."""
    limits = JointLimits(lower=float("nan"), upper=-1.0)
    assert limits.upper == -1.0
    assert Density(density=-5.0).density == -5.0
    assert Pose(quat=(2.0, 0.0, 0.0, 0.0)).quat[0] == 2.0


def test_unknown_extension_keys_round_trip_at_every_level() -> None:
    robot = two_link_robot()
    data = json.loads(robot.model_dump_json())
    future = {"solver": {"iterations": 50}, "tags": ["a", "b"]}
    data["extensions"]["isaac"] = future  # unknown namespace
    data["extensions"]["mjcf"] = {"timestep": 0.002}  # unknown key in a known block
    data["links"][0]["extensions"]["sdf"] = {"self_collide": True}
    data["joints"][0]["extensions"]["mjcf"]["position_actuator"]["gear"] = 2.0
    data["joints"][0]["extensions"]["usd"] = {"drive_stiffness": 100.0}

    loaded = RobotModel.model_validate(data)
    dumped = json.loads(loaded.model_dump_json())
    assert dumped["extensions"]["isaac"] == future
    assert dumped["extensions"]["mjcf"]["timestep"] == 0.002
    assert dumped["links"][0]["extensions"]["sdf"] == {"self_collide": True}
    assert dumped["joints"][0]["extensions"]["mjcf"]["position_actuator"]["gear"] == 2.0
    assert dumped["joints"][0]["extensions"]["usd"] == {"drive_stiffness": 100.0}
    assert RobotModel.model_validate(dumped) == loaded
    assert loaded.extensions.mjcf is not None
    assert loaded.extensions.mjcf.unknown_keys() == {"timestep": 0.002}
    assert not loaded.extensions.is_empty()


def test_inertial_override_round_trip() -> None:
    override = InertialOverride(
        mass=0.018,
        com=(-0.007, 0.0, 0.0),
        inertia=InertiaTensor(ixx=1.2e-6, iyy=2.3e-6, izz=2.1e-6, ixy=1e-8),
        source="XL330 datasheet",
    )
    link = Link(name="servo", inertial_override=override)
    assert Link.model_validate_json(link.model_dump_json()) == link
    matrix = override.inertia.as_matrix()
    assert matrix[0][1] == matrix[1][0] == 1e-8


def test_loop_constraint_stub() -> None:
    robot = two_link_robot()
    loop = LoopConstraint(
        name="closure",
        type=LoopConstraintType.CONNECT,
        link_a=robot.links[0].id,
        link_b=robot.links[1].id,
        frame_b=Pose(xyz=(0.02, 0.0, 0.0)),
    )
    robot.loop_constraints.append(loop)
    assert RobotModel.model_validate_json(robot.model_dump_json()).loop_constraints == [loop]


def test_instanced_parts_share_a_source() -> None:
    source = PartSource(kind=SourceKind.STEP, asset="xl330-1f.step", sha256="1f", scale_to_si=1e-3)
    a = Part(name="servo-1", source=source)
    b = Part(name="servo-2", source=source, placement=Pose(xyz=(0.1, 0.0, 0.0)))
    assert a.source == b.source and a.id != b.id
    assert a.link is None  # unassigned candidate
