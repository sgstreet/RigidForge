"""Tests for the URDF exporter (M1 task 10): inertia against analytics, FK in yourdfpy."""

import io
import math
import re
from uuid import UUID

import numpy as np
import pytest
from hypothesis import given
from hypothesis import strategies as st
from lxml import etree

from rigidforge.core.geometry import Pose
from rigidforge.core.model import (
    Joint,
    JointDynamics,
    JointLimits,
    JointType,
    Link,
    LoopConstraint,
    LoopConstraintType,
    RobotModel,
)
from rigidforge.core.transforms import quat_from_rotation, rotation_from_quat
from rigidforge.exporters.urdf import (
    MeshRef,
    UrdfExportError,
    export_urdf,
    rpy_from_rotation,
)
from rigidforge.physics.inertia import MassProperties, solid_box, solid_cylinder


def _rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _model() -> tuple[RobotModel, dict[UUID, MassProperties]]:
    base = Link(name="base")
    arm = Link(name="arm")
    tip = Link(name="tip")
    pose = Pose(xyz=(0.1, -0.02, 0.3), quat=quat_from_rotation(_rot(0.3, -0.5, 1.1)))
    model = RobotModel(
        name="two_dof",
        links=[base, arm, tip],
        joints=[
            Joint(
                name="shoulder",
                type=JointType.REVOLUTE,
                parent=base.id,
                child=arm.id,
                origin=pose,
                axis=(0.0, 2.0, 0.0),
                limits=JointLimits(lower=-1.0, upper=2.0, effort=5.0, velocity=3.0),
                dynamics=JointDynamics(damping=0.1),
            ),
            Joint(
                name="slide",
                type=JointType.PRISMATIC,
                parent=arm.id,
                child=tip.id,
                origin=Pose(xyz=(0.2, 0.0, 0.0)),
                axis=(1.0, 0.0, 0.0),
                limits=JointLimits(lower=0.0, upper=0.05, effort=10.0, velocity=1.0),
            ),
        ],
    )
    masses = {
        base.id: solid_box(2.0, (0.1, 0.2, 0.3)),
        arm.id: solid_cylinder(0.5, 0.02, 0.2),
        tip.id: solid_box(0.1, (0.02, 0.02, 0.02)),
    }
    return model, masses


def _parse(data: bytes) -> etree._Element:
    return etree.fromstring(data)


@given(
    st.floats(-math.pi, math.pi),
    st.floats(-math.pi / 2, math.pi / 2),
    st.floats(-math.pi, math.pi),
)
def test_rpy_roundtrip(roll: float, pitch: float, yaw: float) -> None:
    r = _rot(roll, pitch, yaw)
    assert np.allclose(_rot(*rpy_from_rotation(r)), r, atol=1e-7)


@pytest.mark.parametrize("pitch", [math.pi / 2, -math.pi / 2])
@pytest.mark.parametrize("roll,yaw", [(0.4, 0.9), (-1.0, 2.0), (0.0, 0.0)])
def test_rpy_gimbal_lock(pitch: float, roll: float, yaw: float) -> None:
    r = _rot(roll, pitch, yaw)
    assert np.allclose(_rot(*rpy_from_rotation(r)), r, atol=1e-12)


def test_inertial_matches_analytic_box() -> None:
    model, masses = _model()
    out = export_urdf(model, masses)
    root = _parse(out.files["two_dof.urdf"])
    base = root.find("link[@name='base']/inertial")
    assert base is not None
    inertia = base.find("inertia")
    assert inertia is not None
    m = 2.0
    assert float(inertia.get("ixx", "")) == pytest.approx(m / 12 * (0.2**2 + 0.3**2), rel=1e-12)
    assert float(inertia.get("iyy", "")) == pytest.approx(m / 12 * (0.1**2 + 0.3**2), rel=1e-12)
    assert float(inertia.get("izz", "")) == pytest.approx(m / 12 * (0.1**2 + 0.2**2), rel=1e-12)
    assert float(inertia.get("ixy", "")) == 0.0
    mass = base.find("mass")
    assert mass is not None and float(mass.get("value", "")) == 2.0


def test_urdf_order_is_ixx_ixy_ixz_iyy_iyz_izz() -> None:
    model, _ = _model()
    i = np.array([[2.0, 0.1, 0.2], [0.1, 3.0, 0.3], [0.2, 0.3, 4.0]])
    masses = {link.id: MassProperties.of(1.0, (0.0, 0.0, 0.0), i) for link in model.links}
    root = _parse(export_urdf(model, masses).files["two_dof.urdf"])
    el = root.find("link[@name='base']/inertial/inertia")
    assert el is not None
    assert dict(el.attrib) == {
        "ixx": "2.0",
        "ixy": "0.1",
        "ixz": "0.2",
        "iyy": "3.0",
        "iyz": "0.3",
        "izz": "4.0",
    }


def test_fk_matches_yourdfpy() -> None:
    from yourdfpy import URDF

    model, masses = _model()
    data = export_urdf(model, masses).files["two_dof.urdf"]
    robot = URDF.load(
        io.BytesIO(data),
        load_meshes=False,
        build_collision_scene_graph=False,
        build_scene_graph=True,
    )
    shoulder, slide = model.joints
    axis = np.array(shoulder.axis) / np.linalg.norm(shoulder.axis)
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = float(rng.uniform(-1.0, 2.0))
        d = float(rng.uniform(0.0, 0.05))
        robot.update_cfg(np.array([q, d]))
        got = robot.get_transform("tip", "base")
        t1 = np.eye(4)
        t1[:3, :3] = rotation_from_quat(shoulder.origin.quat)
        t1[:3, 3] = shoulder.origin.xyz
        rq = np.eye(4)
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        rq[:3, :3] = np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * (k @ k)
        t2 = np.eye(4)
        t2[:3, 3] = slide.origin.xyz
        sl = np.eye(4)
        sl[0, 3] = d
        want = t1 @ rq @ t2 @ sl
        assert np.allclose(got, want, atol=1e-12)


def test_deterministic_and_normalised_zero() -> None:
    model, masses = _model()
    a = export_urdf(model, masses).files["two_dof.urdf"]
    b = export_urdf(model, masses).files["two_dof.urdf"]
    assert a == b
    assert not re.search(rb"-0\.0(?![0-9])", a)


def test_meshes_and_warnings() -> None:
    model, masses = _model()
    base = model.links[0]
    model.loop_constraints.append(
        LoopConstraint(
            name="loop", type=LoopConstraintType.CONNECT, link_a=base.id, link_b=model.links[2].id
        )
    )
    ref = MeshRef("meshes/base.stl", Pose(xyz=(0.0, 0.0, 0.01)), rgba=(0.5, 0.5, 0.5, 1.0))
    out = export_urdf(model, masses, {base.id: [ref]}, {base.id: [MeshRef("meshes/base_c.stl")]})
    root = _parse(out.files["two_dof.urdf"])
    mesh = root.find("link[@name='base']/visual/geometry/mesh")
    assert mesh is not None and mesh.get("filename") == "meshes/base.stl"
    col = root.find("link[@name='base']/collision/geometry/mesh")
    assert col is not None and col.get("filename") == "meshes/base_c.stl"
    assert root.find("link[@name='base']/visual/material/color") is not None
    assert len(out.warnings) == 1 and "loop" in out.warnings[0]


def test_fixed_and_continuous_joints() -> None:
    a, b, c = Link(name="a"), Link(name="b"), Link(name="c")
    model = RobotModel(
        name="r",
        links=[a, b, c],
        joints=[
            Joint(name="f", type=JointType.FIXED, parent=a.id, child=b.id),
            Joint(name="w", type=JointType.CONTINUOUS, parent=b.id, child=c.id),
        ],
    )
    masses = {x.id: solid_box(1.0, (0.1, 0.1, 0.1)) for x in model.links}
    root = _parse(export_urdf(model, masses).files["r.urdf"])
    assert root.find("joint[@name='f']/axis") is None
    assert root.find("joint[@name='w']/axis") is not None
    assert root.find("joint[@name='w']/limit") is None


def test_errors() -> None:
    model, masses = _model()
    with pytest.raises(UrdfExportError, match="no mass properties"):
        export_urdf(model, {k: v for k, v in masses.items() if k != model.links[0].id})

    no_effort = model.model_copy(deep=True)
    no_effort.joints[0].limits = JointLimits(lower=0.0, upper=1.0)
    with pytest.raises(UrdfExportError, match="effort and velocity"):
        export_urdf(no_effort, masses)

    bad = model.model_copy(deep=True)
    bad.links[1].name = "base"
    with pytest.raises(UrdfExportError, match="not valid"):
        export_urdf(bad, masses)
