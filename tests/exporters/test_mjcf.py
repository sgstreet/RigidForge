"""Tests for the MJCF exporter (M1 task 11): compiled in MuJoCo, FK against URDF, inertia."""

import io
import math
import re
from pathlib import Path
from typing import Any
from uuid import UUID

import mujoco as _mujoco
import numpy as np
import pytest
import trimesh
from lxml import etree

from rigidforge.core.extensions import Extensions, MjcfExtension, MjcfPositionActuator
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
from rigidforge.core.transforms import quat_from_rotation
from rigidforge.exporters.common import MeshRef
from rigidforge.exporters.mjcf import MjcfExportError, export_mjcf
from rigidforge.exporters.urdf import export_urdf
from rigidforge.physics.inertia import MassProperties, solid_box, solid_cylinder

mujoco: Any = _mujoco  # mujoco's stubs hide most of its API from pyright


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
    base, arm, tip = Link(name="base"), Link(name="arm"), Link(name="tip")
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
                dynamics=JointDynamics(damping=0.1, friction=0.02),
                extensions=Extensions(
                    mjcf=MjcfExtension(
                        position_actuator=MjcfPositionActuator(
                            kp=20.0, kv=0.5, forcerange=(-5.0, 5.0), ctrlrange=(-1.0, 2.0)
                        )
                    )
                ),
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


def _compile(xml: bytes) -> Any:
    return mujoco.MjModel.from_xml_string(xml.decode())


def test_compiles_and_masses_match() -> None:
    model, masses = _model()
    m = _compile(export_mjcf(model, masses).files["two_dof.xml"])
    assert m.nbody == 4 and m.njnt == 2 and m.nu == 1
    for name, link in zip(("base", "arm", "tip"), model.links, strict=True):
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        assert m.body_mass[bid] == pytest.approx(masses[link.id].mass, rel=1e-12)
        # MuJoCo diagonalises: compare principal moments.
        want = np.sort(np.linalg.eigvalsh(masses[link.id].inertia))
        assert np.allclose(np.sort(m.body_inertia[bid]), want, rtol=1e-9)
        assert np.allclose(m.body_ipos[bid], masses[link.id].com, atol=1e-12)


def test_fullinertia_order_and_off_diagonals() -> None:
    model, _ = _model()
    i = np.array([[2.0, 0.1, 0.2], [0.1, 3.0, 0.3], [0.2, 0.3, 4.0]])
    masses = {link.id: MassProperties.of(1.0, (0.0, 0.0, 0.0), i) for link in model.links}
    root = etree.fromstring(export_mjcf(model, masses).files["two_dof.xml"])
    el = root.find(".//body[@name='base']/inertial")
    assert el is not None and el.get("fullinertia") == "2.0 3.0 4.0 0.1 0.2 0.3"
    # MuJoCo reconstructs the same full tensor from its diagonal form.
    m = _compile(export_mjcf(model, masses).files["two_dof.xml"])
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    q = np.zeros(9)
    mujoco.mju_quat2Mat(q, m.body_iquat[bid])
    r = q.reshape(3, 3)
    assert np.allclose(r @ np.diag(m.body_inertia[bid]) @ r.T, i, atol=1e-12)


def test_joint_attributes_and_actuator() -> None:
    model, masses = _model()
    m = _compile(export_mjcf(model, masses).files["two_dof.xml"])
    shoulder = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "shoulder")
    slide = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "slide")
    assert m.jnt_type[shoulder] == mujoco.mjtJoint.mjJNT_HINGE
    assert m.jnt_type[slide] == mujoco.mjtJoint.mjJNT_SLIDE
    assert np.allclose(m.jnt_range[shoulder], (-1.0, 2.0))
    assert np.allclose(m.jnt_axis[shoulder], (0.0, 1.0, 0.0))  # normalised
    assert m.dof_damping[m.jnt_dofadr[shoulder]] == pytest.approx(0.1)
    assert m.dof_frictionloss[m.jnt_dofadr[shoulder]] == pytest.approx(0.02)
    assert np.allclose(m.jnt_actfrcrange[shoulder], (-5.0, 5.0))
    assert m.actuator_gainprm[0, 0] == pytest.approx(20.0)
    assert m.actuator_biasprm[0, 1] == pytest.approx(-20.0)
    assert m.actuator_biasprm[0, 2] == pytest.approx(-0.5)
    assert np.allclose(m.actuator_forcerange[0], (-5.0, 5.0))
    assert np.allclose(m.actuator_ctrlrange[0], (-1.0, 2.0))


def test_fk_matches_urdf_at_random_poses() -> None:
    from yourdfpy import URDF

    model, masses = _model()
    m = _compile(export_mjcf(model, masses).files["two_dof.xml"])
    d = mujoco.MjData(m)
    robot = URDF.load(
        io.BytesIO(export_urdf(model, masses).files["two_dof.urdf"]),
        load_meshes=False,
        build_collision_scene_graph=False,
    )
    rng = np.random.default_rng(1)
    tip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tip")
    for _ in range(50):
        q = np.array([rng.uniform(-1.0, 2.0), rng.uniform(0.0, 0.05)])
        d.qpos[:] = q
        mujoco.mj_kinematics(m, d)
        robot.update_cfg(q)
        want = robot.get_transform("tip", "base")
        got = np.eye(4)
        got[:3, :3] = d.xmat[tip].reshape(3, 3)
        got[:3, 3] = d.xpos[tip]
        assert np.allclose(got, want, atol=1e-12)


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
    m = _compile(export_mjcf(model, masses).files["r.xml"])
    assert m.njnt == 1 and m.nbody == 4
    assert m.jnt_limited[0] == 0


def test_loop_constraints_compile() -> None:
    model, masses = _model()
    for ctype in LoopConstraintType:
        looped = model.model_copy(deep=True)
        looped.loop_constraints.append(
            LoopConstraint(
                name="loop",
                type=ctype,
                link_a=looped.links[0].id,
                link_b=looped.links[2].id,
                frame_a=Pose(xyz=(0.0, 0.0, 0.1)),
            )
        )
        m = _compile(export_mjcf(looped, masses).files["two_dof.xml"])
        assert m.neq == 1
        want = (
            mujoco.mjtEq.mjEQ_CONNECT
            if ctype is LoopConstraintType.CONNECT
            else mujoco.mjtEq.mjEQ_WELD
        )
        assert m.eq_type[0] == want


def test_meshes_compile(tmp_path: Path) -> None:
    model, masses = _model()
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    box = trimesh.creation.box(extents=(0.1, 0.2, 0.3))
    box.export(mesh_dir / "base.stl")
    base = model.links[0]
    ref = MeshRef("meshes/base.stl", Pose(xyz=(0.0, 0.0, 0.01)), rgba=(0.5, 0.5, 0.5, 1.0))
    out = export_mjcf(model, masses, {base.id: [ref]}, {base.id: [ref]})
    xml_path = tmp_path / "two_dof.xml"
    xml_path.write_bytes(out.files["two_dof.xml"])
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    assert m.nmesh == 1 and m.ngeom == 2
    # Visual geoms and collision geoms add no mass: body mass stays as given.
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    assert m.body_mass[bid] == pytest.approx(2.0)
    assert sorted(m.geom_group.tolist()) == [2, 3]


def test_deterministic_and_normalised_zero() -> None:
    model, masses = _model()
    a = export_mjcf(model, masses).files["two_dof.xml"]
    b = export_mjcf(model, masses).files["two_dof.xml"]
    assert a == b
    assert not re.search(rb"-0\.0(?![0-9])", a)


def test_errors() -> None:
    model, masses = _model()
    with pytest.raises(MjcfExportError, match="no mass properties"):
        export_mjcf(model, {k: v for k, v in masses.items() if k != model.links[0].id})

    bad_inertia = dict(masses)
    bad_inertia[model.links[0].id] = MassProperties.of(
        1.0, (0.0, 0.0, 0.0), np.diag([1.0, 1.0, 5.0])
    )
    with pytest.raises(MjcfExportError, match="triangle_inequality"):
        export_mjcf(model, bad_inertia)

    invalid = model.model_copy(deep=True)
    invalid.links[1].name = "base"
    with pytest.raises(MjcfExportError, match="not valid"):
        export_mjcf(invalid, masses)

    fixed = model.model_copy(deep=True)
    fixed.joints[1].extensions = Extensions(
        mjcf=MjcfExtension(position_actuator=MjcfPositionActuator(kp=1.0))
    )
    fixed.joints[1].type = JointType.FIXED
    with pytest.raises(MjcfExportError, match="fixed joint"):
        export_mjcf(fixed, masses)
