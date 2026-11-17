"""The hexapod leg end to end (M1 task 19): placement, masses, URDF and MJCF, FK comparison."""

import io
import itertools
import math
from pathlib import Path
from typing import Any

import mujoco as _mujoco
import numpy as np
import pytest
from OCP.Bnd import Bnd_Box
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.gp import gp_Trsf
from OCP.GProp import GProp_GProps
from yourdfpy import URDF

from rigidforge.core.model import Massless
from rigidforge.core.project import asset_path, load_project
from rigidforge.importers.step import read_step
from rigidforge.physics.brep import brep_unit_mass_properties
from rigidforge.pipeline import compute_link_masses, export_project
from rigidforge.testing.hexapod_leg import (
    DATA_DIR,
    EFFORT,
    FRAME_DENSITY,
    IDLE_CAP_DENSITY,
    IDLE_HORN_DENSITY,
    LIMITS,
    SERVO_MASS,
    SHAFT_MASS,
    LegBuild,
    build_leg,
    joint_names,
    servo_override,
)

mujoco: Any = _mujoco  # mujoco's stubs hide most of its API from pyright

MM = 1e-3


@pytest.fixture(scope="module")
def leg(tmp_path_factory: pytest.TempPathFactory) -> LegBuild:
    return build_leg(tmp_path_factory.mktemp("leg") / "hexapod-leg.rforge")


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    m = np.eye(4)
    m[0, 0], m[0, 1], m[1, 0], m[1, 1] = c, -s, s, c
    return m


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    m = np.eye(4)
    m[0, 0], m[0, 2], m[2, 0], m[2, 2] = c, s, -s, c
    return m


def _tx(x: float) -> np.ndarray:
    m = np.eye(4)
    m[0, 3] = x
    return m


def _fk(q: tuple[float, float, float]) -> dict[str, np.ndarray]:
    """Link frames in the base frame, from the layout dimensions (independent of the model)."""
    coxa = _rz(q[0])
    femur = coxa @ _tx(43.5 * MM) @ _ry(q[1])
    tibia = femur @ _tx(47.5 * MM) @ _ry(q[2])
    return {"base": np.eye(4), "coxa": coxa, "femur": femur, "tibia": tibia}


# --- Model and masses -----------------------------------------------------------------------


def test_leg_model_structure(leg: LegBuild) -> None:
    m = leg.model
    assert [lk.name for lk in m.links] == ["base", "coxa", "femur", "tibia"]
    assert [j.name for j in m.joints] == list(joint_names())
    for j, (kind, axis) in zip(
        m.joints,
        (("coxa", (0, 0, 1)), ("femur", (0, 1, 0)), ("tibia", (0, 1, 0))),
        strict=True,
    ):
        assert j.limits is not None
        assert (j.limits.lower, j.limits.upper) == LIMITS[kind]
        assert j.limits.effort == pytest.approx(0.52)
        assert j.axis == axis
    assert LIMITS["tibia"][0] == pytest.approx(-math.pi / 2)
    assert LIMITS["tibia"][1] == pytest.approx(math.radians(150.0))
    # 3 servos x (16 body leaves + 3 horn parts) + 6 frames + 1 shaft
    assert len(m.parts) == 3 * 19 + 7
    # the shared sources: one stored file per distinct CAD file
    assert len({p.source.asset for p in m.parts}) == 8
    assert sum(p.inertial_override is not None for p in m.parts) == 4  # 3 servos + shaft


def _volume(stem: str) -> float:
    imported = read_step(DATA_DIR / f"{stem}.step")
    (leaf,) = imported.leaves
    return brep_unit_mass_properties(imported.product_of(leaf).shape, stem).volume


def test_link_masses_match_geometry_and_datasheet(leg: LegBuild) -> None:
    masses = compute_link_masses(leg.model, leg.project)
    rho = FRAME_DENSITY
    idle = (
        _volume("xl330-idle-horn") * IDLE_HORN_DENSITY
        + _volume("xl330-idle-cal") * IDLE_CAP_DENSITY
    )
    # a servo's idle horn and cap turn with the child link; the 18 g (with drive horn) stays put
    expect = {
        "base": SERVO_MASS,
        "coxa": SERVO_MASS + 2 * _volume("coxa-frame") * rho + idle,
        "femur": SERVO_MASS + 2 * _volume("femur-frame") * rho + idle,
        "tibia": 2 * _volume("tibia-frame") * rho + idle + SHAFT_MASS,
    }
    for name, mass in expect.items():
        assert masses[leg.links[name]].mass == pytest.approx(mass, rel=1e-9), name
    # only the drive horn is inside the servo's 18 g; the idle horn and cap have their own mass
    for p in leg.model.parts:
        if p.name.endswith("drive_horn"):
            assert isinstance(p.mass, Massless)
        elif p.name.endswith(("idle_horn", "idle_cap")):
            assert not isinstance(p.mass, Massless) and p.mass is not None


def test_base_link_is_the_datasheet_servo(leg: LegBuild) -> None:
    ov = servo_override()
    props = compute_link_masses(leg.model, leg.project)[leg.links["base"]]
    assert props.mass == pytest.approx(0.018)
    # the PDF's CoM, mapped from its frame (origin on the horn face) to the servo centre frame
    np.testing.assert_allclose(props.com, ov.com, atol=1e-12)
    np.testing.assert_allclose(
        props.com, np.array([-9.2403727, 0.29100243, 1.378252]) * MM, atol=1e-9
    )
    # the tensor is a rotation of the PDF's: same trace and same principal moments
    pdf = (
        np.array(
            [
                [2555.5250, -20.247808, -11.076849],
                [-20.247808, 1271.1372, -229.79973],
                [-11.076849, -229.79973, 2271.2629],
            ]
        )
        * 1e-9
    )
    np.testing.assert_allclose(
        np.linalg.eigvalsh(props.inertia), np.linalg.eigvalsh(pdf), rtol=1e-12
    )
    assert props.inertia[0, 0] == pytest.approx(pdf[1, 1])  # x_cad = y_pdf
    assert props.inertia[1, 1] == pytest.approx(pdf[0, 0])  # y_cad = -x_pdf
    assert props.inertia[0, 2] == pytest.approx(pdf[1, 2])  # z_cad = z_pdf


def test_project_reload_exports_identically(leg: LegBuild, tmp_path: Path) -> None:
    first = export_project(leg.model, leg.project, tmp_path / "a", ("urdf", "mjcf"))
    loaded = load_project(leg.project).model
    second = export_project(loaded, leg.project, tmp_path / "b", ("urdf", "mjcf"))
    assert [p.name for p in first.written] == [p.name for p in second.written]
    for a, b in zip(first.written, second.written, strict=True):
        assert a.read_bytes() == b.read_bytes()


# --- Placement ------------------------------------------------------------------------------


def _world_shapes(leg: LegBuild) -> dict[str, Any]:
    steps: dict[Path, Any] = {}
    shapes: dict[str, Any] = {}
    for p in leg.model.parts:
        file = asset_path(leg.project, p.source.asset)
        imported = steps.setdefault(file, read_step(file))
        shape = imported.product_of(imported.leaf_at(p.source.assembly_path)).shape
        m = leg.world[p.name]
        t = gp_Trsf()
        t.SetValues(*[float(m[i, j]) for i in range(3) for j in range(4)])
        shapes[p.name] = BRepBuilderAPI_Transform(shape, t, True).Shape()
    return shapes


def _box(shape: Any) -> Any:
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    return box


def test_no_part_overlaps_except_the_shaft_press_fit(leg: LegBuild) -> None:
    shapes = _world_shapes(leg)
    group = lambda n: n.split("/")[0]  # noqa: E731  (a servo's own leaves touch by design)
    overlaps: dict[tuple[str, str], float] = {}
    for a, b in itertools.combinations(shapes, 2):
        if "/" in a and group(a) == group(b):
            continue
        if _box(shapes[a]).IsOut(_box(shapes[b])):
            continue
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(shapes[a], shapes[b]).Shape(), props)
        volume = props.Mass() / MM**3  # mm³
        if volume > 0.5:
            overlaps[(a, b)] = volume
    # r 3.76 mm shaft in r 3.58 mm grooves: 28.3 mm³ each, and nothing else
    assert {frozenset(pair) for pair in overlaps} == {
        frozenset(("tibia_frame_idle", "tibia_shaft")),
        frozenset(("tibia_frame_drive", "tibia_shaft")),
    }
    for volume in overlaps.values():
        assert volume == pytest.approx(28.3, abs=0.5)


def test_each_frame_seats_on_its_horn(leg: LegBuild) -> None:
    shapes = _world_shapes(leg)
    pairs = [
        (f"{servo}_servo/{horn}", f"{servo}_frame_{side}")
        for servo in ("coxa", "femur", "tibia")
        for horn, side in (("drive_horn", "drive"), ("idle_horn", "idle"))
    ]
    for horn, frame in pairs:
        dist = BRepExtrema_DistShapeShape(shapes[horn], shapes[frame])
        dist.Perform()
        assert dist.Value() < 1e-6, (horn, frame, dist.Value())


# --- Exports and FK -------------------------------------------------------------------------


def test_exports_compile_and_agree_at_random_poses(leg: LegBuild, tmp_path: Path) -> None:
    result = export_project(leg.model, leg.project, tmp_path, ("urdf", "mjcf"))
    assert result.warnings == ()
    files = {p.name: p.read_bytes() for p in result.written}
    robot = URDF.load(
        io.BytesIO(files["hexapod_leg.urdf"]), load_meshes=False, build_collision_scene_graph=False
    )
    m = mujoco.MjModel.from_xml_string(files["hexapod_leg.xml"].decode())
    d = mujoco.MjData(m)
    assert m.nbody == 5 and m.njnt == 3 and m.nu == 3  # world + 4 links
    for j, limits in enumerate(LIMITS.values()):
        assert np.allclose(m.jnt_range[j], limits)
        assert np.allclose(m.actuator_ctrlrange[j], limits)
        assert np.allclose(m.actuator_forcerange[j], (-EFFORT, EFFORT))
    rng = np.random.default_rng(19)
    for _ in range(50):
        q = tuple(rng.uniform(lo, hi) for lo, hi in LIMITS.values())
        d.qpos[:] = q
        mujoco.mj_kinematics(m, d)
        robot.update_cfg(np.array(q))
        want = _fk(q)  # type: ignore[arg-type]
        for name, expected in want.items():
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
            got = np.eye(4)
            got[:3, :3] = d.xmat[bid].reshape(3, 3)
            got[:3, 3] = d.xpos[bid]
            np.testing.assert_allclose(got, expected, atol=1e-12)
            np.testing.assert_allclose(robot.get_transform(name, "base"), expected, atol=1e-12)


def test_home_pose_tip_and_masses_in_mjcf(leg: LegBuild, tmp_path: Path) -> None:
    result = export_project(leg.model, leg.project, tmp_path, ("mjcf",))
    m = mujoco.MjModel.from_xml_path(str(result.written[0]))
    masses = compute_link_masses(leg.model, leg.project)
    for name, uid in leg.links.items():
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        assert m.body_mass[bid] == pytest.approx(masses[uid].mass, rel=1e-12)
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    # tibia axis at x = 91 mm; the shaft tip is 75 mm further along x at home
    tibia = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tibia")
    np.testing.assert_allclose(d.xpos[tibia], (91.0 * MM, 0.0, 0.0), atol=1e-12)
    tip = leg.link_world["tibia"] @ np.array([75.0 * MM, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(tip[:3], (166.0 * MM, 0.0, 0.0), atol=1e-12)


def test_shaft_is_a_hollow_tube_not_a_solid_rod(leg: LegBuild) -> None:
    shaft = next(p for p in leg.model.parts if p.name == "tibia_shaft")
    assert shaft.inertial_override is not None and shaft.mass is None
    assert shaft.inertial_override.mass == pytest.approx(1.15e-3 * 44.0 / 44.5)
    # a tube of this mass turns more slowly to spin than the same mass as a solid rod
    solid_axial = 0.5 * SHAFT_MASS * (3.76 * MM) ** 2
    assert shaft.inertial_override.inertia.izz > solid_axial
    # wall check: at 1.6 g/cm³ the inferred bore gives the same mass
    ro, ri, length = 3.76 * MM, 3.0 * MM, 0.044
    assert 1600.0 * math.pi * (ro**2 - ri**2) * length == pytest.approx(SHAFT_MASS, rel=0.05)
