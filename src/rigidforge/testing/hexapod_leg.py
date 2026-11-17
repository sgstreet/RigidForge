"""The hexapod leg (M1 task 19): three links driven by three Dynamixel XL330-M288 servos.

Builds a RigidForge project from the STEP files in ``tests/data/hexapod-leg``: a base holding
the coxa servo, a coxa link (coxa frame pair plus the femur servo), a femur link (femur frame
pair plus the tibia servo) and a tibia link (tibia frame pair plus the shaft), joined by a coxa
yaw, a femur pitch and a tibia pitch joint.

World frame (also the base and coxa link frame): origin on the coxa servo's horn axis, centred
in the servo body, z up along the coxa axis, x along the leg, y to the side. Home pose is 0 for
every joint. Lengths in the layout notes are in millimetres; everything stored is SI.

Layout (user-confirmed 2026-11-15 to 2026-11-17, see ``doc/milestones.md``):

* XL330 CAD frame: origin on the horn axis, centred in the body (z -11.5 to 11.5 mm), drive
  horn on z+, idle horn on z-, body length along x (-24.5 to 9.5 mm). Each horn's outer face is
  14.5 mm from the servo centre.
* Coxa servo: servo frame = world frame. Femur and tibia servos: servo z = world y and servo
  x = world x (the body extends back toward the previous joint), so the servo rotation is
  Rx(-90 degrees).
* Femur axis at x = 43.5 mm, tibia axis 47.5 mm further along the femur link, shaft 75 mm from
  the tibia axis to its tip.
* Each frame pair is one part used twice, the second copy rotated 180 degrees about the frame's
  x axis through the servo centre. Frames mate to the horn faces; the horn bolt pattern (4 bolts
  at 6 mm radius) is the placement reference.
* Horns and the idle cap rotate with the child link. Their mass is part of the servo's 18 g,
  so they carry no mass of their own.

Masses: the frames are PETG at 25 % infill (placeholder until the links are weighed), the shaft
is treated as solid PETG (assumption), and each servo has the manufacturer's mass, CoM and
inertia tensor for the XL330-M288 (ROBOTIS, "The moment of inertia", reference only, Feb 2023).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from uuid import UUID

import numpy as np
from numpy.typing import NDArray

from rigidforge.core.extensions import Extensions, MjcfExtension, MjcfPositionActuator
from rigidforge.core.geometry import InertiaTensor, Pose
from rigidforge.core.instancing import instantiate_assembly, rigid_copy
from rigidforge.core.model import (
    Density,
    InertialOverride,
    Joint,
    JointDynamics,
    JointLimits,
    JointType,
    Link,
    Massless,
    Part,
    RobotModel,
)
from rigidforge.core.project import add_asset, save_project
from rigidforge.core.transforms import pose_from_matrix
from rigidforge.core.validation import validate
from rigidforge.importers.step import StepImport, candidate_parts, read_step

__all__ = [
    "DATA_DIR",
    "FRAME_DENSITY",
    "SERVO_MASS",
    "LegBuild",
    "build_leg",
    "joint_names",
    "servo_override",
]

DATA_DIR = Path(__file__).resolve().parents[3] / "tests" / "data" / "hexapod-leg"

MM = 1e-3
FRAME_DENSITY = 0.25 * 1270.0
"""kg/m³: PETG at 25 % infill (placeholder; set from measured link masses later)."""
SHAFT_DENSITY = 1270.0
"""kg/m³: solid PETG, an assumption for the tibia shaft."""

HORN_OFFSET = 13.05 * MM
"""Horn file origin to servo centre along the servo axis: outer face 14.5 mm, file face 1.45 mm."""
FEMUR_X = 43.5 * MM
TIBIA_X = 47.5 * MM
FEMUR_FLANGE_CENTRE_Y = 16.5 * MM
"""Servo centre in the femur frame file, along its y axis."""
SHAFT_X = 31.0 * MM
"""Shaft start along the tibia frame's x axis (groove start)."""

SERVO_MASS = 0.018
EFFORT = 0.52
"""N·m: XL330-M288-T stall torque at 5.0 V (emanual.robotis.com, read 2026-11-17)."""
VELOCITY = 103.0 * 2.0 * math.pi / 60.0
"""rad/s: no-load speed at 5.0 V, 103 rev/min."""
ACTUATOR = MjcfPositionActuator(kp=2.0, kv=0.05, forcerange=(-EFFORT, EFFORT))
"""Rough position gains (N·m/rad, N·m·s/rad); tune against the real servo later."""
DYNAMICS = JointDynamics(damping=0.02, friction=0.0)

LIMITS = {
    "coxa": (-math.pi / 2, math.pi / 2),
    "femur": (-math.pi / 2, math.pi / 2),
    "tibia": (-math.pi / 2, math.radians(150.0)),
}


def joint_names() -> tuple[str, str, str]:
    return ("coxa_yaw", "femur_pitch", "tibia_pitch")


def _t(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> NDArray[np.float64]:
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def _rx(angle: float) -> NDArray[np.float64]:
    c, s = math.cos(angle), math.sin(angle)
    m = np.eye(4)
    m[1, 1], m[1, 2], m[2, 1], m[2, 2] = c, -s, s, c
    return m


def _ry(angle: float) -> NDArray[np.float64]:
    c, s = math.cos(angle), math.sin(angle)
    m = np.eye(4)
    m[0, 0], m[0, 2], m[2, 0], m[2, 2] = c, s, -s, c
    return m


def servo_override() -> InertialOverride:
    """XL330-M288 mass, CoM and tensor in the XL330 CAD frame (metres, kg, kg·m²).

    The PDF frame has its origin on the horn's outer face, z toward the horn, y along the body
    length (up in the front view) and x across the body. The CAD frame has its origin at the
    body centre, so: x_cad = y_pdf, y_cad = -x_pdf, z_cad = z_pdf + 14.5 mm. The tensor is taken
    about the CoM (the usual convention for these tables; the PDF does not say).
    """
    com_pdf = np.array([-2.9100243e-01, -9.2403727e00, -1.3121748e01]) * MM
    i_pdf = (
        np.array(
            [
                [2.5555250e03, -2.0247808e01, -1.1076849e01],
                [-2.0247808e01, 1.2711372e03, -2.2979973e02],
                [-1.1076849e01, -2.2979973e02, 2.2712629e03],
            ]
        )
        * 1e-9
    )
    r = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    com = r @ com_pdf + np.array([0.0, 0.0, 14.5 * MM])
    i = r @ i_pdf @ r.T
    return InertialOverride(
        mass=SERVO_MASS,
        com=(float(com[0]), float(com[1]), float(com[2])),
        inertia=InertiaTensor(
            ixx=float(i[0, 0]),
            iyy=float(i[1, 1]),
            izz=float(i[2, 2]),
            ixy=float(i[0, 1]),
            ixz=float(i[0, 2]),
            iyz=float(i[1, 2]),
        ),
        source="ROBOTIS XL330/XC330 moment of inertia table (XL330-M288), reference only",
    )


@dataclass(frozen=True)
class LegBuild:
    model: RobotModel
    project: Path
    links: dict[str, UUID]
    """Link IDs by name: ``base``, ``coxa``, ``femur``, ``tibia``."""
    world: dict[str, NDArray[np.float64]]
    """World matrix of each part by part name (metres), for checks."""
    link_world: dict[str, NDArray[np.float64]]
    """World matrix of each link frame at the home pose."""


def build_leg(project: Path, data_dir: Path = DATA_DIR) -> LegBuild:
    """Import the leg's STEP files into ``project`` and save the model there."""
    project.mkdir(parents=True, exist_ok=True)
    stored: dict[str, tuple[StepImport, str, str]] = {}
    for stem in (
        "xl330-Body",
        "xl330-drive-horn",
        "xl330-idle-horn",
        "xl330-idle-cal",
        "coxa-frame",
        "femur-frame",
        "tibia-frame",
        "tiba-shaft",
    ):
        path = data_dir / f"{stem}.step"
        asset, sha = add_asset(project, path)
        stored[stem] = (read_step(path), asset, sha)

    def cands(stem: str) -> list[Part]:
        imported, asset, sha = stored[stem]
        return candidate_parts(imported, asset, sha)

    links = {name: Link(name=name) for name in ("base", "coxa", "femur", "tibia")}
    link_world: dict[str, NDArray[np.float64]] = {
        "base": np.eye(4),
        "coxa": np.eye(4),
        "femur": _t(FEMUR_X),
        "tibia": _t(FEMUR_X + TIBIA_X),
    }
    s_coxa = np.eye(4)
    s_femur = _t(FEMUR_X) @ _rx(-math.pi / 2)
    s_tibia = _t(FEMUR_X + TIBIA_X) @ _rx(-math.pi / 2)
    rx180 = _rx(math.pi)

    body_override = servo_override()
    parts: list[Part] = []
    world: dict[str, NDArray[np.float64]] = {}

    def in_link(link: str, matrix: NDArray[np.float64]) -> Pose:
        return pose_from_matrix(np.linalg.inv(link_world[link]) @ matrix)

    def add_servo(name: str, servo: NDArray[np.float64], body_link: str, horn_link: str) -> None:
        body = instantiate_assembly(
            cands("xl330-Body"),
            in_link(body_link, servo),
            link=links[body_link].id,
            prefix=f"{name}/",
            override=body_override,
        )
        parts.extend(body)
        for p, leaf in zip(body, stored["xl330-Body"][0].leaves, strict=True):
            world[p.name] = servo @ leaf.transform
        for stem, label, z in (
            ("xl330-drive-horn", "drive_horn", HORN_OFFSET),
            ("xl330-idle-horn", "idle_horn", -HORN_OFFSET),
            ("xl330-idle-cal", "idle_cap", -HORN_OFFSET),
        ):
            (cand,) = cands(stem)
            m = servo @ _t(z=z)
            horn = rigid_copy(
                cand,
                name=f"{name}/{label}",
                placement=in_link(horn_link, m),
                link=links[horn_link].id,
            ).model_copy(update={"mass": Massless()})
            parts.append(horn)
            world[horn.name] = m

    def add_one(
        stem: str, name: str, link: str, matrix: NDArray[np.float64], density: float
    ) -> None:
        (cand,) = cands(stem)
        part = rigid_copy(
            cand, name=name, placement=in_link(link, matrix), link=links[link].id
        ).model_copy(update={"mass": Density(density=density)})
        parts.append(part)
        world[name] = matrix

    # Coxa servo in the base; its horns and the coxa frame pair turn with the coxa link.
    add_servo("coxa_servo", s_coxa, "base", "coxa")
    add_one("coxa-frame", "coxa_frame_drive", "coxa", s_coxa, FRAME_DENSITY)
    add_one("coxa-frame", "coxa_frame_idle", "coxa", s_coxa @ rx180, FRAME_DENSITY)
    # Femur servo body rides on the coxa link; its horns and the femur frames turn with the femur.
    add_servo("femur_servo", s_femur, "coxa", "femur")
    femur_drive = _t(FEMUR_X, FEMUR_FLANGE_CENTRE_Y) @ rx180
    femur_idle = _t(FEMUR_X, -FEMUR_FLANGE_CENTRE_Y)
    add_one("femur-frame", "femur_frame_drive", "femur", femur_drive, FRAME_DENSITY)
    add_one("femur-frame", "femur_frame_idle", "femur", femur_idle, FRAME_DENSITY)
    # Tibia servo body rides on the femur link; its horns, the tibia frames and the shaft turn
    # with the tibia.
    add_servo("tibia_servo", s_tibia, "femur", "tibia")
    tibia_origin = _t(FEMUR_X + TIBIA_X)
    add_one("tibia-frame", "tibia_frame_idle", "tibia", tibia_origin, FRAME_DENSITY)
    add_one("tibia-frame", "tibia_frame_drive", "tibia", tibia_origin @ rx180, FRAME_DENSITY)
    # The shaft file's axis is z, 0 to 44 mm; it runs along the tibia's x from the groove start.
    add_one(
        "tiba-shaft",
        "tibia_shaft",
        "tibia",
        tibia_origin @ _t(x=SHAFT_X) @ _ry(math.pi / 2),
        SHAFT_DENSITY,
    )

    def joint(
        name: str, kind: str, parent: str, child: str, axis: tuple[float, float, float]
    ) -> Joint:
        lo, hi = LIMITS[kind]
        origin = np.linalg.inv(link_world[parent]) @ link_world[child]
        return Joint(
            name=name,
            type=JointType.REVOLUTE,
            parent=links[parent].id,
            child=links[child].id,
            origin=pose_from_matrix(origin),
            axis=axis,
            limits=JointLimits(lower=lo, upper=hi, effort=EFFORT, velocity=VELOCITY),
            dynamics=DYNAMICS,
            extensions=Extensions(
                mjcf=MjcfExtension(
                    position_actuator=ACTUATOR.model_copy(update={"ctrlrange": (lo, hi)})
                )
            ),
        )

    coxa, femur, tibia = joint_names()
    joints = [
        joint(coxa, "coxa", "base", "coxa", (0.0, 0.0, 1.0)),
        joint(femur, "femur", "coxa", "femur", (0.0, 1.0, 0.0)),
        joint(tibia, "tibia", "femur", "tibia", (0.0, 1.0, 0.0)),
    ]
    model = RobotModel(name="hexapod_leg", links=list(links.values()), joints=joints, parts=parts)
    report = validate(model)
    if not report.ok:
        raise ValueError("; ".join(f"{i.code}: {i.message}" for i in report.errors))
    save_project(model, project)
    return LegBuild(model, project, {k: v.id for k, v in links.items()}, world, link_world)


def main() -> None:
    """``python -m rigidforge.testing.hexapod_leg OUT_DIR``: build, save and export the leg."""
    import sys

    from rigidforge.pipeline import export_project

    if len(sys.argv) != 2:
        raise SystemExit("usage: python -m rigidforge.testing.hexapod_leg OUT_DIR")
    out = Path(sys.argv[1])
    build = build_leg(out / "hexapod-leg.rforge")
    result = export_project(build.model, build.project, out / "export", ("urdf", "mjcf"))
    for path in result.written:
        print(path)
    for warning in result.warnings:
        print("warning:", warning)


if __name__ == "__main__":
    main()
