"""URDF exporter: a pure function from a ``RobotModel`` to URDF bytes (M1 task 10).

The URDF is generated directly from the internal model (never derived from another format).
Everything is SI. Inputs the model does not hold (derived data) are passed in:

* ``masses`` maps each link ID to its mass properties in the link frame (about the CoM);
* ``visuals`` and ``collisions`` optionally map link IDs to mesh references already written
  by the caller (URDF refers to meshes by URI; writing them is not this module's job).

Output is deterministic: model order, ``repr`` floats with ``-0.0`` normalised, fixed
attribute order. Features URDF cannot express (loop constraints) are reported as warnings.
"""

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from uuid import UUID

import numpy as np
from lxml import etree
from lxml.etree import _Element as Element  # pyright: ignore[reportPrivateUsage]

from rigidforge.core.geometry import Pose
from rigidforge.core.model import Joint, JointType, Link, RobotModel
from rigidforge.core.transforms import rotation_from_quat
from rigidforge.core.validation import validate
from rigidforge.physics.inertia import MassProperties

__all__ = ["MeshRef", "UrdfExport", "UrdfExportError", "export_urdf", "rpy_from_rotation"]


class UrdfExportError(ValueError):
    """The model cannot be written as URDF."""


@dataclass(frozen=True, slots=True)
class MeshRef:
    """A mesh file for a link's visual or collision geometry, in metres.

    ``uri`` is written as given (for example ``meshes/coxa.stl`` or ``package://...``).
    ``origin`` places the mesh file's frame in the link frame.
    """

    uri: str
    origin: Pose = field(default_factory=Pose)
    rgba: tuple[float, float, float, float] | None = None


@dataclass(frozen=True, slots=True)
class UrdfExport:
    """The result: file name to content, and non-fatal notes about dropped features."""

    files: dict[str, bytes]
    warnings: tuple[str, ...] = ()


def _num(x: float) -> str:
    if not math.isfinite(x):
        raise UrdfExportError(f"non-finite number {x!r}")
    return repr(0.0 if x == 0.0 else float(x))


def _vec(v: Sequence[float]) -> str:
    return " ".join(_num(float(x)) for x in v)


def rpy_from_rotation(rot: np.ndarray) -> tuple[float, float, float]:
    """URDF roll, pitch, yaw (``R = Rz(yaw) Ry(pitch) Rx(roll)``) of a rotation matrix."""
    r = np.asarray(rot, dtype=np.float64)
    s = float(-r[2, 0])
    if abs(s) < 1.0 - 1e-12:
        pitch = math.asin(s)
        return (
            math.atan2(float(r[2, 1]), float(r[2, 2])),
            pitch,
            math.atan2(float(r[1, 0]), float(r[0, 0])),
        )
    # Gimbal lock: only roll + yaw (or roll - yaw) is determined; put it all in roll.
    if s > 0.0:
        return (math.atan2(float(r[0, 1]), float(r[1, 1])), math.pi / 2, 0.0)
    return (math.atan2(-float(r[0, 1]), float(r[1, 1])), -math.pi / 2, 0.0)


def _origin(parent: Element, xyz: Sequence[float], rot: np.ndarray) -> None:
    roll, pitch, yaw = rpy_from_rotation(rot)
    etree.SubElement(parent, "origin", xyz=_vec(xyz), rpy=_vec((roll, pitch, yaw)))


def _pose_origin(parent: Element, pose: Pose) -> None:
    _origin(parent, pose.xyz, rotation_from_quat(pose.quat))


def _inertial(link_el: Element, props: MassProperties) -> None:
    el = etree.SubElement(link_el, "inertial")
    _origin(el, props.com_tuple, np.eye(3))
    etree.SubElement(el, "mass", value=_num(props.mass))
    i = props.inertia
    etree.SubElement(
        el,
        "inertia",
        ixx=_num(float(i[0, 0])),
        ixy=_num(float(i[0, 1])),
        ixz=_num(float(i[0, 2])),
        iyy=_num(float(i[1, 1])),
        iyz=_num(float(i[1, 2])),
        izz=_num(float(i[2, 2])),
    )


def _geometry(link_el: Element, tag: str, refs: Sequence[MeshRef], index: int) -> None:
    for n, ref in enumerate(refs):
        el = etree.SubElement(link_el, tag)
        _pose_origin(el, ref.origin)
        geom = etree.SubElement(el, "geometry")
        etree.SubElement(geom, "mesh", filename=ref.uri)
        if tag == "visual" and ref.rgba is not None:
            mat = etree.SubElement(el, "material", name=f"mat_{index}_{n}")
            etree.SubElement(mat, "color", rgba=_vec(ref.rgba))


def _link(
    link: Link,
    index: int,
    props: MassProperties,
    visuals: Sequence[MeshRef],
    collisions: Sequence[MeshRef],
) -> Element:
    el = etree.Element("link", name=link.name)
    _inertial(el, props)
    _geometry(el, "visual", visuals, index)
    _geometry(el, "collision", collisions, index)
    return el


def _joint(model: RobotModel, joint: Joint) -> Element:
    el = etree.Element("joint", name=joint.name, type=str(joint.type))
    _pose_origin(el, joint.origin)
    etree.SubElement(el, "parent", link=model.link(joint.parent).name)
    etree.SubElement(el, "child", link=model.link(joint.child).name)
    if joint.type is not JointType.FIXED:
        axis = np.asarray(joint.axis, dtype=np.float64)
        axis = axis / np.linalg.norm(axis)
        etree.SubElement(el, "axis", xyz=_vec(axis.tolist()))
    lim = joint.limits
    if joint.type in (JointType.REVOLUTE, JointType.PRISMATIC):
        if lim is None or lim.effort is None or lim.velocity is None:
            raise UrdfExportError(
                f"joint {joint.name!r}: URDF requires lower, upper, effort and velocity "
                f"limits on a {joint.type} joint"
            )
        etree.SubElement(
            el,
            "limit",
            lower=_num(lim.lower),
            upper=_num(lim.upper),
            effort=_num(lim.effort),
            velocity=_num(lim.velocity),
        )
    elif joint.type is JointType.CONTINUOUS and lim is not None:
        attrs: dict[str, str] = {}
        if lim.effort is not None:
            attrs["effort"] = _num(lim.effort)
        if lim.velocity is not None:
            attrs["velocity"] = _num(lim.velocity)
        if attrs:
            etree.SubElement(el, "limit", attrs)
    if joint.dynamics is not None and joint.type is not JointType.FIXED:
        etree.SubElement(
            el,
            "dynamics",
            damping=_num(joint.dynamics.damping),
            friction=_num(joint.dynamics.friction),
        )
    return el


def export_urdf(
    model: RobotModel,
    masses: Mapping[UUID, MassProperties],
    visuals: Mapping[UUID, Sequence[MeshRef]] | None = None,
    collisions: Mapping[UUID, Sequence[MeshRef]] | None = None,
) -> UrdfExport:
    """Write ``model`` as ``<model.name>.urdf``.

    Raises :class:`UrdfExportError` if validation finds errors, a link has no mass
    properties, or a revolute or prismatic joint lacks effort or velocity limits.
    """
    report = validate(model)
    if not report.ok:
        problems = "; ".join(f"{i.code}: {i.message}" for i in report.errors)
        raise UrdfExportError(f"model is not valid: {problems}")
    visuals = visuals or {}
    collisions = collisions or {}

    root = etree.Element("robot", name=model.name)
    for index, link in enumerate(model.links):
        props = masses.get(link.id)
        if props is None:
            raise UrdfExportError(f"link {link.name!r}: no mass properties supplied")
        root.append(
            _link(link, index, props, visuals.get(link.id, ()), collisions.get(link.id, ()))
        )
    for joint in model.joints:
        root.append(_joint(model, joint))

    warnings = tuple(
        f"loop constraint {c.name!r} is not expressible in URDF and was dropped"
        for c in model.loop_constraints
    )
    data = etree.tostring(root, pretty_print=True, xml_declaration=True, encoding="UTF-8")
    return UrdfExport({f"{model.name}.urdf": data}, warnings)
