"""MJCF exporter: a pure function from a ``RobotModel`` to MJCF bytes (M1 task 11).

The MJCF is generated directly from the internal model (never derived from URDF). SI units
and radians throughout (``<compiler angle="radian">``). Inputs the model does not hold are
passed in, as for the URDF exporter: per-link mass properties and mesh references.

Mapping:

* each link is a ``<body>``; the root link sits in ``<worldbody>`` (fixed base);
* a joint's origin is the child body's ``pos``/``quat`` in its parent; its axis is the MJCF
  joint axis in the child body frame (the joint frame and child frame coincide);
* ``<inertial>`` uses ``pos`` = CoM and ``fullinertia`` about the CoM (MJCF order
  ``ixx iyy izz ixy ixz iyz``, unlike URDF);
* ``extensions.mjcf.position_actuator`` on a joint becomes a ``<position>`` actuator;
* loop constraints become ``<equality>`` ``connect``/``weld`` entries between sites.

MuJoCo rejects inertia that breaks the triangle inequality, so mass properties are checked
here and the export fails with a clear message instead.
"""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from uuid import UUID

import numpy as np
from lxml import etree
from lxml.etree import _Element as Element  # pyright: ignore[reportPrivateUsage]

from rigidforge.core.geometry import Pose
from rigidforge.core.inertia import InertiaProblem
from rigidforge.core.model import Joint, JointType, Link, LoopConstraintType, RobotModel
from rigidforge.core.validation import validate
from rigidforge.exporters.common import MeshRef, format_number
from rigidforge.physics.inertia import MassProperties

__all__ = ["MjcfExport", "MjcfExportError", "export_mjcf"]

_VISUAL_GROUP = "2"
_COLLISION_GROUP = "3"


class MjcfExportError(ValueError):
    """The model cannot be written as MJCF."""


@dataclass(frozen=True, slots=True)
class MjcfExport:
    """The result: file name to content, and non-fatal notes."""

    files: dict[str, bytes]
    warnings: tuple[str, ...] = ()


def _num(x: float) -> str:
    try:
        return format_number(x)
    except ValueError as exc:
        raise MjcfExportError(str(exc)) from exc


def _vec(v: Sequence[float]) -> str:
    return " ".join(_num(float(x)) for x in v)


def _pose_attrs(pose: Pose) -> dict[str, str]:
    """``pos`` and ``quat`` (scalar-first, as MJCF expects), omitting identity parts."""
    attrs: dict[str, str] = {}
    if any(pose.xyz):
        attrs["pos"] = _vec(pose.xyz)
    if tuple(pose.quat) != (1.0, 0.0, 0.0, 0.0):
        attrs["quat"] = _vec(pose.quat)
    return attrs


def _mesh_assets(
    root: Element,
    refs_by_link: Sequence[Mapping[UUID, Sequence[MeshRef]]],
    model: RobotModel,
) -> dict[str, str]:
    """Add one ``<mesh>`` asset per distinct URI (in model order) and return uri -> name."""
    names: dict[str, str] = {}
    for refs in refs_by_link:
        for link in model.links:
            for ref in refs.get(link.id, ()):
                if ref.uri not in names:
                    names[ref.uri] = f"mesh_{len(names)}"
    if names:
        asset = etree.SubElement(root, "asset")
        for uri, name in names.items():
            etree.SubElement(asset, "mesh", name=name, file=uri)
    return names


def _geoms(
    body: Element,
    refs: Sequence[MeshRef],
    names: Mapping[str, str],
    *,
    visual: bool,
) -> None:
    for ref in refs:
        attrs: dict[str, str] = {"type": "mesh", "mesh": names[ref.uri]}
        attrs.update(_pose_attrs(ref.origin))
        if visual:
            attrs.update(group=_VISUAL_GROUP, contype="0", conaffinity="0", mass="0")
            if ref.rgba is not None:
                attrs["rgba"] = _vec(ref.rgba)
        else:
            attrs.update(group=_COLLISION_GROUP, mass="0")
        etree.SubElement(body, "geom", attrs)


def _inertial(body: Element, props: MassProperties) -> None:
    i = props.inertia
    etree.SubElement(
        body,
        "inertial",
        pos=_vec(props.com_tuple),
        mass=_num(props.mass),
        fullinertia=_vec(
            [i[0, 0], i[1, 1], i[2, 2], i[0, 1], i[0, 2], i[1, 2]],
        ),
    )


def _joint(body: Element, joint: Joint) -> None:
    if joint.type is JointType.FIXED:
        return
    axis = np.asarray(joint.axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    attrs: dict[str, str] = {
        "name": joint.name,
        "type": "slide" if joint.type is JointType.PRISMATIC else "hinge",
        "axis": _vec(axis.tolist()),
    }
    lim = joint.limits
    if joint.type is not JointType.CONTINUOUS:
        if lim is None:
            raise MjcfExportError(f"joint {joint.name!r}: a {joint.type} joint needs limits")
        attrs["limited"] = "true"
        attrs["range"] = _vec((lim.lower, lim.upper))
    if lim is not None and lim.effort is not None:
        attrs["actuatorfrclimited"] = "true"
        attrs["actuatorfrcrange"] = _vec((-lim.effort, lim.effort))
    if joint.dynamics is not None:
        if joint.dynamics.damping:
            attrs["damping"] = _num(joint.dynamics.damping)
        if joint.dynamics.friction:
            attrs["frictionloss"] = _num(joint.dynamics.friction)
    etree.SubElement(body, "joint", attrs)


def _body(
    model: RobotModel,
    link: Link,
    joint: Joint | None,
    parent_el: Element,
    masses: Mapping[UUID, MassProperties],
    visuals: Mapping[UUID, Sequence[MeshRef]],
    collisions: Mapping[UUID, Sequence[MeshRef]],
    names: Mapping[str, str],
    sites: Mapping[UUID, list[tuple[str, Pose]]],
) -> None:
    props = masses.get(link.id)
    if props is None:
        raise MjcfExportError(f"link {link.name!r}: no mass properties supplied")
    problems = [p for p in props.problems() if p is not InertiaProblem.EXTREME_RATIO]
    if problems:
        raise MjcfExportError(
            f"link {link.name!r}: invalid mass properties ({', '.join(str(p) for p in problems)})"
        )
    attrs = {"name": link.name}
    if joint is not None:
        attrs.update(_pose_attrs(joint.origin))
    body = etree.SubElement(parent_el, "body", attrs)
    if joint is not None:
        _joint(body, joint)
    _inertial(body, props)
    _geoms(body, visuals.get(link.id, ()), names, visual=True)
    _geoms(body, collisions.get(link.id, ()), names, visual=False)
    for site_name, pose in sites.get(link.id, ()):
        etree.SubElement(body, "site", {"name": site_name, **_pose_attrs(pose)})
    for child_joint in model.child_joints(link.id):
        _body(
            model,
            model.link(child_joint.child),
            child_joint,
            body,
            masses,
            visuals,
            collisions,
            names,
            sites,
        )


def _actuators(root: Element, model: RobotModel) -> None:
    entries: list[Element] = []
    for joint in model.joints:
        mjcf = joint.extensions.mjcf
        if mjcf is None or mjcf.position_actuator is None:
            continue
        if joint.type is JointType.FIXED:
            raise MjcfExportError(f"joint {joint.name!r}: a fixed joint cannot have an actuator")
        act = mjcf.position_actuator
        attrs: dict[str, str] = {
            "name": f"{joint.name}_act",
            "joint": joint.name,
            "kp": _num(act.kp),
        }
        if act.kv is not None:
            attrs["kv"] = _num(act.kv)
        if act.forcerange is not None:
            attrs["forcelimited"] = "true"
            attrs["forcerange"] = _vec(act.forcerange)
        if act.ctrlrange is not None:
            attrs["ctrllimited"] = "true"
            attrs["ctrlrange"] = _vec(act.ctrlrange)
        entries.append(etree.Element("position", attrs))
    if entries:
        el = etree.SubElement(root, "actuator")
        el.extend(entries)


def export_mjcf(
    model: RobotModel,
    masses: Mapping[UUID, MassProperties],
    visuals: Mapping[UUID, Sequence[MeshRef]] | None = None,
    collisions: Mapping[UUID, Sequence[MeshRef]] | None = None,
) -> MjcfExport:
    """Write ``model`` as ``<model.name>.xml``.

    Mesh URIs are written relative to the XML file as given; MuJoCo needs the files to exist
    when it compiles. Raises :class:`MjcfExportError` if validation finds errors, a link has
    no or invalid mass properties, or a revolute or prismatic joint has no limits.
    """
    report = validate(model)
    if not report.ok:
        problems = "; ".join(f"{i.code}: {i.message}" for i in report.errors)
        raise MjcfExportError(f"model is not valid: {problems}")
    visuals = visuals or {}
    collisions = collisions or {}

    root = etree.Element("mujoco", model=model.name)
    etree.SubElement(root, "compiler", angle="radian", autolimits="false", inertiafromgeom="false")
    names = _mesh_assets(root, [visuals, collisions], model)

    # One site per loop-constraint end, named after the constraint.
    sites: dict[UUID, list[tuple[str, Pose]]] = {}
    for c in model.loop_constraints:
        sites.setdefault(c.link_a, []).append((f"{c.name}_a", c.frame_a))
        sites.setdefault(c.link_b, []).append((f"{c.name}_b", c.frame_b))

    child_ids = {j.child for j in model.joints}
    roots = [link for link in model.links if link.id not in child_ids]
    world = etree.SubElement(root, "worldbody")
    for link in roots:
        _body(model, link, None, world, masses, visuals, collisions, names, sites)

    if model.loop_constraints:
        eq = etree.SubElement(root, "equality")
        for c in model.loop_constraints:
            tag = "connect" if c.type is LoopConstraintType.CONNECT else "weld"
            etree.SubElement(eq, tag, name=c.name, site1=f"{c.name}_a", site2=f"{c.name}_b")
    _actuators(root, model)

    data = etree.tostring(root, pretty_print=True, xml_declaration=False, encoding="UTF-8")
    return MjcfExport({f"{model.name}.xml": data})
