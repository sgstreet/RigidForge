"""Exact B-Rep mass properties with OCCT ``GProp`` (M1 task 7).

``brep_unit_mass_properties`` integrates over the exact surfaces of a ``TopoDS_Shape`` that
is already in metres (the STEP importer scales at the boundary), giving volume, CoM and the
inertia tensor about the CoM at unit density. ``rigidforge.physics.mass`` then applies the
part's mass source.

Integration uses OCCT's adaptive Gauss scheme with a relative tolerance of
:data:`GPROP_EPS`. The default fixed-order scheme is exact on analytic primitives but was off
by up to 1e-5 relative on the hexapod frames (B-spline surfaces). The tolerance bounds the
volume error; the inertia converges more slowly, so a tight value is used: at 1e-12 the
frames' tensors agree with a 1e-14 reference to about 1e-11 (4e-9 on the worst small part),
for about 20 ms per frame against 3 ms for the default scheme.

Only faces in closed shells are counted, so a surface model or a solid with a gap gives no
volume and is rejected rather than producing a meaningless tensor. A solid with reversed
orientation (negative volume) is rejected too.
"""

from typing import Any

import numpy as np

from rigidforge.physics.mass import InertiaMethod, MassPropertiesError, UnitMassProperties

__all__ = ["GPROP_EPS", "MIN_VOLUME", "brep_unit_mass_properties"]

GPROP_EPS = 1e-12
"""Relative error target per face for the adaptive volume integration."""

MIN_VOLUME = 1e-15
"""Smallest accepted volume in m³ (a 10 µm cube). Anything smaller is treated as empty."""


def brep_unit_mass_properties(shape: Any, name: str = "shape") -> UnitMassProperties:
    """Unit-density volume, CoM and CoM inertia of ``shape`` (metres) in its own frame.

    ``shape`` is an OCP ``TopoDS_Shape``; its location, if any, is honoured, so the result is
    in the frame the shape is expressed in. ``name`` is used in error messages. Raises
    :class:`MassPropertiesError` for a null shape, a shape without solids, a volume that is
    not positive, or non-finite results.
    """
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopExp import TopExp_Explorer

    if shape is None or shape.IsNull():
        raise MassPropertiesError(f"{name}: null shape")
    if not TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_SOLID).More():
        raise MassPropertiesError(f"{name}: no solids (a surface or wire model has no volume)")

    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props, GPROP_EPS, True, False)
    volume = float(props.Mass())
    if not np.isfinite(volume):
        raise MassPropertiesError(f"{name}: volume is not finite")
    if volume < 0.0:
        raise MassPropertiesError(f"{name}: negative volume {volume} m³ (reversed solid)")
    if volume < MIN_VOLUME:
        raise MassPropertiesError(
            f"{name}: volume {volume} m³ is ~0 (open shells, or geometry not in metres)"
        )

    c = props.CentreOfMass()
    m = props.MatrixOfInertia()  # about the CoM
    com = np.array([c.X(), c.Y(), c.Z()], dtype=np.float64)
    inertia = np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)], dtype=np.float64)
    if not (np.all(np.isfinite(com)) and np.all(np.isfinite(inertia))):
        raise MassPropertiesError(f"{name}: CoM or inertia is not finite")
    return UnitMassProperties(volume, com, 0.5 * (inertia + inertia.T), InertiaMethod.BREP_EXACT)
