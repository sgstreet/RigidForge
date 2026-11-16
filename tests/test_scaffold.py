"""Smoke tests for the project scaffold."""

import importlib

import pytest

HEADLESS_PACKAGES = [
    "rigidforge",
    "rigidforge.core",
    "rigidforge.importers",
    "rigidforge.physics",
    "rigidforge.exporters",
]


@pytest.mark.parametrize("name", HEADLESS_PACKAGES)
def test_headless_packages_import(name: str) -> None:
    importlib.import_module(name)


def test_ocp_available_without_vtk_pin() -> None:
    """OCP comes from cadquery-ocp-novtk; the STEP unit setting takes effect after Init_s()."""
    from OCP.Interface import Interface_Static
    from OCP.STEPControl import STEPControl_Controller

    STEPControl_Controller.Init_s()
    assert Interface_Static.SetCVal_s("xstep.cascade.unit", "M")
    assert Interface_Static.CVal_s("xstep.cascade.unit") == "M"


def test_environment_markers_registered(pytestconfig: pytest.Config) -> None:
    registered = {line.split(":")[0] for line in pytestconfig.getini("markers")}
    assert {"gui", "gazebo", "isaac"} <= registered


@pytest.mark.gazebo
def test_gazebo_marker_skips_without_gz() -> None:
    import shutil

    assert shutil.which("gz") is not None
