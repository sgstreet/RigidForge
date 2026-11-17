"""Layering: headless packages must work with every GUI/viewport package blocked (M1 task 13)."""

import ast
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

import rigidforge

HEADLESS = ("core", "importers", "physics", "exporters", "testing")
BLOCKED = ("PySide6", "shiboken6", "pyvista", "pyvistaqt", "vtk", "vtkmodules", "PyQt5", "PyQt6")
SRC = Path(rigidforge.__file__).parent

# Runs in a fresh interpreter so modules already imported by pytest cannot hide a violation.
_PROBE = textwrap.dedent(
    """
    import importlib, importlib.abc, pkgutil, sys

    BLOCKED = {blocked!r}

    class Blocker(importlib.abc.MetaPathFinder):
        def find_spec(self, name, path=None, target=None):
            if name.split(".")[0] in BLOCKED:
                raise ImportError("blocked import: " + name)
            return None

    sys.meta_path.insert(0, Blocker())
    import rigidforge
    for pkg_name in {packages!r}:
        pkg = importlib.import_module("rigidforge." + pkg_name)
        for info in pkgutil.walk_packages(pkg.__path__, pkg.__name__ + "."):
            importlib.import_module(info.name)
    importlib.import_module("rigidforge.pipeline")
    importlib.import_module("rigidforge.cli")
    leaked = sorted(m for m in sys.modules if m.split(".")[0] in BLOCKED)
    assert not leaked, leaked
    print("ok")
    """
)


def test_headless_packages_import_with_gui_packages_blocked() -> None:
    code = _PROBE.format(blocked=set(BLOCKED), packages=HEADLESS)
    result = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "ok"


def _imported_roots(path: Path) -> set[str]:
    roots: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            roots.add(node.module.split(".")[0])
    return roots


@pytest.mark.parametrize("package", HEADLESS)
def test_no_source_file_names_a_gui_package(package: str) -> None:
    """Static backstop to the ruff banned-api rule, including imports inside functions."""
    files = sorted((SRC / package).rglob("*.py"))
    assert files
    for path in files:
        assert not _imported_roots(path) & set(BLOCKED), path


def test_top_level_modules_are_headless() -> None:
    for name in ("pipeline.py", "cli.py", "__init__.py"):
        assert not _imported_roots(SRC / name) & set(BLOCKED), name


def test_no_subpackage_named_io() -> None:
    assert not [p for p in SRC.rglob("io") if p.is_dir()]
    assert not list(SRC.rglob("io.py"))
