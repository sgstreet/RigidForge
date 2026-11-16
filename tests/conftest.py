"""Shared pytest setup: skip environment-dependent tests cleanly instead of failing."""

import importlib.util
import os
import shutil

import pytest


def missing_requirements() -> dict[str, str | None]:
    """Map each environment marker to a skip reason, or None when its requirement is present."""
    return {
        "gui": None if importlib.util.find_spec("PySide6") else "PySide6 is not installed",
        "gazebo": None if shutil.which("gz") else "the gz CLI is not on PATH",
        "isaac": None
        if os.environ.get("ISAAC_PATH") or importlib.util.find_spec("isaacsim")
        else "Isaac Sim is not available (set ISAAC_PATH)",
    }


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    reasons = missing_requirements()
    for item in items:
        for marker, reason in reasons.items():
            if reason is not None and marker in item.keywords:
                item.add_marker(pytest.mark.skip(reason=reason))
