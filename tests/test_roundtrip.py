"""End-to-end round trip and determinism (M1 task 13)."""

import hashlib
from pathlib import Path

import pytest

from rigidforge.core.project import PROJECT_FILE, dumps_model, load_project
from rigidforge.pipeline import BuildSpec, build_project, export_project
from rigidforge.testing.mesh_fixtures import write_stl_box
from rigidforge.testing.step_fixtures import gripper_arm, two_link_arm, write_step

SPEC = BuildSpec.model_validate(
    {
        "name": "arm",
        "density": 1200.0,
        "links": [
            {"name": "base", "parts": ["base_1"]},
            {"name": "arm", "parts": ["arm_1"]},
        ],
        "joints": [
            {
                "name": "shoulder",
                "type": "revolute",
                "parent": "base",
                "child": "arm",
                "origin": {"xyz": [0.0, 0.0, 0.02]},
                "axis": [0.0, 0.0, 1.0],
                "limits": {"lower": -1.5, "upper": 1.5, "effort": 5.0, "velocity": 3.0},
                "position_actuator": {"kp": 20.0, "kv": 1.0},
            }
        ],
    }
)


def _digests(directory: Path) -> dict[str, str]:
    return {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(directory.iterdir())
        if p.is_file()
    }


@pytest.fixture
def arm_step(tmp_path: Path) -> Path:
    return write_step(two_link_arm(), tmp_path / "arm.step")


def test_in_memory_model_and_reloaded_project_export_identically(arm_step: Path, tmp_path: Path):
    project = tmp_path / "arm.rforge"
    model = build_project(arm_step, project, SPEC)
    original = tmp_path / "original"
    export_project(model, project, original, ("urdf", "mjcf"))

    reloaded = load_project(project).model
    again = tmp_path / "again"
    export_project(reloaded, project, again, ("urdf", "mjcf"))

    assert _digests(original) == _digests(again)
    assert set(_digests(original)) == {"arm.urdf", "arm.xml"}


def test_save_load_save_is_byte_stable(arm_step: Path, tmp_path: Path):
    project = tmp_path / "arm.rforge"
    build_project(arm_step, project, SPEC)
    first = (project / PROJECT_FILE).read_bytes()
    loaded = load_project(project).model
    assert dumps_model(loaded) == first


def test_repeated_export_is_byte_identical(arm_step: Path, tmp_path: Path):
    project = tmp_path / "arm.rforge"
    model = build_project(arm_step, project, SPEC)
    runs = []
    for i in range(3):
        out = tmp_path / f"run{i}"
        export_project(model, project, out, ("urdf", "mjcf"))
        runs.append(_digests(out))
    assert runs[0] == runs[1] == runs[2]


def test_export_does_not_depend_on_the_output_location(arm_step: Path, tmp_path: Path):
    project = tmp_path / "arm.rforge"
    model = build_project(arm_step, project, SPEC)
    export_project(model, project, tmp_path / "x", ("urdf", "mjcf"))
    export_project(model, project, tmp_path / "deeply" / "nested" / "y", ("urdf", "mjcf"))
    assert _digests(tmp_path / "x") == _digests(tmp_path / "deeply" / "nested" / "y")


def test_nested_assembly_round_trip(tmp_path: Path):
    step = write_step(gripper_arm(), tmp_path / "g.step")
    project = tmp_path / "g.rforge"
    model = build_project(step, project)
    first = tmp_path / "first"
    export_project(model, project, first, ("urdf", "mjcf"))
    second = tmp_path / "second"
    export_project(load_project(project).model, project, second, ("urdf", "mjcf"))
    assert _digests(first) == _digests(second)


def test_mesh_input_round_trip(tmp_path: Path):
    stl = tmp_path / "box.stl"
    write_stl_box(stl)
    project = tmp_path / "box.rforge"
    model = build_project(stl, project, BuildSpec(density=1000.0))
    first = tmp_path / "first"
    export_project(model, project, first, ("urdf", "mjcf"))
    second = tmp_path / "second"
    export_project(load_project(project).model, project, second, ("urdf", "mjcf"))
    assert _digests(first) == _digests(second)
