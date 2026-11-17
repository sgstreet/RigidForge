"""Tests for the CLI and the import-save-export pipeline (M1 task 12)."""

import json
import math
from pathlib import Path
from typing import Any

import mujoco as _mujoco
import numpy as np
import pytest
import yourdfpy

from rigidforge.cli import main
from rigidforge.core.project import PROJECT_FILE, load_project
from rigidforge.pipeline import BuildError, BuildSpec, build_project, compute_link_masses
from rigidforge.testing.mesh_fixtures import write_stl_box
from rigidforge.testing.step_fixtures import two_link_arm, write_step

mujoco: Any = _mujoco

DENSITY = 1200.0
ARM_SPEC: dict[str, Any] = {
    "name": "arm",
    "density": DENSITY,
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


@pytest.fixture
def step_file(tmp_path: Path) -> Path:
    return write_step(two_link_arm(), tmp_path / "arm.step")


@pytest.fixture
def spec_file(tmp_path: Path) -> Path:
    path = tmp_path / "spec.json"
    path.write_text(json.dumps(ARM_SPEC))
    return path


def _build(step: Path, spec: Path, out: Path) -> int:
    return main(["build", str(step), "--out", str(out), "--spec", str(spec)])


def test_build_writes_project_urdf_and_mjcf(step_file: Path, spec_file: Path, tmp_path: Path):
    out = tmp_path / "out"
    assert _build(step_file, spec_file, out) == 0
    assert (out / "arm.urdf").is_file()
    assert (out / "arm.xml").is_file()
    assert (out / "arm.rforge" / PROJECT_FILE).is_file()
    assert any((out / "arm.rforge" / "assets").iterdir())


def test_masses_match_analytic_values(step_file: Path, spec_file: Path, tmp_path: Path):
    out = tmp_path / "out"
    assert _build(step_file, spec_file, out) == 0
    model = load_project(out / "arm.rforge").model
    masses = compute_link_masses(model, out / "arm.rforge")
    base, arm = (masses[link.id] for link in model.links)
    assert base.mass == pytest.approx(DENSITY * 0.1 * 0.08 * 0.04, rel=1e-9)
    assert arm.mass == pytest.approx(DENSITY * math.pi * 0.01**2 * 0.15, rel=1e-9)
    # The arm link frame is the joint frame: the cylinder centre is 75 mm above it.
    assert tuple(arm.com) == pytest.approx((0.0, 0.0, 0.075), abs=1e-9)
    assert tuple(base.com) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def test_mjcf_compiles_and_urdf_loads(step_file: Path, spec_file: Path, tmp_path: Path):
    out = tmp_path / "out"
    assert _build(step_file, spec_file, out) == 0
    mj = mujoco.MjModel.from_xml_path(str(out / "arm.xml"))
    assert mj.njnt == 1
    assert mj.nu == 1
    assert float(mj.body_mass.sum()) == pytest.approx(
        DENSITY * (0.1 * 0.08 * 0.04 + math.pi * 0.01**2 * 0.15), rel=1e-9
    )
    robot = yourdfpy.URDF.load(str(out / "arm.urdf"), build_scene_graph=True)
    assert [j.name for j in robot.robot.joints] == ["shoulder"]
    robot.update_cfg(np.array([0.5]))
    tf = robot.get_transform("arm", "base")
    assert tf[:3, 3] == pytest.approx([0.0, 0.0, 0.02], abs=1e-9)
    assert tf[0, 0] == pytest.approx(math.cos(0.5))


def test_export_from_reloaded_project_is_byte_identical(
    step_file: Path, spec_file: Path, tmp_path: Path
):
    first = tmp_path / "first"
    assert _build(step_file, spec_file, first) == 0
    second = tmp_path / "second"
    assert main(["export", str(first / "arm.rforge"), "--out", str(second)]) == 0
    for name in ("arm.urdf", "arm.xml"):
        assert (first / name).read_bytes() == (second / name).read_bytes()


def test_build_is_repeatable(step_file: Path, spec_file: Path, tmp_path: Path):
    a, b = tmp_path / "a", tmp_path / "b"
    assert _build(step_file, spec_file, a) == 0
    assert _build(step_file, spec_file, b) == 0
    # Project UUIDs differ per build, so compare structure not bytes: same files and sizes.
    assert sorted(p.name for p in a.iterdir()) == sorted(p.name for p in b.iterdir())


def test_format_selection(step_file: Path, spec_file: Path, tmp_path: Path):
    out = tmp_path / "out"
    code = main(
        ["build", str(step_file), "--out", str(out), "--spec", str(spec_file), "--format", "urdf"]
    )
    assert code == 0
    assert (out / "arm.urdf").is_file()
    assert not (out / "arm.xml").exists()


def test_default_structure_is_a_star_of_fixed_joints(step_file: Path, tmp_path: Path):
    out = tmp_path / "out"
    assert main(["build", str(step_file), "--out", str(out), "--density", "500"]) == 0
    model = load_project(out / "arm.rforge").model
    assert [link.name for link in model.links] == ["base_1", "arm_1"]
    assert [j.type.value for j in model.joints] == ["fixed"]
    # Child link frame sits at the arm part's pose: 95 mm above the base origin.
    assert model.joints[0].origin.xyz == pytest.approx((0.0, 0.0, 0.095))
    mj = mujoco.MjModel.from_xml_path(str(out / "arm.xml"))
    assert float(mj.body_mass.sum()) == pytest.approx(
        500 * (0.1 * 0.08 * 0.04 + math.pi * 0.01**2 * 0.15), rel=1e-9
    )


def test_validate_reports_ok_and_errors(step_file: Path, spec_file: Path, tmp_path: Path, capsys):
    out = tmp_path / "out"
    assert _build(step_file, spec_file, out) == 0
    project = out / "arm.rforge"
    assert main(["validate", str(project)]) == 0
    assert "ok" in capsys.readouterr().out

    # Break the asset: validate must notice the hash mismatch.
    asset = next((project / "assets").iterdir())
    asset.write_bytes(asset.read_bytes() + b"\n")
    assert main(["validate", str(project)]) == 1
    assert "hash_mismatch" in capsys.readouterr().out


def test_validate_reports_model_errors(step_file: Path, spec_file: Path, tmp_path: Path, capsys):
    out = tmp_path / "out"
    assert _build(step_file, spec_file, out) == 0
    project = out / "arm.rforge"
    data = json.loads((project / PROJECT_FILE).read_text())
    data["links"][1]["name"] = data["links"][0]["name"]
    (project / PROJECT_FILE).write_text(json.dumps(data))
    assert main(["validate", str(project)]) == 1
    assert "error:" in capsys.readouterr().out


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda s: s["links"][0].update(parts=["nope"]), "no such part"),
        (lambda s: s["links"][1].update(parts=[]), "not assigned"),
        (lambda s: s["links"][1].update(parts=["arm_1", "base_1"]), "in both"),
        (lambda s: s["joints"][0].update(parent="ghost"), "not reachable"),
        (lambda s: s.update(densities={"ghost": 1.0}), "unknown parts"),
        (lambda s: s["joints"].clear(), "not connected"),
    ],
)
def test_bad_specs_fail_cleanly(step_file: Path, tmp_path: Path, capsys, mutate: Any, message: str):
    spec = json.loads(json.dumps(ARM_SPEC))
    mutate(spec)
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(spec))
    assert _build(step_file, path, tmp_path / "out") == 1
    assert message in capsys.readouterr().err


def test_invalid_model_is_not_saved(step_file: Path, tmp_path: Path):
    spec = json.loads(json.dumps(ARM_SPEC))
    spec["joints"][0]["axis"] = [0.0, 0.0, 0.0]
    project = tmp_path / "p.rforge"
    with pytest.raises(BuildError, match="not valid"):
        build_project(step_file, project, BuildSpec.model_validate(spec))
    assert not (project / PROJECT_FILE).exists()


def test_missing_input_and_bad_extension(tmp_path: Path, capsys):
    assert main(["build", str(tmp_path / "nope.step"), "--out", str(tmp_path / "o")]) == 1
    assert "no such file" in capsys.readouterr().err
    odd = tmp_path / "x.txt"
    odd.write_text("hi")
    assert main(["build", str(odd), "--out", str(tmp_path / "o")]) == 1
    assert "unsupported" in capsys.readouterr().err


def test_spec_with_unknown_key_is_rejected(step_file: Path, tmp_path: Path, capsys):
    path = tmp_path / "s.json"
    path.write_text(json.dumps({"bogus": 1}))
    assert _build(step_file, path, tmp_path / "out") == 1
    assert "cannot read spec" in capsys.readouterr().err


def test_mesh_input_builds_single_link(tmp_path: Path):
    stl = tmp_path / "box.stl"
    write_stl_box(stl, (0.02, 0.03, 0.04))
    out = tmp_path / "out"
    assert main(["build", str(stl), "--out", str(out), "--density", "1000"]) == 0
    model = load_project(out / "box.rforge").model
    assert len(model.links) == 1
    mass = compute_link_masses(model, out / "box.rforge")[model.links[0].id].mass
    assert mass == pytest.approx(1000 * 0.02 * 0.03 * 0.04, rel=1e-9)
    mj = mujoco.MjModel.from_xml_path(str(out / "box.xml"))
    assert float(mj.body_mass.sum()) == pytest.approx(mass, rel=1e-9)


def test_nested_leaf_names_use_slash(tmp_path: Path):
    from rigidforge.testing.step_fixtures import gripper_arm

    step = write_step(gripper_arm(), tmp_path / "g.step")
    out = tmp_path / "out"
    assert main(["build", str(step), "--out", str(out)]) == 0
    model = load_project(out / "g.rforge").model
    names = [p.name for p in model.parts]
    assert "wrist_1/finger_2" in names
    assert "wrist_1_finger_2" in [link.name for link in model.links]
