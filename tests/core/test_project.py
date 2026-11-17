"""Tests for project persistence (task 9)."""

import json
import os
from pathlib import Path
from typing import Any
from uuid import UUID

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from rigidforge.core import (
    Density,
    Extensions,
    InertialOverride,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    MeasuredMass,
    MirrorPlane,
    MjcfExtension,
    MjcfPositionActuator,
    Part,
    PartSource,
    Pose,
    RobotModel,
    SourceKind,
)
from rigidforge.core import project as proj
from rigidforge.core.project import (
    ProjectError,
    UnsupportedSchemaError,
    add_asset,
    asset_path,
    check_assets,
    dumps_model,
    load_project,
    remove_unused_assets,
    save_project,
)

GOLDEN = Path(__file__).parent.parent / "data" / "golden" / "project-v1"


def _uuid(n: int) -> UUID:
    return UUID(int=n)


def golden_model() -> RobotModel:
    """A fixed model (fixed IDs) whose saved form is the version 1 golden fixture."""
    base = Link(id=_uuid(1), name="base")
    arm = Link(
        id=_uuid(2),
        name="arm",
        inertial_override=InertialOverride(
            mass=0.018,
            com=(-0.007, 0.0, 0.0),
            inertia=InertiaTensor(ixx=1.2e-6, iyy=2.3e-6, izz=2.1e-6),
            source="datasheet",
        ),
    )
    source = PartSource(
        kind=SourceKind.STEP,
        asset="frame-0123456789ab.step",
        sha256="0123456789ab" * 5 + "0123",
        scale_to_si=0.001,
        assembly_path=["frame"],
        original_path="/tmp/frame.step",
    )
    parts = [
        Part(id=_uuid(10), name="frame", source=source, link=base.id, mass=Density(density=1270.0)),
        Part(
            id=_uuid(11),
            name="frame-mirror",
            source=source,
            link=arm.id,
            placement=Pose(xyz=(0.0, 0.0, 0.05)),
            mirror=MirrorPlane(normal=(0.0, 1.0, 0.0)),
            mass=MeasuredMass(mass=0.012),
        ),
    ]
    joint = Joint(
        id=_uuid(20),
        name="shoulder",
        type=JointType.REVOLUTE,
        parent=base.id,
        child=arm.id,
        origin=Pose(xyz=(0.0, 0.0, 0.1)),
        axis=(0.0, 1.0, 0.0),
        limits=JointLimits(lower=-1.5, upper=1.5, effort=0.6, velocity=8.0),
        extensions=Extensions(
            mjcf=MjcfExtension(position_actuator=MjcfPositionActuator(kp=5.0)),
        ),
    )
    model = RobotModel(name="golden", links=[base, arm], joints=[joint], parts=parts)
    model.extensions = Extensions.model_validate({"isaac": {"solver": {"iterations": 50}}})
    return model


def two_assets(tmp_path: Path) -> tuple[Path, Path]:
    a = tmp_path / "src" / "Frame (v2).step"
    a.parent.mkdir()
    a.write_bytes(b"ISO-10303-21; frame\n")
    b = tmp_path / "src" / "copy-of-frame.step"
    b.write_bytes(a.read_bytes())
    return a, b


# --- Save and load -----------------------------------------------------------------------


def test_canonical_form(tmp_path: Path) -> None:
    save_project(golden_model(), tmp_path / "p.rforge")
    text = (tmp_path / "p.rforge" / "project.json").read_text()
    assert text.endswith("}\n")
    data = json.loads(text)
    assert text == json.dumps(data, sort_keys=True, indent=2, ensure_ascii=False) + "\n"
    assert list(data) == sorted(data)


def test_save_load_equal(tmp_path: Path) -> None:
    model = golden_model()
    digest = save_project(model, tmp_path / "p.rforge")
    loaded = load_project(tmp_path / "p.rforge")
    assert loaded.model == model
    assert loaded.disk_hash == digest
    assert not loaded.read_only and loaded.warnings == [] and loaded.migrated_from is None


def test_reloaded_project_saves_identical_bytes(tmp_path: Path) -> None:
    save_project(golden_model(), tmp_path / "a.rforge")
    loaded = load_project(tmp_path / "a.rforge")
    save_project(loaded.model, tmp_path / "b.rforge")
    assert (tmp_path / "a.rforge" / "project.json").read_bytes() == (
        tmp_path / "b.rforge" / "project.json"
    ).read_bytes()


def test_unknown_extension_keys_survive_save_load(tmp_path: Path) -> None:
    save_project(golden_model(), tmp_path / "p.rforge")
    loaded = load_project(tmp_path / "p.rforge")
    data = json.loads(dumps_model(loaded.model))
    assert data["extensions"]["isaac"] == {"solver": {"iterations": 50}}


def test_non_finite_values_round_trip(tmp_path: Path) -> None:
    model = golden_model()
    model.joints[0].limits = JointLimits(lower=float("nan"), upper=float("inf"))
    save_project(model, tmp_path / "p.rforge")
    limits = load_project(tmp_path / "p.rforge").model.joints[0].limits
    assert limits is not None and limits.lower != limits.lower and limits.upper == float("inf")


def test_backup_is_previous_version(tmp_path: Path) -> None:
    project = tmp_path / "p.rforge"
    model = golden_model()
    save_project(model, project)
    first = (project / "project.json").read_bytes()
    model.name = "renamed"
    save_project(model, project)
    assert (project / "project.json.bak").read_bytes() == first
    assert load_project(project).model.name == "renamed"


def test_no_temp_files_left(tmp_path: Path) -> None:
    project = tmp_path / "p.rforge"
    save_project(golden_model(), project)
    save_project(golden_model(), project)
    assert sorted(f.name for f in project.iterdir()) == ["project.json", "project.json.bak"]


def test_failed_save_keeps_old_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    project = tmp_path / "p.rforge"
    save_project(golden_model(), project)
    before = (project / "project.json").read_bytes()

    def boom(src: object, dst: object) -> None:
        raise OSError("disk full")

    model = golden_model()
    model.name = "changed"
    monkeypatch.setattr(os, "replace", boom)
    with pytest.raises(OSError, match="disk full"):
        save_project(model, project)
    monkeypatch.undo()
    assert (project / "project.json").read_bytes() == before
    assert not [f for f in project.iterdir() if f.name.endswith(".tmp")]


# --- Golden fixture and schema versions -------------------------------------------------


def test_golden_v1_loads_to_expected_model() -> None:
    loaded = load_project(GOLDEN)
    assert loaded.model == golden_model()
    assert loaded.migrated_from is None


def test_golden_v1_resaves_byte_identical(tmp_path: Path) -> None:
    loaded = load_project(GOLDEN)
    save_project(loaded.model, tmp_path / "out.rforge")
    assert (tmp_path / "out.rforge" / "project.json").read_bytes() == (
        GOLDEN / "project.json"
    ).read_bytes()


def _write(project: Path, data: dict[str, Any]) -> None:
    project.mkdir(parents=True, exist_ok=True)
    (project / "project.json").write_text(json.dumps(data))


def test_migration_chain_runs_before_validation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Pretend version 0-style data (an old field name) is migrated up to the current one."""
    monkeypatch.setattr(proj, "SCHEMA_VERSION", 3)

    def v1_to_v2(d: dict[str, Any]) -> dict[str, Any]:
        d["title"] = d.pop("robot_name")
        return d

    def v2_to_v3(d: dict[str, Any]) -> dict[str, Any]:
        d["name"] = d.pop("title")
        return d

    _write(tmp_path / "p", {"schema_version": 1, "robot_name": "old"})
    loaded = load_project(tmp_path / "p", {1: v1_to_v2, 2: v2_to_v3})
    assert loaded.model.name == "old"
    assert loaded.migrated_from == 1
    assert not loaded.read_only


def test_missing_migration_is_an_error(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(proj, "SCHEMA_VERSION", 2)
    _write(tmp_path / "p", {"schema_version": 1, "name": "x"})
    with pytest.raises(ProjectError, match="no migration from schema version 1"):
        load_project(tmp_path / "p", {})


def test_newer_file_opens_read_only_with_warning(tmp_path: Path) -> None:
    data = json.loads(dumps_model(golden_model()))
    data["schema_version"] = 99
    _write(tmp_path / "p", data)
    loaded = load_project(tmp_path / "p")
    assert loaded.read_only
    assert "newer" in loaded.warnings[0]


def test_newer_file_with_unknown_fields_is_refused(tmp_path: Path) -> None:
    data = json.loads(dumps_model(golden_model()))
    data["schema_version"] = 99
    data["links"][0]["future_field"] = 1
    _write(tmp_path / "p", data)
    with pytest.raises(UnsupportedSchemaError):
        load_project(tmp_path / "p")


@pytest.mark.parametrize("version", [0, -1, "1", None, True, 1.5])
def test_bad_schema_version(tmp_path: Path, version: object) -> None:
    _write(tmp_path / "p", {"schema_version": version, "name": "x"})
    with pytest.raises(ProjectError, match="schema_version"):
        load_project(tmp_path / "p")


def test_load_errors(tmp_path: Path) -> None:
    with pytest.raises(ProjectError, match="cannot read"):
        load_project(tmp_path / "nothing")
    (tmp_path / "bad").mkdir()
    (tmp_path / "bad" / "project.json").write_text("{not json")
    with pytest.raises(ProjectError, match="not valid JSON"):
        load_project(tmp_path / "bad")
    (tmp_path / "list").mkdir()
    (tmp_path / "list" / "project.json").write_text("[]")
    with pytest.raises(ProjectError, match="JSON object"):
        load_project(tmp_path / "list")
    _write(tmp_path / "wrong", {"schema_version": 1, "links": "nope", "name": "x"})
    with pytest.raises(ProjectError, match="invalid"):
        load_project(tmp_path / "wrong")


def test_bad_semantics_still_load(tmp_path: Path) -> None:
    """Validation, not loading, reports bad values (design doc section 7)."""
    model = golden_model()
    model.parts[0].mass = Density(density=-1.0)
    save_project(model, tmp_path / "p")
    assert load_project(tmp_path / "p").model == model


# --- Assets ------------------------------------------------------------------------------


def test_add_asset_names_by_content_hash(tmp_path: Path) -> None:
    a, _ = two_assets(tmp_path)
    project = tmp_path / "p.rforge"
    name, digest = add_asset(project, a)
    assert name == f"Frame_v2-{digest[:12]}.step"
    assert (project / "assets" / name).read_bytes() == a.read_bytes()


def test_same_content_is_deduplicated(tmp_path: Path) -> None:
    a, b = two_assets(tmp_path)
    project = tmp_path / "p.rforge"
    name_a, digest_a = add_asset(project, a)
    name_b, digest_b = add_asset(project, b)
    assert digest_a == digest_b
    assert name_a != name_b  # different original names keep their own readable stem
    assert add_asset(project, a) == (name_a, digest_a)  # re-import: same file, no copy


def test_damaged_asset_is_repaired_on_reimport(tmp_path: Path) -> None:
    a, _ = two_assets(tmp_path)
    project = tmp_path / "p.rforge"
    name, _ = add_asset(project, a)
    (project / "assets" / name).write_bytes(b"corrupt")
    add_asset(project, a)
    assert (project / "assets" / name).read_bytes() == a.read_bytes()


def test_asset_path_rejects_escape(tmp_path: Path) -> None:
    for bad in ["../project.json", "/etc/passwd", "a/../../x", "", "."]:
        with pytest.raises(ProjectError):
            asset_path(tmp_path, bad)
    assert asset_path(tmp_path, "ok.step") == (tmp_path / "assets" / "ok.step").resolve()


def test_check_assets(tmp_path: Path) -> None:
    a, _ = two_assets(tmp_path)
    project = tmp_path / "p.rforge"
    name, digest = add_asset(project, a)
    source = PartSource(kind=SourceKind.STEP, asset=name, sha256=digest)
    model = RobotModel(name="r", parts=[Part(name="p", source=source)])
    assert check_assets(model, project) == []

    (project / "assets" / name).write_bytes(b"changed")
    assert [p.problem for p in check_assets(model, project)] == ["hash_mismatch"]
    (project / "assets" / name).unlink()
    assert [p.problem for p in check_assets(model, project)] == ["missing"]

    evil = PartSource(kind=SourceKind.STEP, asset="../x.step", sha256="0")
    model.parts.append(Part(name="evil", source=evil))
    assert [p.problem for p in check_assets(model, project)] == ["missing", "unsafe_path"]


def test_remove_unused_assets(tmp_path: Path) -> None:
    a, b = two_assets(tmp_path)
    project = tmp_path / "p.rforge"
    used, digest = add_asset(project, a)
    other, _ = add_asset(project, b)
    model = RobotModel(
        name="r",
        parts=[Part(name="p", source=PartSource(kind=SourceKind.STEP, asset=used, sha256=digest))],
    )
    assert remove_unused_assets(model, project) == [other]
    assert [f.name for f in (project / "assets").iterdir()] == [used]
    assert remove_unused_assets(model, tmp_path / "none") == []


# --- Property test -----------------------------------------------------------------------

_floats = st.floats(allow_nan=False, allow_infinity=False, width=64)
_names = st.text(min_size=1, max_size=12)


@st.composite
def random_models(draw: st.DrawFn) -> RobotModel:
    count = draw(st.integers(min_value=1, max_value=6))
    links = [Link(id=_uuid(i + 1), name=draw(_names)) for i in range(count)]
    joints = [
        Joint(
            id=_uuid(100 + i),
            name=draw(_names),
            type=JointType.REVOLUTE,
            parent=links[draw(st.integers(0, i - 1))].id,
            child=links[i].id,
            origin=Pose(xyz=(draw(_floats), draw(_floats), draw(_floats))),
            limits=JointLimits(lower=draw(_floats), upper=draw(_floats)),
        )
        for i in range(1, count)
    ]
    source = PartSource(kind=SourceKind.MESH, asset="m.stl", sha256="ab", scale_to_si=draw(_floats))
    parts = [
        Part(id=_uuid(200 + i), name=draw(_names), source=source, link=link.id)
        for i, link in enumerate(links)
    ]
    extras = draw(st.dictionaries(st.text(min_size=1, max_size=5), st.integers(), max_size=3))
    model = RobotModel(name=draw(_names), links=links, joints=joints, parts=parts)
    model.extensions = Extensions.model_validate({"future": extras})
    return model


@settings(max_examples=40, deadline=None)
@given(random_models())
def test_property_save_load_equal(model: RobotModel) -> None:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        project = Path(tmp) / "p.rforge"
        save_project(model, project)
        first = (project / "project.json").read_bytes()
        loaded = load_project(project)
        assert loaded.model == model
        save_project(loaded.model, project)
        assert (project / "project.json").read_bytes() == first
