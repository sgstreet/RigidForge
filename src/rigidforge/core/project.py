"""Project persistence: the ``*.rforge/`` directory (design doc section 7).

Layout::

    myrobot.rforge/
      project.json   canonical model: sorted keys, indented, atomic save, one rotating backup
      assets/        copied source files, named ``<stem>-<sha256[:12]><suffix>``
      cache/         derived data (gitignored, never read here)
      workspace.json user-local UI state (gitignored, never read here)

Only inputs are stored. ``schema_version`` drives dict-to-dict migrations that run before
pydantic validation. A file newer than this code understands loads read-only when it still
validates, and is refused otherwise. Unknown extension keys round-trip.

Lock files, autosave recovery and outside-edit prompts belong to the GUI (milestone 2);
``LoadedProject.disk_hash`` gives them what they need.
"""

import hashlib
import json
import os
import re
import tempfile
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, cast

from pydantic import ValidationError

from rigidforge.core.model import SCHEMA_VERSION, PartSource, RobotModel

PROJECT_FILE = "project.json"
BACKUP_FILE = "project.json.bak"
ASSETS_DIR = "assets"

type Migration = Callable[[dict[str, Any]], dict[str, Any]]
"""Takes a project dict at version ``n`` and returns it at version ``n + 1``."""

MIGRATIONS: dict[int, Migration] = {}
"""Migrations keyed by the version they upgrade from. Empty while the schema is at 1."""


class ProjectError(Exception):
    """A project could not be read or written."""


class UnsupportedSchemaError(ProjectError):
    """The file is newer than this version understands and does not validate."""


@dataclass(frozen=True)
class LoadedProject:
    """A loaded project and what the loader learned about it."""

    model: RobotModel
    path: Path
    disk_hash: str
    """sha256 of ``project.json`` as read, to detect outside edits later."""
    read_only: bool = False
    """True for a file written by a newer schema: do not save over it."""
    warnings: list[str] = field(default_factory=list[str])
    migrated_from: int | None = None
    """The file's schema version when migrations ran, otherwise ``None``."""


# --- Canonical JSON ----------------------------------------------------------------------


def dumps_model(model: RobotModel) -> bytes:
    """Canonical bytes for a model: sorted keys, two-space indent, trailing newline, UTF-8.

    Non-finite floats are written as ``NaN``/``Infinity`` so that a model with bad values
    (which validation reports) still saves and reloads unchanged.
    """
    data = model.model_dump(mode="json")
    return (json.dumps(data, sort_keys=True, indent=2, ensure_ascii=False) + "\n").encode()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


# --- Save --------------------------------------------------------------------------------


def _fsync_dir(directory: Path) -> None:
    try:
        fd = os.open(directory, os.O_RDONLY)
    except OSError:  # not supported on this platform
        return
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def atomic_write_bytes(path: Path, data: bytes) -> None:
    """Write ``data`` to ``path`` via a temp file in the same directory, fsync, then rename."""
    fd, tmp_name = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp")
    tmp = Path(tmp_name)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
    except BaseException:
        tmp.unlink(missing_ok=True)
        raise
    _fsync_dir(path.parent)


def save_project(model: RobotModel, path: Path) -> str:
    """Save ``model`` to the project directory ``path``, creating it if needed.

    The previous ``project.json`` is kept as ``project.json.bak`` (one rotating backup).
    Returns the sha256 of the written file. Bytes are identical for equal models.
    """
    path.mkdir(parents=True, exist_ok=True)
    target = path / PROJECT_FILE
    data = dumps_model(model)
    if target.exists():
        atomic_write_bytes(path / BACKUP_FILE, target.read_bytes())
    atomic_write_bytes(target, data)
    return sha256_bytes(data)


# --- Load --------------------------------------------------------------------------------


def migrate(
    data: dict[str, Any], migrations: Mapping[int, Migration] | None = None
) -> tuple[dict[str, Any], int | None]:
    """Run the migration chain up to ``SCHEMA_VERSION``.

    Returns the upgraded dict and the original version if anything ran. A file at the
    current or a newer version is returned as is. Raises ``ProjectError`` for a missing
    or invalid version, or a gap in the chain.
    """
    steps = MIGRATIONS if migrations is None else migrations
    version = data.get("schema_version")
    if not isinstance(version, int) or isinstance(version, bool) or version < 1:
        raise ProjectError(f"invalid schema_version: {version!r}")
    original = version
    while version < SCHEMA_VERSION:
        step = steps.get(version)
        if step is None:
            raise ProjectError(f"no migration from schema version {version}")
        data = step(data)
        version += 1
        data["schema_version"] = version
    return data, (original if original < SCHEMA_VERSION else None)


def load_project(path: Path, migrations: Mapping[int, Migration] | None = None) -> LoadedProject:
    """Load the project directory ``path``.

    Raises ``ProjectError`` when the file is missing, is not JSON, or does not validate, and
    ``UnsupportedSchemaError`` when it is newer than this version and does not validate.
    """
    source = path / PROJECT_FILE
    try:
        raw = source.read_bytes()
    except OSError as exc:
        raise ProjectError(f"cannot read {source}: {exc}") from exc
    try:
        data = json.loads(raw)
    except ValueError as exc:
        raise ProjectError(f"{source} is not valid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ProjectError(f"{source} does not hold a JSON object")
    data = cast("dict[str, Any]", data)

    data, migrated_from = migrate(data, migrations)
    version: int = data["schema_version"]
    newer = version > SCHEMA_VERSION
    try:
        model = RobotModel.model_validate(data)
    except ValidationError as exc:
        if newer:
            raise UnsupportedSchemaError(
                f"{source} uses schema version {version}, newer than {SCHEMA_VERSION}, "
                f"and cannot be read: {exc.error_count()} error(s)"
            ) from exc
        raise ProjectError(f"{source} is invalid: {exc}") from exc

    warnings: list[str] = []
    if newer:
        warnings.append(
            f"schema version {version} is newer than {SCHEMA_VERSION}: opened read-only"
        )
    return LoadedProject(
        model=model,
        path=path,
        disk_hash=sha256_bytes(raw),
        read_only=newer,
        warnings=warnings,
        migrated_from=migrated_from,
    )


# --- Assets ------------------------------------------------------------------------------

_UNSAFE = re.compile(r"[^A-Za-z0-9._-]+")
_HASH_CHARS = 12


def asset_name(source: Path, sha256: str) -> str:
    """The name a file gets in ``assets/``: sanitised stem, short hash, original suffix."""
    stem = _UNSAFE.sub("_", source.stem).strip("._") or "asset"
    return f"{stem}-{sha256[:_HASH_CHARS]}{source.suffix.lower()}"


def add_asset(project: Path, source: Path) -> tuple[str, str]:
    """Copy ``source`` into the project's ``assets/``, deduplicated by content.

    Returns ``(relative name, sha256)``; the name is what ``PartSource.asset`` stores. Two
    imports of the same bytes share one file, even under different original names. The
    copy is atomic.
    """
    digest = sha256_file(source)
    name = asset_name(source, digest)
    assets = project / ASSETS_DIR
    assets.mkdir(parents=True, exist_ok=True)
    target = assets / name
    if target.exists() and sha256_file(target) == digest:
        return name, digest
    # Reached when the asset is new, or the stored copy is damaged: (re)write it.
    atomic_write_bytes(target, source.read_bytes())
    return name, digest


def asset_path(project: Path, name: str) -> Path:
    """Resolve an asset name to a file inside ``assets/``.

    Raises ``ProjectError`` for a name that would escape the directory.
    """
    assets = (project / ASSETS_DIR).resolve()
    resolved = (assets / name).resolve()
    if not resolved.is_relative_to(assets) or resolved == assets:
        raise ProjectError(f"asset path escapes assets/: {name!r}")
    return resolved


@dataclass(frozen=True)
class AssetProblem:
    part_name: str
    asset: str
    problem: str
    """``"missing"``, ``"hash_mismatch"`` or ``"unsafe_path"``."""


def check_assets(model: RobotModel, project: Path) -> list[AssetProblem]:
    """Report parts whose asset file is missing, altered, or has an unsafe path."""
    problems: list[AssetProblem] = []
    seen: dict[tuple[str, str], str | None] = {}
    for part in model.parts:
        src: PartSource = part.source
        key = (src.asset, src.sha256)
        if key not in seen:
            seen[key] = _asset_problem(project, src)
        problem = seen[key]
        if problem is not None:
            problems.append(AssetProblem(part.name, src.asset, problem))
    return problems


def _asset_problem(project: Path, src: PartSource) -> str | None:
    try:
        file = asset_path(project, src.asset)
    except ProjectError:
        return "unsafe_path"
    if not file.is_file():
        return "missing"
    if sha256_file(file) != src.sha256:
        return "hash_mismatch"
    return None


def remove_unused_assets(model: RobotModel, project: Path) -> list[str]:
    """Delete asset files no part refers to, and return their names.

    Callers decide when to run this (the GUI keeps old copies until save, for undo).
    """
    used = {p.source.asset for p in model.parts}
    removed: list[str] = []
    assets = project / ASSETS_DIR
    if not assets.is_dir():
        return removed
    for file in sorted(assets.iterdir()):
        if file.is_file() and file.name not in used:
            file.unlink()
            removed.append(file.name)
    return removed
