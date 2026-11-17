"""Command line interface: ``rforge build``, ``rforge export`` and ``rforge validate`` (M1 task 12).

Exit codes: 0 success, 1 the model or input has errors, 2 bad usage (argparse).
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Literal

from rigidforge.core.project import ProjectError, check_assets, load_project
from rigidforge.core.validation import Severity, validate
from rigidforge.pipeline import (
    DEFAULT_DENSITY,
    BuildError,
    BuildSpec,
    build_project,
    compute_link_masses,
    export_project,
    load_spec,
)

Format = Literal["urdf", "mjcf"]
_FORMATS: tuple[Format, ...] = ("urdf", "mjcf")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="rforge", description="RigidForge headless tools.")
    sub = parser.add_subparsers(dest="command", required=True)

    build = sub.add_parser("build", help="import a STEP or mesh file, save a project, export")
    build.add_argument("input", type=Path, help="STEP (.step/.stp) or mesh (.stl/.obj/.dae/.3mf)")
    build.add_argument("--out", type=Path, required=True, help="output directory")
    build.add_argument("--spec", type=Path, help="JSON build spec (links, joints, densities)")
    build.add_argument(
        "--project", type=Path, help="project directory (default: <out>/<input stem>.rforge)"
    )
    build.add_argument(
        "--density", type=float, help=f"default density in kg/m3 (default {DEFAULT_DENSITY:g})"
    )
    build.add_argument("--scale-to-si", type=float, help="metres per file unit, overrides the file")
    build.add_argument("--format", action="append", choices=_FORMATS, help="default: all")

    export = sub.add_parser("export", help="export URDF and MJCF from a saved project")
    export.add_argument("project", type=Path)
    export.add_argument("--out", type=Path, required=True)
    export.add_argument("--format", action="append", choices=_FORMATS, help="default: all")

    check = sub.add_parser("validate", help="validate a saved project")
    check.add_argument("project", type=Path)
    return parser


def _formats(chosen: list[Format] | None) -> list[Format]:
    return list(dict.fromkeys(chosen)) if chosen else list(_FORMATS)


def _cmd_build(args: argparse.Namespace) -> int:
    source: Path = args.input
    out: Path = args.out
    spec = load_spec(args.spec) if args.spec else BuildSpec()
    if args.density is not None:
        spec.density = args.density
    if args.scale_to_si is not None:
        spec.scale_to_si = args.scale_to_si
    project: Path = args.project or out / f"{source.stem}.rforge"
    model = build_project(source, project, spec)
    result = export_project(model, project, out, _formats(args.format))
    print(f"project: {project}")
    for path in result.written:
        print(f"wrote {path}")
    for warning in result.warnings:
        print(f"warning: {warning}", file=sys.stderr)
    return 0


def _cmd_export(args: argparse.Namespace) -> int:
    loaded = load_project(args.project)
    result = export_project(loaded.model, args.project, args.out, _formats(args.format))
    for path in result.written:
        print(f"wrote {path}")
    for warning in result.warnings:
        print(f"warning: {warning}", file=sys.stderr)
    return 0


def _cmd_validate(args: argparse.Namespace) -> int:
    loaded = load_project(args.project)
    for warning in loaded.warnings:
        print(f"warning: {warning}", file=sys.stderr)
    problems = 0
    for issue in validate(loaded.model).issues:
        label = "error" if issue.severity is Severity.ERROR else "warning"
        print(f"{label}: {issue.code}: {issue.message}")
        problems += issue.severity is Severity.ERROR
    for asset in check_assets(loaded.model, args.project):
        print(f"error: asset {asset.asset!r} of part {asset.part_name!r}: {asset.problem}")
        problems += 1
    if problems == 0:
        try:
            compute_link_masses(loaded.model, args.project)
        except BuildError as exc:
            print(f"error: mass properties: {exc}")
            problems += 1
    print("ok" if problems == 0 else f"{problems} error(s)")
    return 0 if problems == 0 else 1


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    commands = {"build": _cmd_build, "export": _cmd_export, "validate": _cmd_validate}
    try:
        return commands[args.command](args)
    except (BuildError, ProjectError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
