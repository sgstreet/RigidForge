# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project state

RigidForge is a desktop GUI for building robot models from CAD (STEP) and meshes and exporting them to URDF, MJCF, SDF and USD. **No code exists yet.** The repo holds only the design document `doc/rigid-forge-software-stack.md`, which is the authoritative source for stack choices, versions, architecture and persistence format. Read the relevant section before implementing anything, and update the doc in the same change when a decision changes. The copies of this doc in the claude.ai Project are not authoritative; this repo copy is.

Licence: MPL-2.0.

## Open items (ask before building on these)

These are unresolved in the design doc (section 8). Do not treat them as settled; ask the user first.

- **Isaac Sim target version** and its USD layer conventions. Pin the version in `pyproject.toml` or a constants file once confirmed.
- **Xacro import semantics** (section 5) are proposed, not confirmed: one-way import, copy meshes into `assets/`, keep the original xacro and expanded URDF as provenance.
- **CAD re-import matching and change-report rules** (section 7) are proposed, not confirmed. Re-import itself is a required feature.

Items the doc marks **(verify)** must be checked against primary sources when you reach them, not assumed:

- whether `usd-core` conflicts with Qt/OpenGL (believed not to)
- the exact Isaac Sim GA version and layer conventions

Verified on 2026-11-15 (see the spike findings in design doc section 6): the OCCT STEP unit setting and the MuJoCo 4.0 Python API. Note: call `STEPControl_Controller.Init_s()` before setting `xstep.cascade.unit`, or the setting is silently ignored.

## Rules that are easy to get wrong

- Python **3.12–3.14**, developed on 3.13. Take dependency pins from design doc sections 4 and 5, and re-check PyPI before adding or bumping a package.
- PySide6 only: use `Signal`/`Slot`, never `pyqtSignal`. Set `QT_API=pyside6` before importing pyvistaqt/qtpy, and never install a second Qt binding.
- Depend on `cadquery-ocp` (OCP) directly, not `cadquery`. Read STEP with `STEPCAFControl_Reader` on an XCAF document so the assembly tree, names and colours survive.
- Each export format is **generated directly from the internal model**. Never derive one format from another. `gz sdf -k` is an optional validator only. The PyPI `sdformat` package is unrelated and must not be used.
- The model is a **tree plus optional loop-closure constraints**, not a DAG. Validate single root, no cycles, unique names.
- **Units:** convert geometry to SI (m, kg) at the import boundary, then compute mass properties on the scaled geometry. Never rescale inertia tensors afterwards.
- Inertia is about the CoM. Use parallel-axis shifts only when merging parts into one link. Validate mass > 0, symmetric positive-definite tensor, the triangle inequality on principal moments, and watertight meshes.
- `TopoDS_Shape` is not picklable: pass BREP bytes (`BRepTools.Write_s`) to a process pool and return numpy arrays. coacd runs in a worker **process**. Only VTK/Qt calls run on the main thread.
- Do not create a subpackage named `io`.

## Architecture and layering

Planned layout: `core/` (model, validation), `importers/` (STEP, mesh, URDF/xacro), `physics/`, `ui/`, `viewport/`, `exporters/`, `tests/`. Layering rules follow design doc sections 5 to 7.

- **Headless core:** `core/`, `importers/`, `physics/` and `exporters/` must never import PySide6, pyvista or VTK. Only `ui/` and `viewport/` may. This keeps everything else testable without a display, in CI or a cloud session. Fix any violation rather than working around it.
- Exporters are pure functions `RobotModel -> files`. Simulator-specific data lives in namespaced extension blocks (`extensions.urdf`, `.mjcf`, `.sdf`, `.usd`), each its own pydantic model.
- Picking: tessellate per B-Rep face and store a `face_id` cell-data array so a triangle pick maps back to a `TopoDS_Face`.
- USD export lives in its own module and emits a layered structure (geometry layer plus physics layer).

## Persistence

A project is a `*.rforge/` directory: `project.json` (canonical model), `assets/` (copied, content-hashed CAD and meshes), and `cache/` and `workspace.json` (both gitignored). Full rules are in design doc section 7. The ones to keep in mind:

- Persist inputs, not derived data. Caches are keyed by a hash of geometry, scale, density, method and calc-code version.
- Every link, joint and part is referenced by a stable UUID. Names are not keys.
- Joint frames are numeric and authoritative. B-Rep snap references are hints only.
- JSON is written with sorted keys and stable ordering. `schema_version` drives dict-to-dict migrations that run before pydantic validation. Unknown extension keys must round-trip.
- Saves are atomic (temp file, fsync, rename).

## Milestone workflow

Milestones live in `doc/milestones.md`, which opens with the template below. Work one milestone at a time, one task per sitting.

1. **Brief.** Fill in the template. Resolve every "open before starting" item with the user first.
2. **Spikes first.** Run the milestone's risk spikes as throwaway scripts. Record findings in the design doc in the same change.
3. **Tasks in order.** Each task ends with its tests, passing `ruff check`, `ruff format --check`, `pyright` and `pytest`, and one commit. Tasks in `ui/` or `viewport/` are reported as untested in the GUI, with a manual checklist for the user.
4. **Close-out.** Check the "Done =" definition end to end, update the design doc and this file for changed decisions, and tag the milestone.

Branching: one branch per milestone (`m1-core`, `m2-gui`, `m3-formats`, `m4-release`), one commit per task, merged to `main` once the done definition passes. Do not commit to `main` directly. Commit or push only when the user asks.

Automation: the `/milestone-step` skill (`.claude/skills/milestone-step/SKILL.md`) does one task per run and keeps its state in `doc/milestone-progress.md`. A desktop-app scheduled task runs it every two hours. When it needs a decision it sets `Status: BLOCKED` and writes the question there; fill in the `Answer:` line to unblock it.

Template: Done = (one testable sentence) / Open before starting / Tasks (`->` dependencies) / Risks and spikes (top 3) / Testing / Out of scope.

## Setup and commands

Planned tooling is uv + `pyproject.toml` with a lockfile. First tasks when starting implementation:

1. Create `.python-version` (3.13), `pyproject.toml` and the lockfile; run `uv sync`.
2. Create `.gitignore` covering `cache/`, `workspace.json`, `.venv/` and Python build artefacts. None exists yet.
3. Fill in the real build, lint, type-check and test commands in this section.

There are no build, lint or test commands yet.

## Testing

- Compare computed inertia against analytic box, cylinder and sphere.
- Export round-trips: load MJCF in MuJoCo, URDF in yourdfpy, USD via `Usd.Stage.Open`, and run `gz sdf -k` when available.
- Persistence: save/load equality property tests, one golden fixture per schema version, and byte-identical exports from a loaded project versus the original in-memory model.
- Use pytest markers so environment-dependent tests skip cleanly instead of failing: `gui` (needs a display or Qt), `gazebo` (needs `gz`), `isaac` (needs Isaac Sim). Run Qt widget tests headless with `QT_QPA_PLATFORM=offscreen`.

## What Claude Code cannot verify

A headless run cannot confirm viewport or GUI behaviour: picking, joint snapping, layout, camera. When a change touches `ui/` or `viewport/`, say it is untested in the GUI and ask the user to check, rather than reporting it as working. The same applies to anything needing Isaac Sim or Gazebo that is not installed.

## Dependencies and licences

The project is MPL-2.0 and the stack mixes LGPL (PySide6), MIT, BSD and TOST (usd-core) licences. Check licence compatibility before adding a dependency, and record it in the design doc if it is not already there.

## Code style

Decided: ruff (lint and format) and pyright, with type hints on all public functions. Configuration lives in `pyproject.toml` (`[tool.ruff]`, `[tool.pyright]`).

- Pyright runs strict on `core/` and `exporters/`, and basic mode on `importers/`, `physics/`, `ui/` and `viewport/` (OCP, VTK and pyvista have weak stubs).
- Use a ruff banned-api rule to forbid PySide6, pyvista and vtk imports in `core/`, `importers/`, `physics/` and `exporters/`, and to forbid `pyqtSignal`. Keep the import-blocking test as a backstop.
- Commands (once `pyproject.toml` exists): `uv run ruff check`, `uv run ruff format --check`, `uv run pyright`, `uv run pytest`.
