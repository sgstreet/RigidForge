# Milestone progress

State file for the `/milestone-step` loop (`.claude/skills/milestone-step/SKILL.md`). The loop reads and updates this file on every run. Edit the `Answer:` line to unblock it.

- **Milestone:** M1 (headless core, STEP to URDF + MJCF)
- **Branch:** `m1-core`
- **Status:** READY

## Pending question

(none)

## Answered questions

- 2026-11-16: Task 1 (scaffold): `cadquery-ocp` 8.0.1.1.0, the package CLAUDE.md and design doc section 5 name, hard-pins `vtk==9.6.2`. Two problems follow. M1's done definition says CI runs with no PySide6 or VTK installed, which is impossible with this package. And M2 plans VTK 9.7.2 for the viewport, which this pin blocks.
    PyPI also has `cadquery-ocp-novtk` 8.0.1.1.0 (same Apache-2.0 licence and `OCP` module, depends only on `cadquery-ocp-proxy`). Checked 2026-11-16 on Python 3.13: it reads `tests/data/hexapod-leg/coxa-frame.step` through `STEPCAFControl_Reader` with `xstep.cascade.unit = M` set after `Init_s()`, installs no VTK, and imports cleanly next to `vtk==9.7.2` in the same environment. RigidForge does not use OCP's VTK bridge (`OCP.IVtk`), because the viewport tessellates per face itself.
    Should the project depend on `cadquery-ocp-novtk` instead of `cadquery-ocp`? If yes, the loop updates design doc section 5, CLAUDE.md and the licence note, then finishes and commits the scaffold. The scaffold is written but uncommitted (pyproject.toml, .python-version, .gitignore, `src/rigidforge/` with core, importers, physics, exporters).
    Recommended: yes, use `cadquery-ocp-novtk`, and add `vtk` 9.7.2 plus PySide6 and pyvista only in the M2 GUI dependency group.
    Answer: yes, switch to `cadquery-ocp-novtk` (user, in chat, 2026-11-16)
- 2026-11-16: Uncommitted changes are on `m1-core` (the re-exported tibia frame was committed separately): `CLAUDE.md`, `doc/milestones.md`, `doc/rigid-forge-software-stack.md`, `doc/milestone-progress.md` and `.claude/skills/milestone-step/`. The loop will not start task 1 on top of uncommitted work it did not make. May it commit these as one "planning docs and fixtures" commit on `m1-core` and then start task 1?
    Recommended: yes.
    Answer: yes

## M1 status

Spikes (findings in design doc section 6, "Spike findings"):

| Item | Status |
|---|---|
| Spike: STEP units and XCAF assembly | DONE (2026-11-15) |
| Spike: inertia accuracy | DONE (2026-11-15) |
| Spike: URDF vs MJCF semantics | DONE (2026-11-15) |
| Spike: mirror correctness | DONE (2026-11-15) |
| Spike: leg pair placement and horn seating | DONE (2026-11-16) |
| Spike: real 3MF file (part of task 8) | TODO |

Tasks (numbers and dependencies as in `doc/milestones.md`):

| # | Task | Status |
|---|---|---|
| 1 | Scaffold (pyproject, uv lock, .gitignore, markers, ruff + pyright) | DONE (commit "M1 task 1: scaffold") |
| 2 | Model (pydantic, UUIDs, extensions) | TODO |
| 3 | Validation | TODO |
| 4 | Analytic inertia + checks | TODO |
| 5 | Synthetic fixture generator | TODO |
| 6 | STEP importer | TODO |
| 7 | B-Rep mass properties | TODO |
| 8 | Mesh importer (STL, OBJ, DAE, 3MF) | TODO |
| 9 | Persistence | TODO |
| 10 | URDF exporter | TODO |
| 11 | MJCF exporter | TODO |
| 12 | CLI | TODO |
| 13 | Round-trip, determinism and layering tests | TODO |
| 14 | Design doc update from spikes | TODO |
| 15 | Instancing and mirror import | TODO |
| 16 | Servo assembly | TODO |
| 17 | Measured-mass calibration | TODO |
| 18 | Servo inertia override | TODO |
| 19 | Leg assembly script | TODO (needs: joint limits and CAD pose angles, measured link masses, XL330 inertia and CoM source) |

## GUI checks for the user

(none yet)

## Run log

- 2026-11-16: loop set up; waiting for the commit question above.
- 2026-11-16: answer "yes" applied; planning docs, CLAUDE.md and the skill committed on `m1-core`.
- 2026-11-16: started task 1 (scaffold). Paused: `cadquery-ocp` pins vtk 9.6.2; asked whether to switch to `cadquery-ocp-novtk`. Scaffold left uncommitted, task 1 IN PROGRESS.
- 2026-11-16: still waiting for an answer (cadquery-ocp-novtk question); no work done.
- 2026-11-16: answer applied (switch to `cadquery-ocp-novtk`); task 1 (scaffold) done: uv project, src layout, ruff banned-api, pyright strict on core/exporters, markers with auto-skip, `cadquery-ocp-stubs` for OCP types. ruff, format, pyright and pytest (7 passed, 1 skipped) pass.
