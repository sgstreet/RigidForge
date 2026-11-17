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
| Spike: real 3MF file (part of task 8) | DONE (2026-11-16) |

Tasks (numbers and dependencies as in `doc/milestones.md`):

| # | Task | Status |
|---|---|---|
| 1 | Scaffold (pyproject, uv lock, .gitignore, markers, ruff + pyright) | DONE (commit "M1 task 1: scaffold") |
| 2 | Model (pydantic, UUIDs, extensions) | DONE (commit "M1 task 2: core model") |
| 3 | Validation | DONE (commit "M1 task 3: validation") |
| 4 | Analytic inertia + checks | DONE (commit "M1 task 4: analytic inertia and checks") |
| 5 | Synthetic fixture generator | DONE (commit "M1 task 5: synthetic STEP fixtures") |
| 6 | STEP importer | DONE (commit "M1 task 6: STEP importer") |
| 7 | B-Rep mass properties | DONE (commit "M1 task 7: B-Rep mass properties") |
| 8 | Mesh importer (STL, OBJ, DAE, 3MF) | DONE (commit "M1 task 8: mesh importer") |
| 9 | Persistence | DONE (commit "M1 task 9: persistence") |
| 10 | URDF exporter | DONE (commit "M1 task 10: URDF exporter") |
| 11 | MJCF exporter | DONE (commit "M1 task 11: MJCF exporter") |
| 12 | CLI | DONE (commit "M1 task 12: CLI") |
| 13 | Round-trip, determinism and layering tests | DONE (commit "M1 task 13: round-trip, determinism and layering tests") |
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
- 2026-11-16: task 2 (model) done: `rigidforge.core` geometry, extensions and model (links, joints, parts, loop constraints, UUID refs, mass sources, overrides), with round-trip and extension tests. All checks pass (24 passed, 1 skipped).
- 2026-11-16: task 3 (validation) done: `rigidforge.core.validation` with stable issue codes (tree, names, IDs, references, limits, axes, poses, mass inputs, loop constraints); 35 table-driven error cases, 3 warning cases and a hypothesis random-tree test. Tensor SPD/triangle checks deferred to task 4. All checks pass (66 passed, 1 skipped).
- 2026-11-16: task 4 (analytic inertia) done: `rigidforge.core.inertia.check_inertia` (mass, finite, symmetric, SPD, triangle inequality, extreme-ratio warning) wired into validation for inertial overrides; `rigidforge.physics.inertia` with box/cylinder/sphere oracles (match OCCT GProp to 1e-6), rotation, parallel-axis, frame change and combine, plus hypothesis tests. All checks pass (101 passed, 1 skipped).
- 2026-11-16: task 5 (synthetic fixtures) done: `rigidforge.testing.step_fixtures` builds XCAF assemblies from analytic primitives (names, colours, nested sub-assembly, a part instanced twice) and writes STEP in mm or m, with the expected placements, volumes and bboxes; read-back tests through `STEPCAFControl_Reader`. Finding: OCCT STEP output is not byte-reproducible (recorded in design doc section 6, item 7). All checks pass (115 passed, 1 skipped).
- 2026-11-16: task 6 (STEP importer) done: `rigidforge.importers.step` (XCAF tree to shared products and placed leaves in metres, declared-unit and document-unit checks, `scale_to_si` override applied to the B-Rep, candidate parts) and `rigidforge.core.transforms`; tested on the synthetic fixtures (mm and m) and the hexapod coxa frame and XL330 assembly. All checks pass (149 passed, 1 skipped).
- 2026-11-16: task 7 (B-Rep mass properties) done: `rigidforge.physics.brep` (adaptive GProp, Eps 1e-12, closed shells only; rejects surface models and reversed solids) and `rigidforge.physics.mass` (unit-density results, density and override paths with provenance, link merge, geometry cache key); `StepImport.leaf_at`. Finding: default GProp integration is 2e-6 to 1e-5 off on the hexapod frames, recorded in design doc section 7. Measured mass raises until task 17. All checks pass (177 passed, 1 skipped).
- 2026-11-16: task 8 (mesh importer) done: `rigidforge.importers.mesh` (STL, OBJ, DAE, 3MF to metres, one candidate part per object, composed placements, non-rigid transforms baked, welded closed-mesh check) and `rigidforge.physics.mesh` (exact tetrahedron mass properties); synthetic 3MF/STL/OBJ/DAE box fixtures with analytic checks; real-3MF spike recorded in design doc section 6, item 8; `pycollada` added. All checks pass (194 passed, 1 skipped).
- 2026-11-16: task 9 (persistence) done: `rigidforge.core.project` (canonical atomic `project.json` with rotating backup, migration chain, read-only for newer schemas, content-hashed asset copy and checks), golden fixture `tests/data/golden/project-v1`, hypothesis save/load test, byte-identical resave. All checks pass (223 passed, 1 skipped).
- 2026-11-16: task 10 (URDF exporter) done: `rigidforge.exporters.urdf.export_urdf(model, masses, visuals, collisions)` writes deterministic URDF with lxml (validates first; rpy with gimbal-lock handling; requires effort and velocity on revolute and prismatic joints; loop constraints dropped with a warning); tested against analytic box inertia, URDF inertia order and yourdfpy FK at random poses. `types-lxml` (Apache-2.0) added as a dev dependency for pyright strict. All checks pass (237 passed, 1 skipped).
- 2026-11-16: task 11 (MJCF exporter) done: `rigidforge.exporters.mjcf.export_mjcf` (bodies from the joint tree, `fullinertia` in MJCF order, hinge/slide joints with range, damping, friction and effort limits, position actuators from `extensions.mjcf`, loop constraints as site-based connect/weld, mesh assets with visual group 2 and collision group 3); shared `MeshRef` and number formatting moved to `exporters/common.py`. Tested by compiling in MuJoCo (masses, principal moments, full tensor, joints, actuators, meshes, loops) and FK against the URDF export at 50 random poses. All checks pass (246 passed, 1 skipped).
- 2026-11-16: task 12 (CLI) done: `rforge build|export|validate` (`rigidforge.cli`) over `rigidforge.pipeline` (JSON build spec with links, joints in the import frame, densities and MJCF position actuators; default is one link per leaf joined by fixed joints from the first; masses recomputed from the project's own assets). Tested on the synthetic arm (analytic masses, MuJoCo compile, yourdfpy FK, export from a reloaded project byte-identical), a mesh input and bad specs. Not done here: visual/collision mesh files in the exports (the exporters accept `MeshRef`s; B-Rep tessellation is not written yet), and mirrored parts (task 15). All checks pass (266 passed, 1 skipped).
- 2026-11-16: task 13 done: `tests/test_layering.py` (fresh-interpreter import of all headless packages, pipeline and CLI with PySide6/VTK/pyvista/PyQt blocked and a leak check; AST backstop; no `io` subpackage) and `tests/test_roundtrip.py` (in-memory vs reloaded exports, save/load/save byte stability, repeated and location-independent exports, nested STEP and STL inputs). All checks pass (280 passed, 1 skipped).
