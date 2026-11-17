# Milestone progress

State file for the `/milestone-step` loop (`.claude/skills/milestone-step/SKILL.md`). The loop reads and updates this file on every run. Edit the `Answer:` line to unblock it.

- **Milestone:** M1 (headless core, STEP to URDF + MJCF)
- **Branch:** `m1-core`
- **Status:** BLOCKED

## Pending question

Task 19 is done and every M1 task is DONE. Before the milestone close-out, please confirm these assumptions I made in the leg (or correct them):
1. The XL330's 18 g includes its horn, so horns and the idle cap carry no mass (the drive horn's rotating mass therefore sits in the parent link).
2. The tibia shaft is solid PETG (1270 kg/m³); only the frames use 25% infill.
3. The datasheet tensor is about the CoM, and its frame has its origin on the horn's outer face (read from the PDF drawing).
4. Positive joint angle is the servo's +z (right-hand rule): coxa +z, femur and tibia +y, so positive femur and tibia move the tip down.
Then: should the loop run the M1 close-out (check the "Done =" definition end to end, update docs) and merge `m1-core` to `main` and start M2?
Recommended: yes to all four, then close-out, but merge and tagging are yours to do.
Answer:

## Answered questions

- 2026-11-17: Task 19 (leg assembly script) has unresolved "Open before task 19" items in `doc/milestones.md`. I need these to build and export the leg; I will not guess them:
1. Servo-to-frame attachment: by mating faces (horn face flush with frame flange, as verified) or by bolt holes? Recommended: faces, with the bolt pattern only as a placement check.
2. Joint limits (rad) for coxa yaw, femur pitch, tibia pitch, and the joint angles the CAD files are posed at (so the zero pose is defined). Also effort and velocity limits (XL330 datasheet values OK?).
3. Measured masses for the coxa, femur and tibia links (g), or may I use PETG 25% infill placeholder density and flag them as unmeasured?
4. XL330 mass, CoM and inertia source (datasheet mass ~18 g; the datasheet has no tensor). Recommended: use mass 18 g with CoM at the geometry-derived centre and the uniform-density tensor from the servo STEP, marked as an estimate.
Answer:
    Answer (user, in chat, 2026-11-17): 1. Mate the faces; use the bolt pattern as reference. 2. Limits: coxa +-90 deg, femur +-90 deg, tibia -90..+150 deg; home pose is 0 deg for all joints; datasheet effort and velocity are fine. 3. Use PETG 25% infill placeholder density for now; user will measure link masses later. 4. User supplied `XL330,XC330 Moment of Inertia.pdf` (ROBOTIS, reference only): the servo is the XL330-M288: 18.0 g, CoM (-0.29100243, -9.2403727, -13.121748) mm, tensor (g*mm^2) Ixx 2555.5250, Iyy 1271.1372, Izz 2271.2629, Ixy -20.247808, Ixz -11.076849, Iyz -229.79973, in the PDF's servo frame (X right, Y up the horn side view, Z toward the horn; origin on the horn axis).
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
| 14 | Design doc update from spikes | DONE (commit "M1 task 14: design doc update from spikes") |
| 15 | Instancing and mirror import | DONE (commit "M1 task 15: instancing and mirror import") |
| 16 | Servo assembly | DONE (commit "M1 task 16: servo assembly") |
| 17 | Measured-mass calibration | DONE (commit "M1 task 17: measured-mass calibration") |
| 18 | Servo inertia override | DONE (commit "M1 task 18: servo inertia override") |
| 19 | Leg assembly script | DONE (commit "M1 task 19: leg assembly") |

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
- 2026-11-16: task 14 done: design doc section 6 gains spike item 9 (leg pair placement and horn seating); section 7 gains an exporters, CLI and layering note (tasks 10 to 13). Doc-only change; checks pass. Next: task 15 (instancing and mirror import).
- 2026-11-16: task 15 done: `rigidforge.core.instancing` (`rigid_copy`, `mirrored_copy`) and `rigidforge.physics.mirror` (B-Rep via `gp_Trsf.SetMirror`, mesh with reversed winding, reflection matrix); `compute_link_masses` now handles mirrored parts. Tests: reflected CoM and R I Rᵀ on the hexapod frames for three planes, valid mirrored B-Rep, outward-wound meshes, instances and mirrors through the pipeline. All checks pass (300 passed, 1 skipped). Next: task 16 (servo assembly).
- 2026-11-16: task 16 done: `rigidforge.core.instancing.instantiate_assembly` places one imported assembly (the XL330, 16 leaves, one stored file) in a link as rigid copies with composed placements and an optional default density; link merging reuses the parallel-axis `combine`. Tests compare link mass, CoM and inertia against a direct reference, with two servos in one link. Densities for the servo are placeholders until tasks 17 and 18. All checks pass (303 passed, 1 skipped). Next: task 17 (measured-mass calibration).
- 2026-11-16: task 17 done: `part_mass_properties` handles `MeasuredMass` via an effective density (measured / volume), with provenance `measured_mass`; tests against the analytic box and the equivalent density. All checks pass. Next: task 18 (servo inertia override).
- 2026-11-16: task 18 done: `instantiate_assembly(override=, override_on=, com_bounds=)` accepts a manufacturer mass/CoM/tensor in the assembly root frame, validates it (positive mass, SPD, triangle inequality, CoM inside geometry bounds to catch frame/unit mistakes), stores it on one part in that part's frame and marks the rest `Massless` (new mass source; skipped by link merging and the pipeline). Tests: link mass, CoM and rotated tensor against the datasheet values, named carrier, validation errors. All checks pass (308 passed, 1 skipped). Next: task 19 (leg assembly script), which needs user input on joint limits, CAD pose angles, measured link masses and XL330 inertia source, so the next run should ask.
- 2026-11-17: no task started. Task 19 is blocked on the user inputs listed under Pending question; Status set to BLOCKED. Working tree clean.
- 2026-11-17: still waiting for an answer (task 19 questions); no work done.
- 2026-11-17: still waiting for an answer (task 19 questions); no work done.
- 2026-11-17: still waiting for an answer (task 19 questions); no work done.
- 2026-11-17: still waiting for an answer (task 19 open items); no work done.
- 2026-11-17: still waiting for an answer (task 19 questions); no work done.
- 2026-11-17: answer applied (limits, face mating, PETG 25%, XL330-M288 table). Task 19 done: `rigidforge.testing.hexapod_leg.build_leg` (4 links, 3 revolute joints with limits, datasheet effort/velocity, MJCF position actuators, three servos with the manufacturer override) and `tests/physics/test_leg_assembly.py` (model structure, link masses vs geometry, servo tensor mapping, reload-identical exports, pairwise Boolean overlap = only the shaft press fit, flange-horn gaps < 1e-6 m, URDF/MJCF FK vs independent FK at 50 random poses). Verified the XL330-M288 spec on emanual.robotis.com. All checks pass (316 passed, 1 skipped). Bolt-pattern alignment is not checked automatically (placement is verified by overlap and gap instead). Paused for the assumptions above and the close-out decision.
