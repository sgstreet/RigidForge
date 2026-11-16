# Milestone template

- **Done =** one testable sentence
- **Open before starting:** unresolved decisions to settle with the user
- **Tasks:** ordered, one sitting each, `->` for dependencies
- **Risks and spikes:** top 3, each with a 1-2 hour spike
- **Testing:** unit, golden-file, headless, and manual checks
- **Out of scope**

Workflow and branching rules are in CLAUDE.md ("Milestone workflow").

---

# Milestone 1: Headless core, STEP to URDF + MJCF

## Context
Repo has only the design doc and CLAUDE.md. CAD library is already settled (OCP, `STEPCAFControl_Reader`). Single-user, local. Chosen scope: headless core + CLI, URDF and MJCF exports, synthetic fixtures. No GUI, no USD/SDF, no xacro, no re-import (all later).

## Milestone (1-2 weeks)
**Done =** `rforge build robot.step --out out/` (or a Python test) takes a generated 2-3 part STEP assembly, builds a tree model with a revolute joint, computes SI mass/CoM/inertia, validates, saves/loads a `*.rforge/` project, and writes URDF and MJCF. The MJCF compiles in MuJoCo, the URDF loads in yourdfpy, inertia matches analytic box/cylinder/sphere, and a reloaded project exports byte-identical files. CI runs headless with no PySide6/VTK installed.

## Tasks (each one sitting; `->` = depends on)
1. Scaffold: `.python-version` 3.13, `pyproject.toml`, uv lock, `.gitignore`, pytest markers (`gui`, `gazebo`, `isaac`), layout `core/ importers/ physics/ exporters/ tests/`. Confirm ruff + pyright choice with user, then record commands in CLAUDE.md.
2. Model (`core/`): pydantic Link/Joint/Part with UUIDs, SI units, extension blocks, loop-constraint stub. -> 1
3. Validation (`core/`): single root, no cycles, unique names, joint limits sane. -> 2
4. Analytic inertia (`physics/`): box/cylinder/sphere oracles plus parallel-axis merge helper, mass > 0, SPD, triangle inequality checks. -> 1
5. Synthetic fixture generator (OCP): STEP files with assembly names and colours. -> 1
6. STEP importer (`importers/`): XCAF reader to part tree, unit check, scale to SI at the boundary. -> 5, 2
7. B-Rep mass properties on scaled geometry (`GProp_GProps`), mass provenance recorded. -> 6, 4
8. Mesh importer + watertight check (trimesh), mesh inertia. Formats: STL, OBJ, DAE and 3MF. For 3MF, read the model unit and scale to SI at the boundary, apply build-item and component transforms, and import each object as a candidate part. -> 2, 4
   - Add a synthetic 3MF fixture (a box in mm with a transform) and test that bbox, volume and inertia in SI match the analytic values.
   - Spike (1 h): load a real 3MF (for example from a slicer) and check units, transforms, multiple objects and watertightness.
9. Persistence: `project.json` (sorted keys, atomic save), `assets/` content-hash copy, `schema_version` + first golden fixture. -> 2
10. URDF exporter (lxml, pure function). -> 3, 7
11. MJCF exporter, compiled with MuJoCo. -> 3, 7
12. CLI (`rforge build`, `rforge validate`) wiring import -> save -> export. -> 9, 10, 11
13. Round-trip + determinism tests, import-boundary check that headless packages never import PySide6/VTK. -> 12
14. Update design doc with decisions from the spikes. -> spikes

### Hexapod leg fixture (tests/data/hexapod-leg)
Eight mm STEP files, each at its own origin: `coxa-frame`, `femur-frame`, `tibia-frame` (one solid each, one mirror half of the link), `tiba-shaft`, and the XL330 servo (`xl330-Body` assembly plus drive horn, idle horn and idle cap). Added tasks:
- **15. Instancing and mirror import.** Place a part as a rigid copy (rotation plus translation, no new geometry) or as a mirror about a plane of its own file's coordinate system (baked into the B-Rep before mass properties, no negative scale). -> 6, 7
- **16. Servo assembly.** Import the XL330 once as an assembly and instance it per link, with parallel-axis merging into the link. -> 6, 7
- **17. Measured-mass calibration.** Per part, a measured mass sets a density scale factor so computed mass matches; inertia scales with it at fixed shape. -> 7
- **18. Servo inertia override.** Accept a manufacturer inertia tensor, check its frame and CoM, and validate it as SPD. -> 4, 16
- **19. Leg assembly script.** Build the 3-joint leg (coxa yaw, femur pitch, tibia pitch), export URDF and MJCF with position actuators, and run the FK comparison. -> 12
- **Extra spike:** mirror correctness (positive volume, CoM mirrored, inertia equal after axis mirroring, no inside-out mesh in the exports).
- **Material:** PETG, 25% infill; the effective density is set from measured link mass, not a handbook value.
- **MJCF actuators:** position actuators are in M1 (rough kp, damping, effort limits, XL330 values to be verified against the datasheet).
- **Leg layout (from user, 2026-11-15, plus geometry inspection):** each servo is sandwiched between two mirrored frame halves. The coxa servo horn points up (servo z+ = world z+). The femur and tibia servos are rotated so servo z+ = world y+. XL330 local frame: origin on the horn axis, horn axis = z, body spans x -24.5..9.5, y ±10, z ±11.5 mm. Both horns have a 4-bolt pattern at 6 mm radius, matched by the frames' bolt patterns at their origins.
- **Proposed interpretation (to confirm):** coxa link = coxa frames + femur servo body (femur horn axis along y at x = 43.5 mm); femur link = femur frames + tibia servo body (tibia axis at x = 47.5 mm); tibia link = tibia frames + shaft (shaft along x). The coxa servo body belongs to the base. Horns rotate with the child link, so their mass goes to the child. Frame halves are placed by mating their bolt pattern to the drive horn and idle horn on opposite sides of the servo, not by mirroring about the file origin alone.
- **Confirmed (user, 2026-11-15):** the link breakdown above is right, and the coxa servo body belongs to the base. The drive horn is on servo z+ (idle horn on z-). The tibia frame origin is the tibia servo origin (joint axis). The shaft is centred between the tibia frame pair at the far end, along x, and the tibia link is 75 mm from joint axis to shaft tip.
- **From geometry (tibia-frame.step re-exported 2026-11-16):** the frame spans y -19.5..0, and its origin is on the seam face, which is the pair's centre plane and the tibia servo's centre plane. The shaft groove (r 3.58 mm, x = 31..45 mm, 14 mm deep) is a half-hole in the seam face, so the 44 mm shaft spans x = 31..75 mm, consistent with the 75 mm link length.
- **Femur frame (user, 2026-11-15):** origin is on the flange face opposite the servo horn; the flange is 2 mm thick. Geometry agrees: large y-normal planes at y = 0 and y = 2 mm.
- **Horns (user, 2026-11-15):** each horn's outer face is 3 mm from the servo body face, so at servo z = ±14.5 mm (body faces at ±11.5). The drive horn file's outer face is at its z = 1.45 mm, so the drive horn origin sits at servo z = +13.05 mm.
- **Frame offsets (verified 2026-11-16):** the servo centre is at the coxa frame origin, at y = +16.5 mm in the femur frame file (origin on the outer flange face), and at the tibia frame origin. With each horn's outer face at 14.5 mm from the servo centre (11.5 mm body half-thickness + 3 mm), every frame's horn face seats flush: frame material starts at 14.5 mm, with zero overlap and zero gap.
- **Tibia pair (user screenshot, 2026-11-16):** a U shape. The two arms bolt to the tibia servo horns, and the far-end blocks meet at the centre seam, clamping the shaft, which exits the end face centred. The groove radius (3.58 mm) is deliberately smaller than the shaft (3.76 mm) for a press fit, so the interference check must allow this shaft-to-groove overlap.
- **Second half of every pair (user, 2026-11-16):** coxa, femur and tibia pairs all use a copy of the same frame rotated 180° about the frame's x axis through the servo centre (the same printed part used twice; no mirrors in this leg). Verified: zero overlap within each pair (gaps: coxa 9 mm, femur 19 mm, tibia 0 mm, i.e. touching at the seam), and the tibia shaft overlaps each groove by 28.3 mm³ (the press fit). The model still supports mirrored copies for other designs; task 15 covers both.
- **Placement check for task 19:** after assembling, compute the Boolean common volume between every pair of parts (must be ~0) and check that each frame flange touches its horn face (gap ~0). This catches wrong offsets or mirror sides automatically.
- **Open before task 19:** how each servo attaches to its frame (faces or bolt holes), joint limits and the CAD pose angles, measured masses, and the XL330 CoM and inertia source.

Tasks 4, 5 and 2 can proceed in parallel after 1.

## Top risks and spikes (do first, ~1-2 h each)
1. **STEP loading/units.** Spike: generate a mm-based STEP in OCP, read via `STEPCAFControl_Reader`, print names, colours, bbox, and the effect of `xstep.cascade.unit`. Resolves a design-doc (verify) item.
2. **Inertia accuracy.** B-Rep exact vs mesh, non-uniform density, hollow parts. Spike: compare `GProp_GProps` and trimesh on a box with a hole against the analytic result; decide the policy for parts with manual mass override (scale the inertia at fixed shape, not a tensor rescale).
3. **URDF vs MJCF semantics.** Spike: hand-write one 2-link robot in both, load in yourdfpy and MuJoCo (check the 4.0 `MjSpec` API, a (verify) item), compare FK at a few joint angles. Watch for MJCF `fusestatic`/`balanceinertia`, inertial frames, and joint axis/limit conventions.

## Testing
- Inertia: parametrised analytic box/cylinder/sphere, tolerance 1e-6 relative; parallel-axis merge vs a single compound solid; hypothesis tests for SPD and triangle inequality.
- Exports: golden-file tests (normalised XML) for a fixed model; MuJoCo compile succeeds; yourdfpy loads; FK agreement between the two at random joint angles.
- Kinematic validation (headless): table-driven bad models (two roots, cycle, duplicate name, zero mass, NaN limits) assert specific error codes.
- Persistence: hypothesis save/load equality, golden project per schema version, unknown extension keys round-trip, exports byte-identical after reload.
- Layering: a test that imports `core`, `importers`, `physics`, `exporters` with PySide6/VTK blocked.

## Out of scope for M1
SDF, USD (Isaac Sim version unpinned), xacro import, CAD re-import, any `ui/` or `viewport/` work.

---

# Milestone 2 (1-2 weeks): GUI over the M1 core
Starts after M1 is done. All GUI behaviour is **untested by Claude headless**; each task ends with a manual check by you.

## Done =
Open the app, import a STEP file, see the part tree and a 3D view, click a part (and a face) to select it, create a link and a revolute joint by snapping to a cylindrical face axis, edit joint limits/mass/density in a property panel, see mass/CoM/inertia and validation warnings live, undo/redo, save/reopen the `*.rforge/` project (camera/layout restored), and export URDF + MJCF from a dialog. Imports and tessellation run off the UI thread without freezing it.

## Tasks (`->` = depends on)
1. UI scaffold (`ui/`): `QT_API=pyside6`, main window, dock layout, `workspace.json` load/save. Core packages stay Qt-free. -> M1
2. Background worker: process pool for STEP load/tessellate (BREP bytes in, numpy out), progress + cancel via `Signal`/`Slot`. -> 1
3. Viewport (`viewport/`): `QtInteractor`, per-part actors from tessellation with `face_id` cell data, camera reset, axes. -> 2
4. Project tree + selection model shared by tree and viewport. -> 1, 3
5. Face picking: triangle pick -> `face_id` -> `TopoDS_Face`; highlight; report surface type (cylinder axis / plane normal) via `BRepAdaptor_Surface`. -> 3
6. Command layer + `QUndoStack`: add/remove link, set joint, edit properties as model commands. -> 4
7. Joint creation by snapping: axis from picked cylinder, origin from face; store numeric frame, snap ref as hint only; joint frame gizmo. -> 5, 6
8. Property panel: mass source (density x volume vs explicit), joint limits/type/axis, manual inertia override. -> 6
9. Live physics/validation panel: recompute via cache (keyed by hash), show inertia ellipsoid/CoM marker, list validation errors, click error -> select item. -> 4, 8
10. Joint slider preview (FK with scipy), values saved in `workspace.json`. -> 7
11. Save/open/recover: file dialogs, autosave recovery file, lock file, stale-source hint (no re-import yet). -> 6
12. Export dialog: pick targets, output dir, run exporters in worker, show MuJoCo compile / validation result. -> 9
13. Polish: error dialogs for unsupported STEP, empty states, shortcuts.

Tasks 5 and 6 can run in parallel after 3/4.

## Top risks and spikes
1. **pyvistaqt + PySide6 integration.** Spike (1 h): minimal `QtInteractor` window with `QT_API=pyside6`, no second Qt binding installed, on your Linux GPU/Wayland setup; confirm `usd-core` import does not break it (design-doc verify item).
2. **Face picking fidelity.** Spike: tessellate a cylinder-through-box per face, pick with `vtkCellPicker`, confirm `face_id` -> `TopoDS_Face` -> correct axis; check cost on a ~100-part assembly.
3. **Threading/responsiveness.** Spike: load a large STEP in the process pool, confirm the UI stays live and cancel works; confirm results marshal to the main thread only for VTK calls.

## Testing
- Headless Qt (`QT_QPA_PLATFORM=offscreen`, marker `gui`): model/command tests with `QUndoStack` (do/undo/redo restore model equality), selection-model sync, workspace.json round-trip, worker `Signal` delivery with `pytest-qt`.
- Pure-logic unit tests (no Qt): face-id lookup, snap-frame math (cylinder axis -> joint frame), FK preview vs yourdfpy.
- Not verifiable by Claude: rendering, picking feel, gizmo, layout. Each task gets a short manual checklist; I will say "untested in GUI" and ask you to confirm.

## Open before starting M2
Code style/tooling (ruff + pyright) must be confirmed; the Isaac Sim version, xacro semantics and re-import rules remain unresolved and are deferred to M3+.

---

# Milestone 3 (outline): Remaining formats and CAD workflow
Starts after M2. Needs the open items below resolved first.

## Done =
Export SDF and USD from the same model, re-import a changed CAD file with a change report, import a URDF/xacro, and generate collision geometry, without losing joint frames or overrides.

## Tasks (`->` = depends on)
1. Pin the Isaac Sim version and confirm its USD layer conventions (design doc verify item). Record in `pyproject.toml` or a constants file.
2. SDF exporter (lxml, direct from the model); validate with `gz sdf -k` under the `gazebo` marker. -> M1
3. USD exporter in its own module: geometry layer plus physics layer, tuning in `extensions.usd`; check with `Usd.Stage.Open`, and open in Isaac Sim under the `isaac` marker. -> 1
4. Collision workflow: coacd in a worker process, hull cache keyed by hash, stale flags. -> M2
5. CAD re-import: import records, matching (XCAF name/path, then volume + bbox, then ask), preserved vs recomputed data, change report, orphans, undoable. Rules must be confirmed first. -> M2
6. Stale-source detection on project open, offer re-import. -> 5
7. URDF/xacro import: expand with `xacro`, parse into the model, copy meshes into `assets/`, keep provenance, args and package-path dialog. Semantics must be confirmed first. -> M2
8. Loop-closure constraints: UI to add them, validation, MJCF/SDF export (URDF warns). -> M2

## Risks and spikes
1. **USD/Isaac conventions.** Spike: hand-author a 2-link USD with `UsdPhysics`, open it in the pinned Isaac Sim version.
2. **Re-import matching.** Spike: re-export a modified STEP and test whether name/path matching survives.
3. **xacro resolution.** Spike: expand a real-world xacro using `$(find pkg)` and see what fails.

## Testing
Golden files for SDF and USD, `gz sdf -k` and Isaac checks under their markers, re-import fixtures (moved, resized, removed and added parts), xacro round-trip to the model.

## Open before starting M3
Isaac Sim version, xacro import semantics, re-import matching and change-report rules.

---

# Milestone 4 (outline): Release hardening and Briefcase packaging

## Done =
A Briefcase-built Linux package installs and runs on a clean Ubuntu 26.04 Wayland machine, opens a sample project, and exports all four formats.

## Tasks
1. Packaging spike: build a minimal Briefcase app bundling PySide6, VTK, cadquery-ocp-novtk and usd-core. Check that the Wayland plugin, the OpenGL context and USD `plugInfo.json` resources work from the bundle.
2. Briefcase config in `pyproject.toml`; build and run scripts. Record the exact commands in CLAUDE.md.
3. Fix bundling gaps found in the spike (data files, plugin paths, licence notices for LGPL PySide6 and TOST usd-core).
4. Clean-machine test: fresh Ubuntu 26.04 VM or container, install the package, run an end-to-end export.
5. Schema-migration tests for every version, crash-recovery polish, performance pass on large assemblies.
6. Sample project, user docs, release notes, version tag.

## Risks
- **Bundle size and missing data files** (VTK, USD plugin metadata). Spike first.
- **Briefcase on Linux** produces a system package (deb) or Flatpak/AppImage-style bundle; confirm which format works with Wayland and OpenGL.
- **conda-forge fallback** if OCP or VTK wheels conflict with the bundle.

## Testing
Smoke test of the built package on a clean machine; exports from the bundle compared against the source-installed app; licence-file check.
