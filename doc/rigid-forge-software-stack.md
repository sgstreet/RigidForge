# RigidForge — Validated Software Stack

Status: validated 2026-11-15 against PyPI metadata and public docs. Supersedes the stack in the original Gemini chat ("Python URDF GUI Stack Selection"). Items marked **(verify)** were not confirmed against primary sources and should be checked at implementation time.

## 1. Verdict

The architecture holds up: Python + PySide6 + PyVista/VTK viewport, OpenCascade (OCP) for B-Rep, trimesh for meshes, an internal model with pure-function exporters. Eight claims in the original chat were wrong or risky and are corrected below. The most important are the SDF binding, the inertia unit-scaling formula, and the "DAG" model.

## 2. Decisions

- **Direct generation.** URDF, MJCF, SDF and USD are each generated directly from the internal model. No format is derived by converting another (decided 2026-11-15).
- **Persistence.** One person resuming work over days. Project directory with a JSON model file (section 7).
- **Concurrent or collaborative editing is not in scope.**
- **Xacro import is in scope** (section 5, "URDF / xacro import").
- **USD exporter target is the latest Isaac Sim release** (section 5, "USD"). Pin the exact version at implementation time.
- **CAD assets are copied into the project**, and re-importing a changed CAD object is a required feature (section 7).

## 3. Corrections to the original chat

| # | Original claim | Finding | Fix |
|---|---|---|---|
| 1 | `sdformat` Python bindings for URDF→SDF | The PyPI package `sdformat` is an unrelated 2019 GPLv3 project. Official bindings (`sdformat14` etc.) ship with libsdformat via apt/conda, not as a PyPI wheel. URDF→SDF is done by the `gz sdf` CLI. | Generate SDF directly from the internal model. Optionally validate with `gz sdf -k` if Gazebo tools are installed. `python-sdformat` (BSD-2, v0.4.0) or `rod` are pure-Python helpers if a typed SDF layer is wanted. |
| 2 | `I_scaled = s²·I_cad`, `m = s³·ρV` | Dimensionally unsafe. At fixed density inertia scales as s⁵, at fixed mass as s². Applying one factor to a tensor of unknown provenance is how silent errors happen. | Convert geometry to metres and kg at the import boundary (scale vertices / apply `gp_Trsf` scale to the B-Rep), then compute mass properties on the already-scaled geometry. Never rescale tensors afterwards. |
| 3 | "Shift inertia from CoM to the joint/link origin" | URDF `<inertial>` takes the tensor about the CoM, with the inertial frame origin at the CoM. MJCF `<inertial>` is the same. | Parallel-axis shifts are only needed when merging parts into one link (shift each to the combined CoM). |
| 4 | Robot as a DAG | URDF is a tree (one parent per link). Closed loops are not representable. SDF and MJCF can express loops. | Model a tree plus optional loop-closure constraints. Validate: single root, no cycles, unique names. |
| 5 | `pyqtSignal` in QThread guidance | PySide6 uses `Signal`. | Use `Signal` / `Slot`. |
| 6 | `cadquery-ocp` wheels are ~500 MB+ | Current wheels are at most ~68 MB. | Not a real concern. |
| 7 | usd-core + Qt will fight over OpenGL/TBB | Likely overstated: the `usd-core` wheel is the core libraries without the imaging/Hydra viewer stack **(verify)**. | Keep USD export in its own module and test it in CI; no special isolation needed beyond that. |
| 8 | `yourdfpy` loads Xacro | It does not expand xacro. Xacro is a separate preprocessor (`xacro` on PyPI, 2.1.1). Features like `$(find pkg)` need ROS package paths. | See "URDF / xacro import" in section 5. |

## 4. Python version window

| Package | Python |
|---|---|
| PySide6 6.12.0 | ≥3.10, <3.16 |
| numpy 2.5.4 / scipy 1.18.1 | ≥3.12 |
| cadquery-ocp-novtk 8.0.1.1.0 | ≥3.11, <3.15 |
| usd-core 26.11 | ≥3.11, <3.15 |
| mujoco 4.0.1 | ≥3.11 |
| vtk 9.7.2 | wheels cp310–cp315 |
| pyvista 0.49.2, pyvistaqt 0.13.2, trimesh 5.2.1, pydantic 2.14.3, coacd 1.0.14 | ≥3.9–3.10 |

Intersection: **3.12 – 3.14**. Recommend developing on 3.13 and testing 3.12 and 3.14 in CI. Linux x86_64/aarch64, macOS and Windows wheels exist for cadquery-ocp-novtk, usd-core and vtk. RigidForge only writes USD files and does not run inside Isaac Sim, so Isaac's own Python (3.12 for 6.x, 3.11 for 5.x) does not constrain this window.

## 5. Final stack

| Layer | Choice | Notes |
|---|---|---|
| GUI | PySide6 6.12 (LGPL) | Set `QT_API=pyside6` before importing pyvistaqt / qtpy, which otherwise defaults to PyQt5. Do not install other Qt bindings in the same environment. **Target platform is Wayland only (Ubuntu 26.04, no X11/XWayland).** Use Qt's bundled Wayland plugin (`QT_QPA_PLATFORM=wayland`), and do not depend on xcb libraries. Confirm the PyPI `vtk` wheel renders, picks and scales correctly through pyvistaqt's Qt-owned OpenGL context on Wayland **(verify)**; fallbacks are EGL/OSMesa or conda-forge builds. |
| Viewport | pyvista + pyvistaqt (`QtInteractor`) over VTK 9.7 | MIT / BSD. Drop to raw VTK only if pyvista's picking is limiting. |
| B-Rep / STEP | `cadquery-ocp-novtk` directly (Apache-2.0) | Depend on OCP, not the full `cadquery` package. Use the `-novtk` build (decided 2026-11-16): `cadquery-ocp` hard-pins `vtk==9.6.2`, which would put VTK in the headless core and block the viewport's VTK 9.7.2. Both builds provide the same `OCP` module; RigidForge does not use OCP's VTK bridge (`OCP.IVtk`). Checked 2026-11-16: the novtk build reads STEP through XCAF with the unit setting working, and imports alongside `vtk==9.7.2`. Never install both builds in one environment. Type stubs come from `cadquery-ocp-stubs` (Apache-2.0, same publisher and version, dev dependency only). `cadquery.importers.importStep` uses `STEPControl_Reader` and drops colours and assembly metadata. Use `STEPCAFControl_Reader` with name, colour and material modes on an XCAF document to get the assembly tree. `xstep.cascade.unit` verified 2026-11-15 (section 6, spike 1). |
| Meshes | trimesh 5.2 (+`pycollada` for DAE; `trimesh[easy]` pulls lxml, networkx, manifold3d) | `moment_inertia` is about the CoM at the mesh's density; `moment_inertia_frame()` gives other frames. Results are meaningless unless the mesh is watertight, so check `is_watertight` first. **3MF import is in scope** via trimesh's 3MF loader (needs lxml, in `trimesh[easy]`): a 3MF file is a scene of objects with build-item transforms and components, so import each object as a candidate part and apply the transforms. 3MF declares its own unit (default millimetre), so read the unit from the model and convert to metres at the import boundary **(verify trimesh's handling of the 3MF `unit` attribute against the 3MF spec and a real file)**. Colours and materials are optional; 3MF is mesh-only, with no B-Rep or assembly names beyond object names. |
| Collision | coacd 1.0 (MIT) | Run in a worker process, not a thread. |
| Math | numpy, scipy.spatial.transform | |
| Data model | pydantic v2 | Tree of Links / Joints plus loop constraints, plus per-simulator extension blocks (section 7). |
| URDF export | direct lxml serializer | yourdfpy 0.0.60 (MIT) or urchin for FK preview only. Last yourdfpy release was Jan 2026; urchin is releasing more recently. |
| URDF / xacro import | `xacro` 2.1.1 (pip) to expand, then a URDF parser into the internal model | See below. |
| MJCF | direct generation, validated by compiling with mujoco 4.0.1 (`MjSpec` or XML string) | Python API verified 2026-11-15 against mujoco 4.0.1 (section 6, spike 4). |
| SDF | direct lxml serializer; `gz sdf -k` for optional validation | See correction 1. |
| USD | usd-core 26.11 (TOST licence), targeting the latest Isaac Sim | Build stages with `UsdPhysics` schemas. See below. |
| Packaging | uv + `pyproject.toml`, lockfile | Briefcase for distribution (decided; build in M4, after a bundling spike for the Wayland plugin, OpenGL and USD plugin data). conda-forge is the fallback if OCP wheels conflict with a system library. |

**URDF / xacro import (in scope).** Proposed semantics, to confirm: import is one-way. Run `xacro` to expand to URDF, parse the URDF into the internal model, and resolve `package://` and `file://` mesh URIs, copying the meshes into `assets/`. The model does not round-trip back to xacro. Keep the original xacro and its expanded URDF in the project as import provenance. Expose user-supplied xacro args and package search paths in the import dialog, because `$(find pkg)` and `xacro.load_yaml()` need files on disk and a package path. Fail with a clear message when they cannot be resolved. Imported URDFs often have joint and inertial data with no CAD source, so those links get meshes only, with manual mass and inertia and the source marked "imported".

**USD target: latest Isaac Sim.** As of 2026-11-15, Isaac Sim 6.x docs are published up to 6.2.0, and 6.0.0 moved to Kit 110 and Python 3.12. Sources report that 6.0 introduced "Asset Structure 3.0": USDC for geometry, with materials, instances, base and physics in separate USDA layers, and separate MuJoCo and PhysX definitions for tuning values. 6.x also supports multiple physics backends (PhysX and Newton). Design the USD exporter to emit the layered structure (geometry layer plus physics layer, with physics tuning kept in the `extensions.usd` block). Pin the target Isaac Sim version in `pyproject.toml` or a constants file and test exported assets by opening them in that version **(verify the exact GA version and layer conventions when implementing)**.

## 6. Design points the original chat missed

1. **Face-to-B-Rep mapping for picking.** VTK picks triangles, not B-Rep faces. Tessellate per face (`BRepMesh_IncrementalMesh`) and store a `face_id` cell-data array on the mesh. A pick then maps to a `TopoDS_Face`, and `BRepAdaptor_Surface` can return the cylinder axis or plane normal for joint snapping.
2. **Threading.** Run STEP load, tessellation and coacd off the UI thread. `TopoDS_Shape` is not picklable, so pass BREP bytes (`BRepTools.Write_s`) to a process pool and return numpy arrays plus mass properties. Only VTK/Qt calls run on the main thread.
3. **Units and mass provenance.** Store per part: unit-normalised geometry (m), mass source (explicit mass or density × volume), and the inertia method (B-Rep exact or mesh). Surface the source in the UI.
4. **Inertia validation.** For each link: mass > 0, tensor symmetric and positive definite, principal moments satisfy the triangle inequality (Ixx + Iyy ≥ Izz and permutations), meshes watertight. Warn on extreme aspect ratios of principal moments.
5. **Package layout.** Avoid a subpackage named `io` (confusing next to stdlib). Suggested: `core/` (model, validation), `importers/` (STEP, mesh, URDF/xacro), `physics/`, `ui/`, `viewport/`, `exporters/`, `tests/`. Exporters are pure functions `RobotModel -> files`.
6. **Test oracles.** Compare computed inertia against analytic box, cylinder and sphere. Round-trip exports: load MJCF in MuJoCo, load URDF in yourdfpy, open USD with `Usd.Stage.Open`, run `gz sdf -k` where available.

### Spike findings (2026-11-15)

Run with cadquery-ocp 8.0.1.1.0 (since replaced by cadquery-ocp-novtk 8.0.1.1.0, the same OCP build without the VTK pin), trimesh 5.2.1, mujoco 4.0.1, yourdfpy 0.0.60 on Python 3.13, against `tests/data/hexapod-leg`.

1. **STEP units.** `xstep.cascade.unit` is empty until the STEP controller is initialised. `Interface_Static.SetCVal_s` called before that returns `False` and is ignored silently, and the file then reads in mm. Call `STEPControl_Controller.Init_s()`, then set `"M"`, check the return value, and assert the result (for example by bounding box) in each worker process, because the setting is process-global. Volumes scale exactly (4475.23 mm³ to 4.47523e-6 m³). `STEPCAFControl_Reader` keeps names, colours, nested sub-assemblies and instance references (the XL330 body: 10 products, 4 screw instances, 2 connector sub-assemblies).
2. **OCP 8 API differences.** Label sequences are `OCP.collections.Sequence_TDF_Label` (no `TDF_LabelSequence`). `TopoDS.Face` has no `_s` suffix. `Bnd_Box.Get()` is unbound; use `CornerMin`/`CornerMax`. `BRepBndLib.Add_s` is loose on curved geometry (1.5 mm over on a 33 mm frame); use `AddOptimal_s` for change reports.
3. **Mirroring.** Mirroring a B-Rep with `gp_Trsf.SetMirror` plus `BRepBuilderAPI_Transform(copy=True)` gives positive volume, CoM equal to the reflected CoM, and inertia equal to R·I·Rᵀ, with a watertight, consistently wound tessellation. Bake mirrors into geometry at import; never use negative mesh scale.
4. **Inertia accuracy.** B-Rep `GProp` matches analytic box, cylinder, sphere and box-with-hole to 1e-15. Mesh inertia from tessellation is off by about 1e-3 on curved surfaces (sphere 1e-2 at default settings, dominated by angular deflection, not linear deflection). Use B-Rep mass properties whenever a B-Rep exists; mesh-only inputs (STL, OBJ, DAE, 3MF) get mesh inertia with that error. On the hexapod frames, mesh and B-Rep agree to 3e-4. Naively concatenated per-face triangulations of a sphere were not watertight (seam and pole vertices), so the importer must weld vertices with a tolerance before the watertight check.
5. **Non-uniform density (3D printing).** A printed part is not uniform (solid walls, sparse infill), and the model treats it as uniform. A measured mass sets an effective density for the part: mass = measured, inertia = (measured / volume) × unit-density inertia, CoM from geometry. Example: the coxa frame as solid PETG (1270 kg/m³) is 5.68 g. This is an approximation; the error is not quantified.
6. **URDF vs MJCF.** A 3-joint leg written in both formats gives identical FK at 200 random poses (6.9e-17 m). `MjSpec.from_string(...).compile()` and `MjModel.from_xml_string` give the same model. Pitfalls confirmed: MJCF `fullinertia` order is `ixx iyy izz ixy ixz iyz` while URDF is `ixx ixy ixz iyy iyz izz`; MJCF quaternions are scalar-first; MuJoCo diagonalises inertia into `body_inertia` plus `body_iquat`; MuJoCo rejects inertia that violates the triangle inequality (suggesting `balanceinertia`), so RigidForge must validate before export rather than rely on that flag. Position actuators with `forcerange` compile as expected.

## 7. Persistence (multi-session work)

**Layout.** A project directory with a JSON model file as the canonical store:

```
myrobot.rforge/
  project.json    # the model: links, joints, parts, materials, export targets
  assets/         # copied source STEP/meshes, imported xacro/URDF and their meshes
  cache/          # tessellation, BREP, collision hulls, mass props (regenerable, gitignored)
  workspace.json  # camera, panel layout, slider values (user-local, gitignored)
```

- Use pydantic `model_dump_json` with sorted keys, indentation and stable child order so diffs are readable in git. A zipped single-file `.rforge` bundle can be offered for sharing, but the directory is the working format. SQLite was rejected: it diffs badly and the model is small.
- **CAD assets are always copied** into `assets/`. Files are stored under their content hash or original name plus hash, so two imports of the same file are deduplicated.
- **Persist inputs, not derived data.** Persist part references, mass/density/material, manual inertia overrides, joint frames and limits, collision settings and export settings. Do not persist `TopoDS_Shape`, VTK actors, tessellations or computed inertia as authoritative values. Cache them under `cache/` keyed by a hash of geometry, scale, density, method and calc-code version, so a stale cache recomputes.
- **Identity.** Every link, joint and part has a stable UUID and is referenced by ID. Names are user-editable and exporters need unique names, so names are not keys.
- **Joint frames are numeric and authoritative**, in link coordinates (xyz + quaternion; RPY shown in the UI only). A snap reference (face and feature signature) is stored as a hint only, because B-Rep face indices are not stable across CAD re-exports.
- **Canonical units** are SI.
- **Simulator-specific data.** Core model plus namespaced extension blocks (`extensions.mjcf`, `.sdf`, `.usd`, `.urdf`), each its own pydantic model, for things like MJCF actuators and solver parameters, SDF friction and plugins, USD drive stiffness and physics-backend tuning. Per-target export config (output directory, mesh format, naming) is stored alongside.
- **Schema evolution.** `schema_version` in the file; a chain of dict-to-dict migrations runs before pydantic validation. Keep one fixture file per past version in tests. Newer-than-known files open read-only with a warning. Unknown extension keys round-trip rather than being dropped.
- **Save mechanics.** Atomic write (temp file, fsync, rename) with one rotating backup. Autosave to a recovery file beside the project and offer recovery after a crash. Undo is session-only (`QUndoStack` over model commands). A lock file (PID and host) warns about a second instance, and the on-disk hash is compared with the hash at load to catch outside edits.
- **Tests.** Round-trip property tests (save, load, equal); golden project files per schema version; exporting a loaded project gives byte-identical output to exporting the original in-memory model.

**Model shape (implemented 2026-11-16, `rigidforge.core.model`).** `RobotModel` holds flat, ordered lists of links, joints, parts and loop constraints, each with a UUID; references are by ID. A part points to its link (`None` while it is an unassigned import candidate), carries its source file, a placement in the link frame, an optional mirror plane in its own file coordinates, a mass source (`density` or `measured_mass`, a discriminated union) and an optional inertial override. A link may also carry an inertial override. Quaternions are scalar first (w, x, y, z). The pydantic layer checks structure only (types, vector lengths, unknown core fields rejected); semantic checks (single root, positive mass, unit quaternions, finite and ordered limits) belong to validation, so that a bad file loads and gets reported rather than failing to open. Extension blocks and the extension container allow extra keys so they round-trip.

**Validation (implemented 2026-11-16, `rigidforge.core.validation`).** `validate(model)` returns a `ValidationReport` of `Issue`s, each with a stable `IssueCode`, a severity (error blocks export, warning does not) and the offending item's UUID. Errors: duplicate IDs; empty names; duplicate names among links, among joints and among loop constraints (part names may repeat, since instances share a name); references to unknown links; tree structure (no links, self-joint, several parent joints for one link, no root, several roots, cycles found with Tarjan's SCC, unreachable links); revolute and prismatic joints without limits; non-finite or reversed limits; negative effort, velocity, damping or friction; zero or non-finite motion axes; non-finite poses and quaternions off unit length by more than 1e-6; non-positive or non-finite density, measured mass, override mass and `scale_to_si`; degenerate mirror planes; a loop constraint joining a link to itself. Warnings: position limits on fixed or continuous joints, and parts not assigned to a link. Inertia tensor checks (SPD, triangle inequality) are added with the mass-properties code in M1 task 4.

**Re-import of CAD objects (required).** Proposed behaviour, to confirm:

1. Each part keeps an import record: source filename, sha256, the file's path on disk when last imported (as a hint), units, tessellation settings, and the part's path inside the STEP assembly (name plus index).
2. "Re-import" lets the user pick a new file for a part, or "Refresh" re-reads the original path. The new file is copied into `assets/` and the old copy is kept until the project is saved, so the action is undoable.
3. Matching sub-parts of an assembly to existing links: first by XCAF part name and path, then by geometric signature (volume and bounding box within tolerance), then ask the user for anything left over.
4. Preserved across re-import: link and joint identity, joint frames (numeric, in link coordinates), limits, mass/density/material, and manual overrides.
5. Recomputed: tessellation, B-Rep inertia and CoM, collision hulls (flagged as stale).
6. After matching, show a change report per part: volume, CoM shift and bounding-box change. Warn when a joint frame now lies outside the part's bounding box (likely the part moved in CAD). Snap hints are re-resolved where possible; where they fail, keep the numeric frame and mark the hint as unresolved.
7. Parts removed from the new file are shown as orphans for the user to delete or remap, and new parts are offered as additions. Nothing is deleted silently.
8. Detect stale sources on project open by comparing hashes against the original path, and offer a re-import without forcing it.

## 8. Open items

- Pin the exact Isaac Sim version and confirm its USD layer conventions when starting the USD exporter.
- Confirm the xacro import semantics proposed in section 5 (one-way, copy meshes, keep provenance).
- Confirm the re-import matching and change-report rules proposed in section 7.

## Sources

- PyPI JSON metadata for each package listed (queried 2026-11-15)
- MuJoCo modeling docs and URDF→MJCF examples (mujoco.readthedocs.io; docs.picknik.ai)
- pyvistaqt usage docs (qt.pyvista.org) and qtpy `QT_API` behaviour
- trimesh API docs (`moment_inertia`, `moment_inertia_frame`, `density`)
- Open Robotics discourse and the `rod` project README on SDF/URDF conversion via `gz sdf`
- OCCT STEP user guide and forum threads on `STEPCAFControl_Reader`
- Isaac Sim documentation (docs.isaacsim.omniverse.nvidia.com: 6.0.0 release notes, 6.2.0 docs) and the NVIDIA developer forum thread on Isaac Sim Python versions
- ROS xacro changelog and Open Robotics discourse on xacro limits (`xacro.load_yaml`, `$(find pkg)`)
