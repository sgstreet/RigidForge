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
| cadquery-ocp 8.0.1.1.0 | ≥3.11, <3.15 |
| usd-core 26.11 | ≥3.11, <3.15 |
| mujoco 4.0.1 | ≥3.11 |
| vtk 9.7.2 | wheels cp310–cp315 |
| pyvista 0.49.2, pyvistaqt 0.13.2, trimesh 5.2.1, pydantic 2.14.3, coacd 1.0.14 | ≥3.9–3.10 |

Intersection: **3.12 – 3.14**. Recommend developing on 3.13 and testing 3.12 and 3.14 in CI. Linux x86_64/aarch64, macOS and Windows wheels exist for cadquery-ocp, usd-core and vtk. RigidForge only writes USD files and does not run inside Isaac Sim, so Isaac's own Python (3.12 for 6.x, 3.11 for 5.x) does not constrain this window.

## 5. Final stack

| Layer | Choice | Notes |
|---|---|---|
| GUI | PySide6 6.12 (LGPL) | Set `QT_API=pyside6` before importing pyvistaqt / qtpy, which otherwise defaults to PyQt5. Do not install other Qt bindings in the same environment. |
| Viewport | pyvista + pyvistaqt (`QtInteractor`) over VTK 9.7 | MIT / BSD. Drop to raw VTK only if pyvista's picking is limiting. |
| B-Rep / STEP | `cadquery-ocp` directly | Depend on OCP, not the full `cadquery` package. `cadquery.importers.importStep` uses `STEPControl_Reader` and drops colours and assembly metadata. Use `STEPCAFControl_Reader` with name, colour and material modes on an XCAF document to get the assembly tree. Check `xstep.cascade.unit` (OCCT converts to mm by default) **(verify)**. |
| Meshes | trimesh 5.2 (+`pycollada` for DAE; `trimesh[easy]` pulls lxml, networkx, manifold3d) | `moment_inertia` is about the CoM at the mesh's density; `moment_inertia_frame()` gives other frames. Results are meaningless unless the mesh is watertight, so check `is_watertight` first. |
| Collision | coacd 1.0 (MIT) | Run in a worker process, not a thread. |
| Math | numpy, scipy.spatial.transform | |
| Data model | pydantic v2 | Tree of Links / Joints plus loop constraints, plus per-simulator extension blocks (section 7). |
| URDF export | direct lxml serializer | yourdfpy 0.0.60 (MIT) or urchin for FK preview only. Last yourdfpy release was Jan 2026; urchin is releasing more recently. |
| URDF / xacro import | `xacro` 2.1.1 (pip) to expand, then a URDF parser into the internal model | See below. |
| MJCF | direct generation, validated by compiling with mujoco 4.0.1 (`MjSpec` or XML string) | Re-check the Python API against 4.0 release notes **(verify)**. |
| SDF | direct lxml serializer; `gz sdf -k` for optional validation | See correction 1. |
| USD | usd-core 26.11 (TOST licence), targeting the latest Isaac Sim | Build stages with `UsdPhysics` schemas. See below. |
| Packaging | uv + `pyproject.toml`, lockfile | Briefcase or PyInstaller later. conda-forge is the fallback if OCP wheels conflict with a system library. |

**URDF / xacro import (in scope).** Proposed semantics, to confirm: import is one-way. Run `xacro` to expand to URDF, parse the URDF into the internal model, and resolve `package://` and `file://` mesh URIs, copying the meshes into `assets/`. The model does not round-trip back to xacro. Keep the original xacro and its expanded URDF in the project as import provenance. Expose user-supplied xacro args and package search paths in the import dialog, because `$(find pkg)` and `xacro.load_yaml()` need files on disk and a package path. Fail with a clear message when they cannot be resolved. Imported URDFs often have joint and inertial data with no CAD source, so those links get meshes only, with manual mass and inertia and the source marked "imported".

**USD target: latest Isaac Sim.** As of 2026-11-15, Isaac Sim 6.x docs are published up to 6.2.0, and 6.0.0 moved to Kit 110 and Python 3.12. Sources report that 6.0 introduced "Asset Structure 3.0": USDC for geometry, with materials, instances, base and physics in separate USDA layers, and separate MuJoCo and PhysX definitions for tuning values. 6.x also supports multiple physics backends (PhysX and Newton). Design the USD exporter to emit the layered structure (geometry layer plus physics layer, with physics tuning kept in the `extensions.usd` block). Pin the target Isaac Sim version in `pyproject.toml` or a constants file and test exported assets by opening them in that version **(verify the exact GA version and layer conventions when implementing)**.

## 6. Design points the original chat missed

1. **Face-to-B-Rep mapping for picking.** VTK picks triangles, not B-Rep faces. Tessellate per face (`BRepMesh_IncrementalMesh`) and store a `face_id` cell-data array on the mesh. A pick then maps to a `TopoDS_Face`, and `BRepAdaptor_Surface` can return the cylinder axis or plane normal for joint snapping.
2. **Threading.** Run STEP load, tessellation and coacd off the UI thread. `TopoDS_Shape` is not picklable, so pass BREP bytes (`BRepTools.Write_s`) to a process pool and return numpy arrays plus mass properties. Only VTK/Qt calls run on the main thread.
3. **Units and mass provenance.** Store per part: unit-normalised geometry (m), mass source (explicit mass or density × volume), and the inertia method (B-Rep exact or mesh). Surface the source in the UI.
4. **Inertia validation.** For each link: mass > 0, tensor symmetric and positive definite, principal moments satisfy the triangle inequality (Ixx + Iyy ≥ Izz and permutations), meshes watertight. Warn on extreme aspect ratios of principal moments.
5. **Package layout.** Avoid a subpackage named `io` (confusing next to stdlib). Suggested: `core/` (model, validation), `importers/` (STEP, mesh, URDF/xacro), `physics/`, `ui/`, `viewport/`, `exporters/`, `tests/`. Exporters are pure functions `RobotModel -> files`.
6. **Test oracles.** Compare computed inertia against analytic box, cylinder and sphere. Round-trip exports: load MJCF in MuJoCo, load URDF in yourdfpy, open USD with `Usd.Stage.Open`, run `gz sdf -k` where available.

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
