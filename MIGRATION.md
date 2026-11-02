# In-Grasp Planner Migration: Python 3 + viser

This is the living plan for moving the relaxed-rigidity in-grasp planner off ROS Kinetic / Python 2. The goal
is to run it from Python 3 and drive interactive [viser](https://viser.studio) demos. Update it as work lands:
- tick checkboxes;
- update the status table;
- record decisions;
- add a changelog line.

- **Branch:** `dev/gepetto` is the integration branch. Use one feature branch per milestone (`dev/gepetto-m1-replay`, ...).
- **Owner:** joe liechty
- **Last updated:** 2026-11-02

---

## Status at a glance

| Milestone | Status | Estimate | Notes |
|---|---|---|---|
| M0 Bookkeeping | **done** (pending commit) | ½–1 d | Docs, `.gitignore`, repo-audit README, `in_grasp_py/` skeleton |
| M1 Replay viewer (pure Python) | not started | 1–2 d | Can run in parallel with M2 |
| M2 ROS-free C++ core | not started | 3–5 d | |
| M3 Python 3 bindings | not started | 2–3 d | |
| M4 Live viser demo | not started | 3–4 d | |
| M5 SNOPT + fidelity check | not started | 2–4 d | SNOPT 7.4-1.1 source available (`~/Downloads/snopt7.zip`), not built |
| M6 Polish | not started | 1–2 d | |

Rough total: 2.5–4 weeks for one developer.

---

## 1. Goal and scope

**Goal:** interactive, near-real-time viser demos of the in-grasp planner **as it exists today**:
1. Drag a goal object pose.
2. The planner solves.
3. The hand and object trajectory animate.

The method is purely kinematic: there's no physics engine anywhere in this repo, so a simulator isn't needed.

**In scope**
- The live relaxed-rigidity formulation:
  - [`in_grasp_manipulation/src/problems/in_grasp_rr_const.cpp`](in_grasp_manipulation/src/problems/in_grasp_rr_const.cpp)
  - its server logic in [`in_grasp_manipulation/src/in_grasp_server.cpp`](in_grasp_manipulation/src/in_grasp_server.cpp)
- The 2019 benchmark variants in `in_grasp_manipulation/src/problems/methods/` (`in_grasp_rr_cost.cpp`, `in_grasp_ik_rigid.cpp`, `in_grasp_point_contact.cpp`). These are needed to reproduce the benchmark numbers.
- A minimal Python 3 port of the data/metrics/benchmark scripts these need.

**Out of scope for now** (tracked under [Future work](#future-work)):
- ROS of any kind. Deferred; see [docs/ROS_MIGRATION_TODO.md](docs/ROS_MIGRATION_TODO.md).
- `finger_relocation`: doesn't compile at HEAD.
- `in_hand_controllers`: needs Allegro + BioTac hardware.
- `grasp_gaits/`, `old/`, and all other legacy Python 2 code.

---

## 2. Background (why this shape)

- **Repo state.** The repo is ROS Kinetic + Python 2 + C++11, with hardcoded `$HOME/local` / `$HOME/pkgs` paths and Bitbucket lab dependencies. It doesn't build on Ubuntu 22.04, which this box runs (ROS 2 Humble only).
- **The live planner is C++.** `in_grasp_server` builds an `opt_problems::optProblem` (relaxed rigidity) and solves it with **pagmo2 + SNOPT7** (via pagmo_plugins_nonfree).
  - Decision variables: 16 joints × T steps.
  - Cost: final pose error of the rigid finger (thumb) plus KOMO-style joint acceleration.
  - Inequality constraints: joint-velocity limits, and per-finger relaxed-rigidity position/orientation offsets.
- **ROS is only in the shell.** The method sources include just the problem header, which pulls in `ll4ma_opt_utils` (base class), `ll4ma_kdl` (KDL FK/Jacobians) and, optionally, `ll4ma_collision_wrapper`. ROS appears only in the server: param server, `.srv`, `tf`, and `*_msgs` types.
- **Speed is fine for interactive use.** In Bala's benchmark (`in_grasp_manipulation/scripts/in_grasp_bih`, which is a CSV), relaxed-rigidity solves had a median of ≈0.25 s. About 15% of runs stalled at ≈13 s (probably the iteration limit), so a solver time limit is needed for demos.

---

## 3. Target architecture

```
viser app (py3.11, in-process) ── import in_grasp ──► pybind11 extension (one per method variant)
                                                        ├─ planner shell  (ROS-free port of in_grasp_server.cpp)
                                                        ├─ method source, compiled UNMODIFIED from
                                                        │    in_grasp_manipulation/src/problems/in_grasp_rr_const.cpp
                                                        │    (+ methods/in_grasp_rr_cost.cpp, ik_rigid, point_contact)
                                                        ├─ vendored, ROS-stripped ll4ma pieces (same include paths)
                                                        └─ pagmo2 ──► SNOPT7 (loaded at runtime from a .so path)
                                                                  └─► IPOPT  (fallback, `solver="ipopt"`)
```

Key properties:
- **The method code is not edited.** The original `.cpp` files compile against vendored headers that keep their original include paths, so the math stays identical to Bala's.
- **One extension module per variant.** Every variant defines the same class `opt_problems::optProblem`, so they can't share one module. Planned modules: `_rr_const` (the live method), `_rr_cost` (used for the benchmark), and optionally `_ik_rigid` and `_point_contact`.
- **SNOPT loads at runtime.** pagmo_plugins_nonfree loads the SNOPT library when the program runs, so building never needs the license; you pass a library path when you want SNOPT. IPOPT lets everything run end-to-end before SNOPT is installed.
- **viser runs in-process.** No middleware is needed. Solves are serialized behind a lock, because SNOPT is not thread-safe.

### Planned layout

```
in_grasp_py/
  pyproject.toml          # scikit-build-core + pybind11; dist name "in-grasp", import name "in_grasp"
  environment.yml         # conda-forge env (see §5)
  CMakeLists.txt          # add_variant(<name> <method .cpp>) → one extension per variant
  cpp/
    third_party/ll4ma/    # vendored minimal copies, ROS removed, upstream commits recorded in a README
      ll4ma_kdl/manipulator_kdl/robot_kdl.{h,cpp}           # from ll4ma_kdl main@78fa833
      ll4ma_opt_utils/problems/opt_problem_base.{hpp,cpp}   # from ll4ma_opt_utils main@4ea5bfa
      ll4ma_opt_wrapper/{debug.h,pagmo_opt/wrapper.hpp}     # from ll4ma_opt_utils main@4ea5bfa
      kdl_parser/                                           # URDF→KDL tree builder (BSD) on urdfdom
    planner.{hpp,cpp}     # Config / Request / Result structs; port of in_grasp_call()
    bindings.cpp          # pybind11
    smoke_main.cpp        # C++ smoke test
  assets/                 # the ~13 Allegro STLs the URDF needs (copied, see §6)
  src/in_grasp/
    planner.py            # Python API: dataclasses, presets, two-stage plan helper, solve lock
    data.py               # goal-pose CSVs, stored reachable_poses trajectories
    metrics.py            # port of in_hand_manipulation_data/scripts/error_api.py
    bench.py              # port of in_grasp_manipulation/scripts/generate_in_grasp.py loop
    objects.py            # benchmark object name → YCB mesh
    assets.py             # URDF/mesh resolution (package:// → assets/), <klampt> strip, STL case fix
    viz/                  # viser app (scene, playback, GUI)
  tests/
```

The hand model is a single shared file: [`in_grasp_manipulation/urdf/allegro.urdf`](in_grasp_manipulation/urdf/allegro.urdf). The planner (KDL via urdfdom) and the viewer (yourdfpy) both load it.

---

## 4. Roadmap and milestones

Each milestone lists its tasks and the acceptance criteria that mark it done.

### M0 Bookkeeping (½–1 d)
- [x] Write this plan and [docs/ROS_MIGRATION_TODO.md](docs/ROS_MIGRATION_TODO.md).
- [x] Narrow the root `.gitignore`. The `*data` rule silently ignored every new file under `in_hand_manipulation_data/` and any `data/` directory; it has been removed. `*.p` and `*.eps` are kept, and Python tooling ignores were added.
- [x] Write the repo-audit README ([readme.md](readme.md)): what every package is, what builds, and what's legacy. `readme.org` is marked superseded.
- [x] Create `in_grasp_py/` skeleton: `pyproject.toml` (scikit-build-core), placeholder `CMakeLists.txt`, `environment.yml` (unresolved; env creation moved to M2), empty `in_grasp` package, one import test.

**Done when:** docs merged on `dev/gepetto`, `.gitignore` fixed, and the skeleton `pytest` passes. It passes under `gepetto_py10` with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, because a broken `dash` plugin in `~/.local` crashes pytest's plugin autoload. The `ihm` env is created in M2.

### M1 Replay viewer, pure Python, no C++ (1–2 d; parallel with M2)
- [ ] Add `assets.py`. It loads `allegro.urdf` with yourdfpy:
  - map `package://ll4ma_robots_description/` → `in_grasp_py/assets/`;
  - fix the `link_0.0.STL` case mismatch;
  - strip the `<klampt>` block (line ~733).
- [ ] Add `data.py`:
  - parse `in_grasp_manipulation/data/initial_data/goal_poses/*.csv` (columns listed under [Data notes](#data-notes));
  - read `in_grasp_manipulation/data/trajectories/reachable_poses/<method>/<Obj>_<k>_<idx>` files (space-separated, 16 joint angles per row).
- [ ] Add `objects.py`: map the 10 benchmark objects to YCB `google_16k` meshes (see §6), with a configurable root via the `IN_GRASP_YCB_ROOT` env var.
- [ ] Add a viser app in replay mode:
  - hand rendered in the palm frame;
  - object at the initial pose;
  - goal ghost;
  - goal dropdown;
  - playback slider and play/stop.
- [ ] Python FK object trajectory, using the planner's own rule: `obj(t) = thumb_tip(t) · (thumb_tip(0)⁻¹ · obj(0))`.

**Accept:** `python -m in_grasp.viz --replay` shows any of the 100 relaxed-rigidity goals and plays the stored trajectories. A pytest checks that the FK-derived object pose at t=0 equals the CSV initial pose.

### M2 ROS-free C++ core (3–5 d)
- [ ] Create a conda env from `environment.yml` (§5) and confirm every package resolves.
- [ ] Vendor and strip the ll4ma pieces:
  - Remove `ros/ros.h`, `tf`, `tf_conversions`, `eigen_conversions`, `geometry_msgs` and `urdf/model.h`.
  - Keep the file-based `robotKDL(file_name, base_names, EE_names, …)` constructor.
  - Replace `urdf::Model` and `kdl_parser` with urdfdom plus a vendored tree builder.
  - Compile with `LL4MA_COLLISION_MODEL=false`.
- [ ] Port the planner shell (`planner.{hpp,cpp}`) from `in_grasp_call()`:
  - `Config{urdf_path, base_frames, ee_frames, joint_names}`, defaulting to the values in `in_grasp_manipulation/config/robot/allegro.yaml`;
  - `Request` / `Result` structs mirroring `in_grasp_manipulation/srv/InGraspManipulation.srv`;
  - `solver ∈ {snopt, ipopt}`, `snopt_library`, `time_limit`, `seed`.
- [ ] CMake `add_variant()`: compile `planner.cpp` and one method `.cpp` (unmodified) into a library per variant. Define `SPARSE_GRADIENT` in both, as the originals do on line 1.
- [ ] IPOPT path: `pagmo::ipopt` with `hessian_approximation=limited-memory`, and the same c_tol of 1e-4.
- [ ] Shell fixes (behaviour-preserving for the finger sets used in practice):
  - [ ] Map the finger offset matrices by finger id. They are pushed in `fingers` order (`in_grasp_server.cpp:394-403`) but read as `data_mats_[f]` (`in_grasp_rr_const.cpp:368`). Today only finger sets whose non-rigid IDs are 0..k-1 work.
  - [ ] Validate requests:
    - `rigid_finger == 3`, since the thumb is hardcoded at several server sites;
    - `fingers` contains 3;
    - `weights.size() >= 5`, since `weights[4]` is read unchecked;
    - `T >= 1`.
  - [ ] Make the SNOPT library path a parameter. The hardcoded `pagmo::snopt7(false,"/usr/local/lib/")` (`in_grasp_server.cpp:461`) is a directory, and upstream ppnf wants the `.so` file.
  - [ ] Add a time limit (SNOPT `Time limit`, IPOPT `max_cpu_time`) for the ≈13 s outliers. Default it off in benchmark mode.
  - [ ] Fill `time_from_start` and `metrics`: solve time, objective, max constraint violation, solver status.
- [ ] `smoke_main`: plan goal #0 of `relaxed-rigidity.csv` with IPOPT.

**Accept:** `smoke_main` returns T+1 joint points with max constraint violation ≤ 1e-4, and a test greps the build's include closure and finds no ROS headers.

### M3 Python 3 bindings (2–3 d)
- [ ] pybind11 bindings exposing `Planner(config, variant="rr_const").plan(request) -> Result`, with NumPy arrays in and out.
- [ ] Python API in `planner.py`:
  - dataclasses;
  - weight presets from `generate_in_grasp.py:181-187` (`rr`, `rpo`, `rp`, `pc`, `ik`; order `[wo, wr, wd, wi, vel_lim]`; on `rr_cost`, `wi` means waypoint tracking);
  - `plan_two_stage()` mirroring the benchmark (T=1 IK-like solve, then T=10 seeded with `final_joints`);
  - a global solve lock.
- [ ] Tests:
  - request validation;
  - C++ FK == Python FK (1e-6);
  - `obj_traj` == thumb-FK rule;
  - identical results before and after the finger-index fix for `[0,1,3]` and `[0,1,2,3]`;
  - IPOPT solve on 5 goals succeeds.

**Accept:** `pip install -e in_grasp_py && pytest` passes in the conda env, and `from in_grasp import Planner` works in Python 3.11.

### M4 Live viser demo (3–4 d)
- [ ] Add a transform gizmo for the desired object pose, in the palm frame. Plan on drag-end, with an optional "live" mode that is debounced and latest-wins.
- [ ] Warm-start from the previous solution (`initial_jtraj`) when replanning.
- [ ] Animate the hand and object along the result, show the goal ghost, and optionally draw fingertip contact markers.
- [ ] GUI controls: variant, preset, the individual weights, T, solver, time limit, and finger set.
- [ ] Metrics panel: position/orientation error to goal, max rigidity violation, solve time, and solver status.
- [ ] Goal dropdown populated from the benchmark CSVs.
- [ ] Planning runs off the GUI thread with the button disabled while solving. Failures surface in the status line and never hang the UI.

**Accept:** typical replan plus animation in ~1–2 s with IPOPT, and the demo runs from one command, `python -m in_grasp.viz`.

### M5 SNOPT + fidelity check (2–4 d; SNOPT source in hand, not yet built)
SNOPT source: `/home/joe/Downloads/snopt7.zip` is **SNOPT 7.4-1.1** (Gill & Wong, Jan 2015; ChangeLog through 2015-12). It is a source distribution only (Fortran + `interfaces/` C/C++ API), with no prebuilt libraries and no license file. Bala's `ll4ma_opt_wrapper/install_snopt.sh` used the same flow:
1. build `snopt7` with `./configure --prefix=/usr/local && make install`;
2. build his `snopt-interface` fork (branch `ll4ma_pkgs`) for `libsnopt7_c`;
3. build his `pagmo_plugins_nonfree` fork.

**Never commit SNOPT sources or binaries to this repo** (licensed). Build into a local prefix outside the repo and point to it via config or an env var (e.g. `IN_GRASP_SNOPT_LIB`).

- [ ] Unpack into its own directory outside the repo (e.g. `~/pkgs/snopt7`).
- [ ] Build the Fortran library and the C interface with gfortran (≥4.4) into a local prefix (e.g. `~/.local/opt/snopt7`).
  - `INSTALL` says `./configure --with-c` (or `--with-c-cpp`), then `make && make interface && make install`. `interfaces/README.C` says `--with-api`; try `INSTALL`'s flags first.
  - Expected outputs: `libsnopt7.so` plus the C wrapper (`snopt_cwrap`, i.e. `snInit` / `setIntParameter` / `solveA` …).
- [ ] Check that the resulting C library matches the API pagmo_plugins_nonfree loads. I believe upstream ppnf's `snopt7` takes a SNOPT minor-version argument to select the pre-7.6 vs 7.6 C API, so pass 4 here; verify against the installed ppnf. If upstream ppnf can't drive 7.4, fall back to building `libsnopt7_c` from `snopt-interface` (upstream, or Bala's `balakumar-s/snopt-interface@ll4ma_pkgs`).
- [ ] Toy check: run the SNOPT `interfaces/cexamples/toyC` example, then a pagmo+ppnf toy problem, before wiring it into the planner.
- [ ] `solver="snopt"` end-to-end. Confirm option handling: upstream may ignore "Major iterations limit" set via `set_numeric_option`, so decide whether to keep it as-is for fidelity or switch to `set_integer_option`, and record the choice in the decision log.
- [ ] `bench.py`: run 100 goals × {rr, rpo, rp} with `_rr_cost`, plus `_ik_rigid` and `_point_contact` if built, and compare with `in_grasp_manipulation/scripts/in_grasp_bih`.
- [ ] Record the results table and timing distribution in this doc.

**Accept:** success rate and median P_err/O_err per method fall within the agreed tolerance of `in_grasp_bih`. Proposed tolerance: ±10% or ≤2 mm / ≤0.02 rad absolute, whichever is looser; confirm before running. With the demo time limit, the `_rr_const` median solve is ≤0.5 s.

### M6 Polish (1–2 d)
- [ ] README quickstart: create the env, build, and run the demo.
- [ ] All tests green, and this doc's [Known limitations](#known-limitations) is current.
- [ ] Tag `in-grasp-py-v0.1`.

### Future work
Tracked here; not scheduled.
- **F1 ROS Noetic node** wrapping the `in_grasp` core library, keeping `InGraspManipulation.srv`. Then **F2 Humble**. See [docs/ROS_MIGRATION_TODO.md](docs/ROS_MIGRATION_TODO.md).
- **F3 Finger relocation.** `finger_relocation/src/problem/finger_gait.cpp` has compile errors at HEAD (undeclared `dist_thresh`/`obj_pose_err`, a stray `)`), and its sparsity work is unfinished. Port it with the same shell/binding pattern once it's fixed.
- **F4 Collision checking.** It's a no-op today (`collision_constraints.cpp` stubs). One option is fpte's cuRobo sphere checker.
- **F5 Legacy cleanup.** Remove or archive `grasp_gaits/`, `old/` and `src/**/old/`, and dedupe the `in_grasp_manipulation/data/` copy (see [Data notes](#data-notes)).
- **F6 Hardware controllers** (`in_hand_controllers`): needs Allegro + BioTac, plus the TacNet force model, which hasn't been found.
- **F7 Theseus cross-check**: reimplement the formulation in Theseus to compare.

---

## 5. Dependencies (new stack)

**conda-forge** (to be confirmed in M2; build from source if a package is missing):

| Package | Purpose |
|---|---|
| python 3.11 | runtime |
| pagmo-devel (≥2.19, C++17) | optimizer framework. Bala used a fork of 2.10 (`balakumar-s/pagmo2`) |
| pagmo_plugins_nonfree | SNOPT7 wrapper (runtime `.so` load) |
| ipopt | fallback solver (pagmo must be built with IPOPT) |
| orocos-kdl, urdfdom, urdfdom_headers | kinematics + URDF parsing |
| eigen, boost, tbb | C++ deps |
| pybind11, scikit-build-core, cmake, ninja | build |

**pip:** viser (1.1.x is what `gepetto_py10` has), yourdfpy, trimesh, numpy, pandas, pytest.

**SNOPT7:** commercial and licensed. **SNOPT 7.4-1.1 source** is at `/home/joe/Downloads/snopt7.zip`. Building it needs gfortran (≥4.4) plus the C interface (see M5). It's not needed at build time of `in_grasp`; it's loaded at runtime. Keep it out of git.

Use a separate env (`ihm`) rather than `gepetto_py10`, so these conda-forge C++ packages stay out of the CUDA/Theseus env.

---

## 6. Reuse

Copy with attribution rather than importing fpte, to avoid coupling.

| What | From |
|---|---|
| URDF → viser loading | `fpte/web_vis/main.py` `load_visual_urdf` (≈l.155), `create_visualized_urdf` (≈l.166) |
| Trajectory playback | `fpte/web_vis/main.py` `update_trajectory_step` / `set_trajectory` / `play_trajectory` (≈l.1599–1684); `fpte/web_vis/ik_custom_mesh_state.py` `make_trajectory_state` |
| Slider sync guard + safe range | `fpte/web_vis/main.py` `set_trajectory_slider_value` / `configure_trajectory_slider` (≈l.583–599) |
| Status line | `fpte/web_vis/main.py` `set_status` (≈l.485) |
| Warm-up pattern | `fpte/web_vis/main.py` `warm_up_curobo` (≈l.1480) |
| Transform gizmo | `fpte/third_party/curobo/curobo/_src/util/viser_visualizer.py` `add_control_frame` (≈l.167–202) |
| URDF consistency tests | `fpte/web_vis/tests/test_urdf_visualization_consistency.py` |
| Allegro meshes | `fpte/curobo_content/robot_assets/ll4ma_robots_description/meshes/allegro/` (copy the ones `allegro.urdf` references into `in_grasp_py/assets/`) |
| YCB object meshes | `gepetto_solvers/python/gepetto_solvers/core/objects/ycb/ycb_data/google_16k/` |

**Benchmark object → YCB map** (to verify visually in M1):

| Benchmark object | YCB mesh |
|---|---|
| Apple | 013_apple |
| Banana | 011_banana |
| Jello | 009_gelatin_box |
| Lego | 073-*_lego_duplo |
| Fork | 030_fork |
| Orange | 017_orange |
| Plane | 072-*_toy_airplane |
| Screwdriver | 043/044_*_screwdriver |
| Spatula | 033_spatula |
| Tuna | 007_tuna_fish_can |

**Python 3 ports.** Only the minimal set gets ported:
- the needed subset of `in_grasp_manipulation/python/object_list.py`;
- `in_hand_manipulation_data/scripts/error_api.py`;
- the loop in `in_grasp_manipulation/scripts/generate_in_grasp.py`.

Everything else (≈200 Python 2 files) stays untouched as legacy.

---

## 7. Data notes

- **Goal-pose CSVs:** `in_grasp_manipulation/data/initial_data/goal_poses/{relaxed-rigidity,relaxed-position,relaxed-position-orientation,trinkle}.csv`. Each has 100 rows (10 objects × 10).
  - Columns: `0` name, `1` unused "x,y,z" offset, `2` 16 grasp joint angles, `3` fingers (e.g. `0,1,3`), `4–9` initial object pose (xyz rpy, palm frame), `10–15` desired pose, `16–21` probably a previously reached pose (unverified).
  - `random_relaxed-rigidity.csv` and `2f_4f_relaxed-rigidity.csv` use a different 17-column layout.
- **Duplicate tree.** `in_grasp_manipulation/data/` (1,512 files) duplicates `in_hand_manipulation_data/data/in_grasp_manipulation/`: 1,510 files are identical in content and 2 are extra. The port treats **`in_grasp_manipulation/data/`** as canonical.
- **Missing upstream LFS blobs:** `relaxed_rigidity*.p` (in both data trees) and `in_grasp_manipulation/python/rss_data.hdf5` were never uploaded. They're not needed on this path.
- **Pickles** (`*.p`) are Python 2 protocol-0 pickles of ROS message objects. They're not used by the port; if ever needed, convert them once with ROS message classes available.

---

## 8. Risks and mitigations

| Risk | Mitigation |
|---|---|
| API drift from Bala's pagmo2 v2.10 fork to upstream 2.19 in `wrapper.hpp` (`con_optimization`) | Compile in M2 week 1; add small shims in the vendored copy |
| Upstream pagmo_plugins_nonfree differs from Bala's fork (`snopt7` ctor path semantics, integer vs numeric options) | Path becomes a parameter; decide option handling in M5 and log it |
| SNOPT **7.4** (pre-7.6 C API) incompatible with upstream ppnf's C interface | Check the ppnf minor-version / API option early; fallback: build `libsnopt7_c` via `snopt-interface` (Bala's `ll4ma_pkgs` fork matched this setup) |
| Licensed SNOPT source accidentally committed | Build outside the repo; reference by path / env var only |
| IPOPT gives different solutions than SNOPT | IPOPT is for bring-up only; fidelity is judged with SNOPT in M5 |
| conda-forge lacks a package (ppnf, orocos-kdl, urdfdom) | Build that one from source inside the env |
| YCB mesh origin ≠ Bala's object frame (ArUco/centroid based) | Visual offset only; calibrate with `in_grasp_manipulation/data/initial_data/object_grasps/object_centroids.csv` or accept |
| SNOPT not thread-safe | Single global solve lock |
| ≈15% of solves hit ≈13 s | Demo time limit; warm start |

---

## 9. Known limitations

These come from the method and stay unless they're explicitly changed.
- The thumb is the only supported rigid finger.
- Collision avoidance is not active (stubs).
- Only the y-component of the orientation offset is constrained (`in_grasp_rr_const.cpp:386-393`).
- The object pose is inferred from thumb FK × a fixed initial offset, not tracked.

---

## 10. Decision log

| Date | Decision | Why |
|---|---|---|
| 2026-10-27 | `dev/gepetto` = `regrasp_refactor_dev` + `sys_id_dev` (+ no-op `master` merge) is the integration branch | Newest code was split across branches; master was stale (2019-04) |
| 2026-11-02 | Scope = in-grasp relaxed-rigidity planner only | Only working, hardware-free method; finger relocation is broken, controllers need hardware |
| 2026-11-02 | Drop ROS for now; Noetic→Humble deferred to [docs/ROS_MIGRATION_TODO.md](docs/ROS_MIGRATION_TODO.md) | Focus on Python 3 + viser; long-term target is Humble anyway |
| 2026-11-02 | Keep Bala's C++ method + pagmo; strip the ROS shell; pybind11 to Python 3 | Exact same math and solver, so his benchmark numbers can be reproduced |
| 2026-11-02 | SNOPT stays primary (runtime-loaded); IPOPT fallback via a `solver` param | License available but not installed yet; unblocks bring-up |
| 2026-11-02 | Use the provided SNOPT 7.4-1.1 source; build it outside the repo into a local prefix; never commit it | Licensed code; same SNOPT generation Bala's install flow used |
| 2026-11-02 | The user makes all git commits; work pauses at commit points | User preference |
| 2026-11-02 | viser in-process (no middleware) | Purely kinematic, single process, simplest |
| 2026-11-02 | Vendor ROS-stripped minimal copies of `ll4ma_kdl` / `ll4ma_opt_utils` pieces, recording upstream commits | Their catkin packaging can't be used without ROS; only ~5 files needed |
| 2026-11-02 | Method `.cpp` files compiled unmodified from their original location | Single source of truth; fidelity |

---

## 11. Open questions

- [x] Which SNOPT version, and is a C interface included? **7.4-1.1 source** (`~/Downloads/snopt7.zip`), with a C/C++ interface in `interfaces/`. Still to confirm that it drives upstream ppnf (M5).
- [ ] Fidelity tolerance for M5: is the proposed ±10% / 2 mm / 0.02 rad acceptable?
- [ ] Should the demo default to `_rr_const` (live method) or `_rr_cost` (what produced the published benchmark)?
- [ ] Vendor the Allegro meshes into `in_grasp_py/assets/` (≈ a few MB), or resolve them from an `ll4ma_robots_description` checkout?

---

## 12. Changelog

- 2026-11-02: Plan created (M0). ROS deferred; architecture, milestones, risks and decisions recorded.
- 2026-11-02: SNOPT source identified (7.4-1.1, `~/Downloads/snopt7.zip`); M5 build steps and the related risk added; open question closed.
- 2026-11-02: M0 complete: `.gitignore` `*data` rule removed, repo-audit `readme.md` written, `in_grasp_py/` skeleton added (1 test passing).
