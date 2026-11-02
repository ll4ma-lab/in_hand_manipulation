# In-Hand Manipulation

Motion planning and feedback control for in-hand manipulation with an Allegro hand and BioTac tactile sensors.

> **Migration in progress (branch `dev/gepetto`).** The in-grasp planner is being ported to Python 3 + viser without ROS. See [MIGRATION.md](MIGRATION.md) for the roadmap and status, and [docs/ROS_MIGRATION_TODO.md](docs/ROS_MIGRATION_TODO.md) for the deferred ROS work. The new code lives in [in_grasp_py/](in_grasp_py/).

*Original author's note:* "This package contains research code that I am working on as part of my PhD program at the University of Utah. Visit my website for more information: <https://balakumar-s.github.io>" (Balakumar Sundaralingam, LL4MA Lab, University of Utah)

This README is a repo audit written in Nov 2026 for someone who has never opened the repo and wants to understand it, run it or build on it.
- "Builds" means it is enabled in the catkin build and its CMake targets exist. Nothing here has actually been compiled on a modern machine.
- Everything marked *(inferred)* was read from the code but not executed.

## Contents

1. [At a glance](#1-at-a-glance)
2. [Research threads → packages](#2-research-threads--packages)
3. [Repository map](#3-repository-map)
4. [Branches and history](#4-branches-and-history)
5. [Package guides](#5-package-guides)
6. [External dependencies](#6-external-dependencies)
7. [Building (original ROS Kinetic workflow)](#7-building-original-ros-kinetic-workflow)
8. [Running (original workflows)](#8-running-original-workflows)
9. [Data](#9-data)
10. [Modifying and extending](#10-modifying-and-extending)
11. [Gotchas](#11-gotchas)
12. [Glossary](#12-glossary)

---

## 1. At a glance

| | |
|---|---|
| **What** | Research code: in-grasp object reposing (relaxed rigidity), finger gaiting / regrasp planning, and tactile controllers (slip, friction estimation, grasp stabilization) |
| **Robot** | Right **Allegro hand** (16 DOF) with 4 **BioTac** fingertip sensors, mounted on a KUKA **LBR4** arm (7 DOF) |
| **Stack** | ROS 1 **Kinetic** (Ubuntu 16.04), catkin, **Python 2.7**, **C++11**, KDL, pagmo2 + **SNOPT7** (commercial solver) |
| **Physics** | **None.** The planners are purely kinematic; physics only enters through the real robot and sensors |
| **Size** | 12 catkin packages, ~3,750 tracked files (mostly data), ~230 Python files, 227 commits (2015-12 → 2020-03) |
| **Authors** | Balakumar Sundaralingam (almost all commits), Tucker Hermans (advisor) |
| **State** | Not maintained since 2020. Only parts build; much is legacy or half-refactored. The **in-grasp planner** is the most complete and is the migration target |

**Finger and joint convention** (used everywhere):

| Finger index | Finger | Joint indices |
|---|---|---|
| 0 | index | 0–3 |
| 1 | middle | 4–7 |
| 2 | ring | 8–11 |
| 3 | thumb | 12–15 |

Joint names are `{index,middle,ring,thumb}_joint_{0..3}`. With the arm, the 23-joint vector is `lbr4_j0..6` followed by the 16 hand joints.

---

## 2. Research threads → packages

| Thread | What it does | Packages | Related publications (as far as we can tell) |
|---|---|---|---|
| **In-grasp manipulation** | Repose a grasped object relative to the palm by moving the fingers *without* breaking or making contact. Kinematic trajectory optimization with "relaxed rigidity" constraints | `in_grasp_manipulation`, `in_hand_manipulation_data` | Sundaralingam & Hermans, *Relaxed-Rigidity Constraints: In-Grasp Manipulation using Purely Kinematic Trajectory Optimization* (RSS 2017) and its journal extension; 2019 benchmarking work (`sys_id_dev`; "bih" ≈ *Benchmarking In-Hand manipulation*) |
| **Finger gaiting / regrasping** | Relocate one fingertip at a time on the object surface, alternated with in-grasp moves, to reach a new grasp | `grasp_gaits` (legacy, original), `finger_relocation` + `in_hand_collision_lib` (2019 refactor, unfinished), `grasp_stabilization` (stub) | *Geometric In-Hand Regrasp Planning: Alternating Optimization of Finger Gaits and In-Grasp Manipulation* (ICRA 2018); see [dev_notes.org](dev_notes.org) |
| **Tactile control** | BioTac-based controllers: slip-reactive grasping, friction-coefficient estimation by pushing until slip, impedance grasp stabilization; surface-normal estimation | `in_hand_controllers`, `in_hand_utils` | 2019–2020 friction / grasp-stabilization experiments |
| **Infrastructure** | Messages, early pipelines, archived code | `in_hand_manipulation_msgs`, `in_hand_manipulation`, `in_hand_manipulation_pipeline`, `old/` | — |

---

## 3. Repository map

| Package | Thread | Catkin build today | State | Start here |
|---|---|---|---|---|
| [in_grasp_manipulation](in_grasp_manipulation/) | in-grasp | ❌ `CATKIN_IGNORE` | Most complete planner; C++ service + older Python planner + experiment scripts | `src/in_grasp_server.cpp`, `src/problems/in_grasp_rr_const.cpp` |
| [in_hand_manipulation_data](in_hand_manipulation_data/) | in-grasp | ✅ (no targets) | Data store + error/plot scripts | `scripts/plot_in_grasp.py`, `scripts/error_api.py` |
| [in_hand_controllers](in_hand_controllers/) | tactile | ✅ 3 nodes | Works with hardware (friction, impedance grasp, eigengrasp hand services); much dead code | `src/external_controller/`, `scripts/demo_friction.py` |
| [in_hand_utils](in_hand_utils/) | tactile | ✅ 2 nodes | Surface-normal nodes + joint-state sim | `src/surface_normal_node.cpp` |
| [finger_relocation](finger_relocation/) | regrasp | ✅ enabled, **but doesn't compile** | Half-finished Nov 2019 refactor | `src/finger_server.cpp`, `src/problem/finger_gait.cpp` |
| [in_hand_collision_lib](in_hand_collision_lib/) | regrasp | ✅ lib + demo | Hand/object collision queries on `ll4ma_opt_utils::envUtils`; several methods stubbed | `include/in_hand_collision_lib/scene/hand_env.h` |
| [grasp_stabilization](grasp_stabilization/) | regrasp | ✅ (no targets) | **Stub**: comments only | — |
| [in_hand_manipulation](in_hand_manipulation/) | infra | ✅ (no targets) | Builds nothing, but its CMake still requires libccd 2.0 + Pagmo. Legacy headers and Python 2 controllers | `launch/description.launch` |
| [in_hand_manipulation_msgs](in_hand_manipulation_msgs/) | infra | ✅ | Shared msg/srv; most are unused | `srv/`, `msg/` |
| [in_hand_manipulation_pipeline](in_hand_manipulation_pipeline/) | infra | ✅ (srv only) | Early (2017) grasp pipeline around `allegro_driver` | `scripts/grasp_server.py` |
| [grasp_gaits](grasp_gaits/) | regrasp | ❌ `CATKIN_IGNORE` | ICRA 2018 regrasp pipeline (Python + C++). Unbuildable as-is; the only copy of the full alternating planner | `python/automated_gait_planner.py` |
| [old/](old/) | — | ❌ `CATKIN_IGNORE` | Archived `grasp_transitions`, `hand_task_controllers`. Safe to ignore | — |
| [in_grasp_py/](in_grasp_py/) | migration | (not catkin) | New ROS-free Python 3 package (skeleton) | [MIGRATION.md](MIGRATION.md) |

Other top-level files:
- [docs/Doxyfile](docs/Doxyfile): Doxygen config.
- [dev_notes.org](dev_notes.org): the author's regrasp refactor TODO.
- `readme.org`: the original short readme, now superseded by this file.

---

## 4. Branches and history

| Branch | Last commit | Contents |
|---|---|---|
| **`dev/gepetto`** (local; work here) | 2026 | `regrasp_refactor_dev` + `sys_id_dev` merged (+ a no-op `master` merge), so it contains **every** remote branch |
| `origin/regrasp_refactor_dev` | 2020-03-22 | Newest upstream: regrasp refactor onto `plan_env_utils`, friction experiments, surface normals |
| `origin/sys_id_dev` | 2019-07-23 | In-grasp benchmarking: IK-rigid / point-contact baselines, RR-cost variant, ~1,500 data files. Never merged upstream |
| `origin/fcl_dev` | 2019-05-11 | Tactile slip controller; contained in both of the above |
| `origin/master` | 2019-04-08 | **Stale**; don't use |

History highlights:
- **2015–2017:** in-grasp planner (Python, pyOpt/SNOPT) and RSS experiments.
- **2017-10:** `grasp_gaits` and `in_grasp_manipulation` merged in as subtrees.
- **2018:** C++ in-grasp planner, relaxed rigidity as inequality constraints, IROS fan challenge.
- **2019:** collision library rework, tactile controllers, benchmarking.
- **2019-11 → 2020-03:** regrasp refactor (unfinished) and friction experiments.

---

## 5. Package guides

### 5.1 `in_grasp_manipulation`: relaxed-rigidity in-grasp planner

**What it does:** given a grasp (16 joint angles), the current object pose and a desired object pose (both in `palm_link`), it plans a joint trajectory for the active fingers. Over the trajectory, the rigid finger (always the thumb) carries the object to the goal, while the other fingertips keep roughly the same offset from the thumb. The object pose along the trajectory is *inferred* as `thumb FK × fixed initial thumb→object offset`; it is not simulated.

**The live C++ planner**
- Formulation: [`src/problems/in_grasp_rr_const.cpp`](in_grasp_manipulation/src/problems/in_grasp_rr_const.cpp) implements class `opt_problems::optProblem`.
  - Decision variables: `16 × T` joint angles, bounded by URDF limits.
  - Cost: `wd·‖rigid-finger tip pose error at the final step‖²` (rotation weighted 0.1) plus `wi·‖finite-difference joint acceleration‖²`.
  - Inequality constraints:
    - per-step joint velocity `≤ vel_lim`;
    - per non-rigid finger and step, `wr·‖position offset error‖² ≤ 1e-4`;
    - `wo·(orientation error, y component only)² ≤ 1e-4`.
  - The "relaxation" comes from that tolerance: the allowed position drift is `√(1e-4 / wr)`, e.g. ≈4.5 cm at `wr=0.05`.
  - Analytic, sparse gradients from KDL Jacobians.
- Server: [`src/in_grasp_server.cpp`](in_grasp_manipulation/src/in_grasp_server.cpp) builds the problem and solves it with **pagmo2 + SNOPT7** (`pagmo_plugins_nonfree`).
  - Node: `in_grasp_planner_node`.
  - Service: **`/in_grasp_planner/get_plan`** (`in_grasp_manipulation/InGraspManipulation`).
  - Parameters: read under the node namespace from [`config/robot/allegro.yaml`](in_grasp_manipulation/config/robot/allegro.yaml) (`urdf_param`, `base_frames`, `ee_frames`, `joint_names`, `hand_coll_links`). The URDF comes from `/robot_description`.
- Service request ([`srv/InGraspManipulation.srv`](in_grasp_manipulation/srv/InGraspManipulation.srv)):
  - `init_joints`, `object_pose`, `desired_object_pose`
  - `weights = [wo, wr, wd, wi, vel_lim]`
  - `fingers`, `rigid_finger` (must be 3)
  - `T`
  - `initialize_opt`, `final_joints` (seed by interpolation), `initial_jtraj` (seed trajectory)
- Service response: `joint_traj` (T+1 points), `obj_traj` (PoseArray in `palm_link`). `n_js` and `metrics` are never filled.
- Weight presets used in practice ([`scripts/generate_in_grasp.py`](in_grasp_manipulation/scripts/generate_in_grasp.py), lines 181–187):
  - `rr = [0.001, 0.05, 1.0, 0.01, 0.4]`
  - `rpo = [0.001, 0.05, 1.0, 0, 0.4]`
  - `rp = [0, 0.05, 1, 0, 0.4]`
  - The benchmark calls the service twice: first `T=1` (≈IK), then `T=10` seeded with the first result.
- Speed (from the benchmark results CSV [`scripts/in_grasp_bih`](in_grasp_manipulation/scripts/in_grasp_bih)): relaxed-rigidity median ≈0.25 s, but about 15% of runs take ≈13 s.

**Alternative formulations.** Every file below defines the **same class**. You choose one by changing the source file in `add_library(in_grasp_rr …)` in [`CMakeLists.txt`](in_grasp_manipulation/CMakeLists.txt) and rebuilding.

| File | Status | Notes |
|---|---|---|
| `src/problems/in_grasp_rr_const.cpp` | **live** | as above |
| `src/problems/in_grasp_sysid.cpp` | built, unused | adds per-axis pose weights `weights[5..10]` |
| `src/problems/methods/in_grasp_rr_cost.cpp` | benchmark | relaxed rigidity moved into the cost; `weights[3]` = waypoint tracking |
| `src/problems/methods/in_grasp_ik_rigid.cpp` | benchmark | full 6-D rigidity penalty |
| `src/problems/methods/in_grasp_point_contact.cpp` | benchmark | ik_rigid without orientation terms |
| `src/problems/old/*`, `src/old/*` | dead | won't compile (missing headers); includes `_bck` / `_back` / `_old` copies |

**Build:**
- `CATKIN_IGNORE` is present. `-std=c++11`.
- `ENABLE_COLLISION_LIB` defaults to ON, but the collision constraints are stubs; turn it OFF.
- Hardcoded paths: `$HOME/pkgs/pagmo_plugins_nonfree/include`, `$HOME/local` (fcl/ccd), and the SNOPT library directory `"/usr/local/lib/"` (server line 461).
- PCL is required but unused.

**Launch files:**
- `launch/in_grasp.launch`: loads params and starts the server plus `trajectory_smoothing` (that repo is missing). It does not load the URDF.
- `allegro_traj_rviz.launch` / `allegro_sim.launch`: load `urdf/allegro.urdf` as `robot_description`, plus state publishers and rviz. These use `state_publisher`, which is Kinetic-only.

**Python 2 code:**
- `scripts/` holds clients and experiments:
  - `in_grasp_client.py`;
  - `generate_in_grasp.py`: the benchmark;
  - `demo_in_grasp.py`: real robot;
  - `in_grasp_pipeline.py`: 2017–18 experiments with MPC, ArUco/DBOT tracking and rosbag recording.
- `python/` holds the older offline planner and experiments:
  - `main_tjo_*.py` with pyOpt/SNOPT;
  - iLQR (`ilqr*.py`) and CMA-ES;
  - rosbag post-processing;
  - plotting.
- The Python weight order differs from the C++ one.
- Both directories have many hardcoded `/home/bala/…` and `/media/bala/…` paths.

**Known bugs:**
- Finger offsets are pushed in `fingers` order but read by finger id. Only finger sets whose non-rigid ids are 0..k-1 work, e.g. `[0,1,3]` and `[0,1,2,3]`; `[1,3]` does not.
- The thumb is hardcoded as the rigid finger.
- `weights[4]` is read without a bounds check.

### 5.2 `in_hand_manipulation_data`
- A data store (see [§9](#9-data)) plus analysis helpers:
  - [`scripts/error_api.py`](in_hand_manipulation_data/scripts/error_api.py): position and quaternion error;
  - `scripts/data_api.py`: pandas wrapper;
  - `scripts/plot_in_grasp.py`: benchmark plots.
- `data/grasps/mustard_0.json` is used by the regrasp demos.
- The C++ in `src/database/` (PostgreSQL logging) is not built. It hardcodes credentials and uses the obsolete `database_interface`.

### 5.3 `in_hand_controllers`: tactile controllers (hardware)
Each controller is a C++ node that subclasses `hand_services`' `inHandController` and publishes joint *position* commands on its own `…/joint_cmd` topic. A hand-services server relays these to the robot once a client sends `control_mode 5` (connect external controller).

| Executable | Purpose | Key topics / services |
|---|---|---|
| `hand_services_node` | Eigengrasp extension of the hand-services server. Modes: 0 joint position, 1 trajectory, 2 grasp, 3 approach-until-contact, 4 stiffen, 5/6 connect/disconnect external controller, 7 BioTac tare, 8 force servo | `/allegro/hand_service`; `/allegro_hand_right/joint_cmd`; `/biotac/force_data` |
| `tactile_friction_estimator` | Push one finger tangentially until kinematic slip; return the BioTac force at slip (→ friction coefficient μ) | srv `/tactile_friction_estimation/get_estimate` (`TactileFrictionEstimation`); `/TacNet/bt_force`, `/biotac_tare_pub` |
| `tactile_impedance_grasp_node` | Grasp stabilization: keeps fingertip forces inside the friction cone with a "leaky integrator" force law | srv `/tactile_impedance_control/update_params`; `/TacNet/bt_force`, `/in_hand_utils/surface_normal` |

- Config lives in `config/*.yaml`: topics, gains, eigengrasps, thresholds.
- Launch files:
  - `tactile_friction_estimation.launch`, `tactile_impedance_grasp.launch`, `eigen_hand_services.launch`.
  - `eigen_grasp_stable.launch` and `in_grasp_mpc.launch` are **broken**: their targets aren't built or their packages are missing.
- Experiment scripts (Python 2, run with `python2` from `scripts/`):
  - `demo_friction.py`: the main friction experiment. Interactive; writes `friction_exp_final.p`.
  - `demo_impedance_grasp.py`.
  - `plot_friction.py`: plots the committed `friction_exp.p`.
  - `read_friction_data.py`: needs `friction_exp_final_v2.p`, which is not in the repo.
- Dead code:
  - the slip controller (`tactile_grasp_stabilization.cpp`) is commented out in CMake;
  - `relaxed_rigidity.cpp`, `src/in_grasp/*` (MPC), `src/finger_gait/*` and `src/grasp/*` include headers that no longer exist.
- Bug: `tactile_impedance_grasp.cpp:219,221` calls `setIdentity()` on vectors, so only finger 0 is initialised. `setOnes()` was probably intended.

### 5.4 `in_hand_utils`
- `surface_normal_node`: object surface normals at the BioTac contacts, from TF object pose plus collision geometry.
- `point_surface_normal_node`: normal at a detected point (`/biotac_detector/point`); prints a friction-angle estimate.
- Both publish `visualization_msgs/MarkerArray` on `/in_hand_utils/surface_normal`.
- `scripts/joint_state_sim_pub.py`: a tiny "simulator" that serves `/in_hand_utils/update_robot_state` and republishes joint states on `/lbr4_allegro/joint_states`.
- Both nodes call `ros::init` with a "/" in the name, so start them via their launch files, not `rosrun`.

### 5.5 `finger_relocation` and `in_hand_collision_lib`: regrasp refactor (unfinished)
**finger_relocation:**
- `fg_server` (`src/finger_server.cpp`) serves `/in_hand/finger_gait/plan` (`RelocateFingerKin`). It plans one finger's trajectory to a desired contact point with pagmo+SNOPT, using collision distance constraints.
- **It does not compile at HEAD.** `src/problem/finger_gait.cpp` has:
  - undeclared `dist_thresh` (the member is `dist_thresh_`) and `obj_pose_err`;
  - a stray `)`;
  - a local variable named `coll_data` that shadows the type.
- Logic gaps even after those fixes:
  - constraint gradients are empty;
  - arm/hand dof indexing overflows;
  - `time_from_start` is 0.
- The last hardware-tested version is commit `15c9c78` (2018-04-20), which used the old FCL-based API.

**in_hand_collision_lib:**
- `hand_env` wraps `ll4ma_opt_utils::envUtils` for per-finger and fingertip-vs-object distance queries. About 8 methods are stubbed.
- `demo_allegro_contact` is a collision demo (`launch/allegro_demo.launch`).

### 5.6 `grasp_gaits` (legacy, ICRA 2018) and `old/`
**grasp_gaits:**
- The original regrasp planner. It alternates finger relocation (contact ellipses fitted by PCA, signed-distance object relocation, a stability heuristic) with in-grasp moves.
- Code:
  - C++ per-finger trajectory optimizers;
  - ~74 Python 2 files: services, pipeline drivers `python/automated_gait_planner.py` / `main_gait_planner_kinematic.py`, the IROS fan-challenge scripts in `scripts/`, and vendored "splocs" geometry code;
  - its own copies of 17 msg/srv definitions.
- **Unbuildable:** all targets are commented out, scene sources are missing (recoverable with `git show 19df2bd^:in_hand_collision_lib/src/scene/allegro_env.cpp`), and nothing generates its messages.
- It is the *only* implementation of the full alternating regrasp pipeline, the object-relocation and ellipse services, and the fingertip grasp planner.
- About 63 MB, mostly meshes and pickles.

**old/:** archived 2018 packages; nothing references them.

### 5.7 `in_hand_manipulation`, `in_hand_manipulation_msgs`, `in_hand_manipulation_pipeline`
**`in_hand_manipulation`:**
- Builds nothing, but its CMake *requires* libccd 2.0 (from `$HOME/local`) and Pagmo, so configuring fails without them.
- Exports legacy headers (`pagmo_opt/trajectory_optimization.hpp`).
- Holds 2017–18 Python 2 controllers (impedance, pressure, grasp services) that import a never-generated `in_hand_manipulation.srv`.
- `launch/description.launch` loads the LBR4+Allegro description and rviz.

**`in_hand_manipulation_msgs`:**
- Builds with standard deps.
- In use: `RelocateFingerKin`, `UpdateObj`, `UpdateRobotConfig`, `SurfaceNormal`, `GetHandIK`.
- Unused: Ellipse, Grasp, GaitNode/GaitPlan, GetEllipse, RelocateFinger, MoveObj, GetGaitPlan, RelocateObj(Sd), ComputeGD(Raw), and more.
- `computeGD.srv` and `ComputeGD.srv` differ only by case, which breaks on case-insensitive filesystems.

**`in_hand_manipulation_pipeline`:** a 2017 preshape/grasp service wrapper around `allegro_driver`. It is not an "experiment evaluation" package, despite what the old readme said.

---

## 6. External dependencies

**Lab packages** are in the Bitbucket `robot-learning` workspace. Accessibility was checked 2026-10-30; recommended refs and per-repo state are in [docs/ROS_MIGRATION_TODO.md](docs/ROS_MIGRATION_TODO.md).

| Package | Used by | Available? |
|---|---|---|
| `ll4ma_opt_utils` (also contains `ll4ma_opt_wrapper`) | in-grasp, finger_relocation, collision lib, in_hand_utils | ✅ |
| `ll4ma_kdl` | all planners and controllers | ✅ |
| `ll4ma_collision_wrapper` (KrisLibrary/libccd/FCL) | collision (optional for in-grasp) | ✅ |
| `ll4ma_robots_description` | URDF/xacro and meshes | ✅ |
| `ll4ma_planner`, `ll4ma_scene_description` | configs, legacy | ✅ |
| `hand_services`, `allegro_driver`, `biotac_sensors`, `grasp_utils` | controllers / hardware | ✅ |
| `trajectory_smoothing` | launch files, scripts | ❌ not found (replace with toppra/ruckig) |
| `urlg_robots_description` | old URDFs | ❌ renamed to `ll4ma_robots_description` |
| `optimization_pkg` | legacy only | ❌ |
| TacNet (`/TacNet/bt_force`) | tactile controllers | ❌ not found |

**Solvers and libraries:**
- **pagmo2:** Bala's fork, `github.com/balakumar-s/pagmo2`, ≈v2.10.
- **pagmo_plugins_nonfree:** also a fork.
- **SNOPT 7:** commercial and licensed; obtain it from the lab. It is built from source together with `snopt-interface` (fork, branch `ll4ma_pkgs`).
- **fcl** and **libccd 2.0** (exact version) under `$HOME/local`.
- Eigen3, orocos KDL, `trac_ik_lib`, PCL 1.7, Boost.
- Legacy only: OMPL, NLopt, KrisLibrary.

**Python 2 libraries:** numpy, scipy, pandas, matplotlib/seaborn, trimesh, h5py, `pykdl_utils` / PyKDL / `urdf_parser_py`, tf, rospkg.
- Legacy only: pyOpt (SNOPT), pathos, mayavi.

**External nodes:** DBOT particle tracker (`/particle_tracker/object_state`), ArUco markers, TacNet.

---

## 7. Building (original ROS Kinetic workflow)

> For new work, use [in_grasp_py/](in_grasp_py/) ([MIGRATION.md](MIGRATION.md)). This section documents what the original stack needs, and has not been verified end-to-end.

1. **Environment:** Ubuntu 16.04 + ROS Kinetic, preferably in Docker. ROS Kinetic cannot be installed on 22.04.
2. **Clone into `catkin_ws/src`:** this repo plus the ✅ lab packages from §6, preferably at their 2019–2020 refs.
3. **Install the solvers** at the paths the CMake files expect (or edit them):
   - pagmo2, pagmo_plugins_nonfree headers at `$HOME/pkgs/pagmo_plugins_nonfree/include`;
   - the SNOPT C library in `/usr/local/lib`;
   - fcl and libccd 2.0 in `$HOME/local`.
4. **Pre-build fixes:**
   - `finger_relocation`: fix `finger_gait.cpp` or add a `CATKIN_IGNORE`.
   - `in_hand_controllers/package.xml`: `hand_service` → `hand_services`, and declare `biotac_sensors`.
   - To build the in-grasp planner, delete `in_grasp_manipulation/CATKIN_IGNORE` and configure with `-DENABLE_COLLISION_LIB=OFF`.
5. Run `catkin build` (or `catkin_make`).

---

## 8. Running (original workflows)

All of these are *(inferred)* from the code. Python scripts are Python 2, have no exec bit, and use relative paths, so run them as `python2 <script>` from their own directory.

**A. In-grasp planner (no hardware)**
```bash
roslaunch in_grasp_manipulation allegro_traj_rviz.launch   # robot_description + rviz
roslaunch in_grasp_manipulation in_grasp.launch            # /in_grasp_planner/get_plan (comment out the trajectory_smoothing node)
cd in_grasp_manipulation/scripts && python2 generate_in_grasp.py   # benchmark loop over goal-pose CSV
```
`generate_in_grasp.py` imports several external Python 2 helpers (allegro_driver, trajectory_smoothing, ll4ma_kdl), so expect to trim it.

**B. Friction estimation** (hardware: Allegro + 4 BioTacs + TacNet + surface-normal node)
```bash
roslaunch in_hand_utils point_surface_normal.launch
roslaunch in_hand_controllers tactile_friction_estimation.launch
cd in_hand_controllers/scripts && python2 demo_friction.py     # interactive; writes friction_exp_final.p
```

**C. Tactile impedance grasp** (hardware): run `roslaunch in_hand_controllers tactile_impedance_grasp.launch`, then `python2 demo_impedance_grasp.py`.

**D. Finger relocation in rviz** (after fixing `finger_gait.cpp`):
1. `python2 in_hand_utils/scripts/joint_state_sim_pub.py`
2. `roslaunch finger_relocation relocation.launch`
3. From `finger_relocation/scripts/`, run `python2 demo_finger_plan.py`.

**E. Collision demo:** `roslaunch in_hand_collision_lib allegro_demo.launch`.

A hand-services server must be running for anything on hardware: either the `hand_services` node or `in_hand_controllers/launch/eigen_hand_services.launch`.

---

## 9. Data

- **In-grasp benchmark inputs:** `in_grasp_manipulation/data/initial_data/`.
  - `goal_poses/*.csv`: 100 goals per method = 10 objects × 10.
  - Objects: Apple, Banana, Jello, Lego, Fork, Orange, Plane, Screwdriver, Spatula, Tuna.
  - Columns: name, offset, 16 grasp joints, fingers, initial pose (xyz rpy), desired pose, ….
  - `object_grasps/*.csv`: initial grasps and marker→object centroids.
- **Trajectories:** `in_grasp_manipulation/data/trajectories/reachable_poses/<method>/<Obj>_<k>_<idx>`. Space-separated text, one row of 16 joint angles per timestep. Other trajectory sets sit alongside, plus `trials/`, `weights/` and experiment CSVs.
- **Benchmark results:** `in_grasp_manipulation/scripts/in_grasp_bih` and `in_grasp_ik` are CSVs (method, object, planning time, position/orientation error), not scripts.
- **Duplicate tree:** `in_grasp_manipulation/data/` duplicates `in_hand_manipulation_data/data/in_grasp_manipulation/` (1,510 identical files, 2 extra). Code under `python/` reads the former and `scripts/` reads the latter.
- **Git LFS:**
  - `*.p` under `in_grasp_manipulation/` is tracked in LFS.
  - **5 LFS objects were never uploaded** and remain pointer stubs: `relaxed_rigidity*.p` (in both data trees) and `in_grasp_manipulation/python/rss_data.hdf5`. Clones and merges that try to download them fail; use `GIT_LFS_SKIP_SMUDGE=1`, then `git lfs checkout`.
  - `in_grasp_manipulation/python/.gitattributes` uses `*{p,hdf5}`, but gitattributes has no brace expansion, so it matches nothing.
- **Pickles** (`*.p`): Python 2 protocol-0 pickles, often containing ROS message objects. Under Python 3, load them with `encoding="latin1"` and the msg classes importable.
- **Friction data:** `in_hand_controllers/scripts/friction_exp.p` (an early dataset; the final one is not in the repo).
- **Meshes:** banana in `in_hand_manipulation/meshes/` and `grasp_gaits/data/object_models/`. Most other object meshes referenced by code (YCB, `/home/bala/object_models/…`) are not in the repo.

---

## 10. Modifying and extending

- **New work goes in [in_grasp_py/](in_grasp_py/)** (Python 3, no ROS). [MIGRATION.md](MIGRATION.md) explains the architecture: the original C++ method files are compiled unmodified and wrapped with pybind11.
- **Changing the in-grasp formulation:**
  - Edit or add a file that implements `opt_problems::optProblem`: `objFunction`, `objGradient`, `inEqConstraints`, `inEqGradient`, `bounds`, `gradient_sparsity`.
  - Keep `m_nic` and the sparsity pattern consistent with the constraint vector.
  - Only one formulation can be linked into a binary.
  - `debugGrad` compares analytic gradients with `pagmo::estimate_gradient`, which is useful after edits.
- **Adding a tactile controller:** subclass `hand_task_controller::inHandController` (from `hand_services`) and implement `configureHook`/`updateHook`. Publish joint commands on your own topic and connect it with hand-services `control_mode 5`. Use `tactile_impedance_grasp.cpp` as the template.
- **Robot model:** `in_grasp_manipulation/urdf/allegro.urdf` is the hand used by the planner. Root `allegro_mount`; `palm_link` at xyz (0.065, 0, 0.0275), rpy (0, 1.57, 0); fingertip frames `*_tip`. Meshes come from `package://ll4ma_robots_description`. Lines ~733+ contain a `<klampt>` block that some parsers reject.

---

## 11. Gotchas

- **Python 2 everywhere**: `print x`, `except E, e`, `raw_input`, `xrange`, pandas `df.append` / `.as_matrix()`. Scripts use `sys.path` hacks into other packages, and have no shebangs or exec bits.
- **Hardcoded paths**: `/home/bala/...`, `/media/bala/...`, `$HOME/local`, `$HOME/pkgs`, `/usr/local/lib`.
- **Kinetic-only launch syntax**: `xacro.py --inorder`, `type="state_publisher"`, `joint_state_publisher use_gui`.
- **`ros::init` names containing "/"** throw under `rosrun`; use the launch files.
- **Shared param namespace**: the friction and slip-stabilization configs both use `/allegro_grasp`, so don't load both.
- **Hardware assumptions**: 16 joints, 4 fingers, thumb = finger 3, and demos use fingers `[0,1,3]`.
- **`.gitignore`**: until 2026-11 it contained `*data`, which silently ignored new files in data directories. Now fixed. `*.p` and `*.eps` are still ignored; use `git add -f` to commit one on purpose.
- **Repo clutter**: tracked `*~`, `#…#`, `.pyc`, merge-tool leftovers (`*.BACKUP/BASE/LOCAL/REMOTE.*`) and a dangling symlink in `grasp_gaits/scripts/`.

---

## 12. Glossary

| Term | Meaning |
|---|---|
| **In-grasp manipulation** | Moving a grasped object relative to the palm using only finger joint motion, without changing contacts |
| **Relaxed rigidity** | The constraint that non-rigid fingertips keep (approximately) their initial offsets relative to the rigid finger (thumb) |
| **Finger gaiting / relocation** | Lifting and placing one fingertip at a new contact point while the others hold the object |
| **Eigengrasp** | Low-dimensional synergy basis for hand joint motions (`grasp_utils`) |
| **BioTac** | SynTouch multimodal tactile fingertip sensor |
| **TacNet** | Learned BioTac → force model publishing `/TacNet/bt_force` (external; not found) |
| **KOMO-style acceleration** | Finite-difference joint acceleration cost, as in Toussaint's KOMO |
| **pagmo / SNOPT** | Optimization framework (ESA) / commercial SQP solver used for every planner |
