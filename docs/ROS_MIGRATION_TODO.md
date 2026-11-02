# TODO (deferred): ROS Noetic → ROS 2 Humble migration

**Status: deferred** (decision 2026-11-02, see [MIGRATION.md §10](../MIGRATION.md#10-decision-log)). The
current focus is a ROS-free Python 3 + viser stack. This file keeps what we learned so the ROS work can start
later without repeating the research.

- **Last updated:** 2026-11-02
- **Snapshot date for the dependency findings:** 2026-10-30

---

## Target sequence

1. **Noetic** (Ubuntu 20.04, in Docker; the host is 22.04 with Humble). This is the smallest step from Kinetic: still catkin and ROS 1, but Python 3.
2. **Humble.** The lab's newer robots run Humble control stacks, so this is the long-term target.

## Recommended approach

- **Thin ROS node.** Don't port the old server. Wrap the ROS-free `in_grasp` core library (built in MIGRATION.md M2) in a thin node that keeps the existing `InGraspManipulation.srv` interface. The node then just maps the srv to the core's `Request`/`Result`, and the Noetic→Humble jump only touches that wrapper.
- **Keep viser ROS-agnostic.** If viser ever needs to talk to a ROS planner, use `rosbridge_server` plus `roslibpy`. The same client then works against both Noetic and Humble rosbridge.
- **Replace `trajectory_smoothing`** (only used for execution/visualization; the repo was not found) with toppra or ruckig.

---

## Lab dependency status (Bitbucket `robot-learning`)

These were checked with `git ls-remote` and blob-less clones on 2026-10-30. "Bala era" means the last commits by Balakumar, around 2020-04-16, which form a consistent snapshot.

| Repo | Needed for | Recommended ref | Noetic / Py3 state | Notes / risks |
|---|---|---|---|---|
| `ll4ma_opt_utils` | in-grasp planner (core) | `main@4ea5bfa`; C++ identical to Bala's `d3485b7` | Not ready: C++11, Melodic install scripts, package.xml format 1/2 | **Contains `ll4ma_opt_wrapper`** as a subpackage (`ll4ma_opt_wrapper/include/ll4ma_opt_wrapper/pagmo_opt/wrapper.hpp`). CMake requires fcl and `ccd 2.0 EXACT` from `$HOME/local`; ppnf headers from `$HOME/pkgs`. `plan_env_utils.hpp` always includes kris_ccd unless `LL4MA_COLLISION_MODEL` is predefined. Base-class virtuals have empty bodies that return values (UB if called) |
| `ll4ma_kdl` | in-grasp planner (core) | `main@78fa833` (superset of `bala_devel@26cff40`) | Closest: C++14, DART optional. Python scripts are Py2. Needs `trac_ik_lib` | Header dropped `using namespace std`. `getFKEigen` (used by `in_hand_controllers`) exists only in the legacy `robot_kdl_bck.h` |
| `ll4ma_collision_wrapper` | optional (collision is off / stubbed) | `110c4fb` (Bala era) or `main@32fbc67` | Melodic at best; `std::shared_ptr` change marked untested | Heavy: KrisLibrary, libccd, FCL, CGAL, octomap, PCL, assimp. `kris_ccd/manipulation.h` changed after Bala's commits. Recommend disabling |
| `ll4ma_robots_description` | meshes / URDF | `main@c3d999d` for meshes; `8a93a13` for the Bala-era xacro | Unverified; actively maintained (2026, cuRobo work) | Main's Allegro xacro was restructured (virtual sphere links, mount, table). Old in-repo URDFs use the pre-2018 name `urlg_robots_description`; a sed rename to `ll4ma_robots_description` fixes them |
| `ll4ma_planner` | not needed | — | Not ready | Only used by `old/` code |
| `ll4ma_scene_description` | launch-file YAMLs only | `main@71ed0e9` | Unverified | Scene YAMLs (`table_obj.yaml`) |
| `hand_services` | hardware controllers | `bala_dev@583c7ab`, or `main@8477897` | Kinetic/Melodic era (C++14) | `in_hand_controllers` subclasses its `inHandController` and `controlInterface` |
| `allegro_driver` | hardware | `main@684a75d` | Unverified | PCAN `canAPI` |
| `biotac_sensors` | hardware | `bala-devel@106d2f0`, or `main@2c5fd3f` | Unverified | Ships prebuilt Total Phase Cheetah `.so` files |
| `grasp_utils` | `in_hand_controllers` (eigengrasps) | `master@8a21566` | Unverified | Small library |
| `trajectory_smoothing` | execution / visualization | **not found** | — | Replace with toppra or ruckig |
| `urlg_robots_description` | legacy only | **not found** (renamed) | — | Use `ll4ma_robots_description` |
| `optimization_pkg` | legacy only | **not found** | — | Only `old/` code |
| `ll4ma_opt_wrapper` (as its own repo) | — | **not found** | — | Lives inside `ll4ma_opt_utils` |
| TacNet (`/TacNet/bt_force`) | tactile controllers | **not found** | — | Ask the lab |

**Solver forks.** Bala's install scripts (`ll4ma_opt_wrapper/install_dependencies.sh`, `install_snopt.sh`) pull his GitHub forks:
- `balakumar-s/pagmo2` ("version 10");
- `balakumar-s/pagmo_plugins_nonfree`;
- `balakumar-s/snopt-interface`, branch `ll4ma_pkgs`;
- plus NLopt 2.4.2.

The core port (MIGRATION.md) moves to upstream pagmo2 and ppnf. A ROS wrapper should reuse that core instead of these forks.

---

## Known Noetic breakages to fix

**Launch / description**
- [ ] Replace `xacro.py` and `--inorder` with `xacro` (`in_hand_controllers/launch/eigen_hand_services.launch`, `in_hand_manipulation/launch/description.launch`, `in_grasp_manipulation/launch/allegro_marker_rviz.launch`).
- [ ] Change `type="state_publisher"` to `robot_state_publisher` (all `in_grasp_manipulation/launch/*.launch` and `in_hand_manipulation_msgs/launch/allegro_traj_rviz.launch`).
- [ ] Change `joint_state_publisher` with `use_gui` to `joint_state_publisher_gui`.
- [ ] Rename `package://urlg_robots_description` to `ll4ma_robots_description` (`in_grasp_manipulation/urdf/allegro_{edit,marker,thumb}.urdf`, `in_hand_manipulation_msgs/urdf/`).

**Build**
- [ ] Remove the hardcoded `$HOME/local` (fcl/ccd) and `$HOME/pkgs/pagmo_plugins_nonfree` paths in the `CMakeLists.txt` of `in_grasp_manipulation`, `in_hand_manipulation`, `finger_relocation`, `in_hand_collision_lib`, `grasp_gaits` and `ll4ma_opt_utils`.
- [ ] Remove the `find_package(ccd 2.0 EXACT REQUIRED)` and `Pagmo REQUIRED` requirements from `in_hand_manipulation/CMakeLists.txt`. That package builds nothing but still demands them.
- [ ] Raise `-std=c++11` to C++17 for pagmo ≥2.16-ish.
- [ ] Fix `in_hand_controllers/package.xml`: the `hand_service` typo; declare `biotac_sensors` and `visualization_msgs`.
- [ ] Decide on the `CATKIN_IGNORE` files. Currently ignored: `in_grasp_manipulation`, `grasp_gaits`, `old/*`. Alternatively use `catkin build` with an explicit package list.
- [ ] `finger_relocation/src/problem/finger_gait.cpp` doesn't compile at HEAD (MIGRATION.md F3).

**Runtime**
- [ ] `ros::init` names containing "/" (`in_hand_utils/src/*surface_normal_node.cpp`, `in_hand_controllers/src/node/tactile_impedance_grasp_node.cpp`) throw under `rosrun`.
- [ ] The friction and slip-stabilization YAMLs share the param namespace `/allegro_grasp`.
- [ ] `in_grasp_server` busy-waits on `/robot_description` with no sleep, and reads params relative to its namespace.

**Python**
- [ ] All ROS-side scripts are Python 2:
  - `print x`, `except X, e`, `raw_input`, `xrange`, `file()`;
  - pandas `df.append` / `as_matrix`;
  - many `sys.path` hacks into other packages;
  - no shebangs or exec bits.
- [ ] Pickled data contains Python 2 ROS message objects. Load with `encoding="latin1"` and the msg classes available.

---

## Per-package port order (when this resumes)

1. Thin `in_grasp` ROS node around the core lib, plus `in_grasp_manipulation/srv/InGraspManipulation.srv`, plus a Noetic launch file (URDF, params, node).
2. `in_hand_manipulation_msgs`. It builds with standard deps only; most of its msgs are unused (see the repo audit).
3. `in_hand_utils`: surface-normal nodes and the joint-state sim. Needs `ll4ma_opt_utils::envUtils`, which means collision is required.
4. `in_hand_controllers`: needs the hardware repos above, plus TacNet.
5. `finger_relocation` / `in_hand_collision_lib`, after the F3 fixes.
6. Then repeat 1–5 for Humble (rclcpp/rclpy, ament, Python launch files).

## Checklist to un-defer

- [ ] MIGRATION.md M3 is done, so the core library exists.
- [ ] Decide Noetic-in-Docker vs going straight to Humble.
- [ ] Confirm the lab's current Humble control stack interfaces for the Allegro hand and its sensors.
