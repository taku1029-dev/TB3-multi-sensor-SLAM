# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# Session initialization

Before responding to any request, do the following:

1. Read `STATUS.md` and `CLAUDE.md`
2. List which ADRs you've read in this session (initially: none)
3. When I ask about a decision, read the relevant ADR(s) and cite file:line
4. If a request would touch an open decision (per STATUS.md), pause and ask whether to draft an ADR first

Never claim a decision exists without a file:line citation. If you're
unsure, say so.

## Project Overview

Develop a multi-sensor SLAM system where autonomous navigation is driven by a Soft Actor-Critic Reinforcement Learning agent trained in dynamic environments in simulation and transferred to a real robot (Sim-to-Real). The objective is to overcome the limitations of a static Extended Kalman Filter (EKF) by implementing an architecture where an RL agent dynamically adjusts sensor reliability (covariance matrices) in real-time.

## Target Hardware

- Target Robot: TurtleBot3 Burger (equipped with Raspberry Pi 4)
- Sensors: LDS-02 LiDAR, Intel RealSense D435 RGB-D Camera, 9-axis IMU
- Software Stack: ROS 2 Jazzy, Gazebo, PyTorch, Stable Baselines3, OpenAI Gymnasium, OpenCV

## Workspace layout

This is a standard ROS 2 colcon workspace (`/opt/ros/jazzy`) with two packages under `src/`:

- **`custom_robot_description/`** — `ament_cmake` package with URDF/xacro, Gazebo SDF worlds, meshes, RViz config, and the `spawn_robot.launch.py` that launches `gz_sim`, spawns the robot, and bridges Gazebo topics into ROS 2 via `ros_gz_bridge`. Includes a vendored copy of `_d435.urdf.xacro` so it does not depend on `realsense2_description` being installed.
    - `gazebo/`: Stores config file for Gazebo components alignment on GUI.
    - `launch/`: Stores launch files to start `ros_gz_sim` to start a world and spawn a robot with specified URDF on Gazebo, `ros_gz_bridge` to bridge topics into ROS2.
    - `meshes/`: Stores resource files for robot visualization.
    - `rviz/`: Stores config file for RViz settings.
    - `urdf/`: Stores URDF and xacro files for custom robot description.
    - `worlds/`: Stores Gazebo world sdf files.
    - `typings/`: Stores stubs for Python.

- **`rl_navigation_pkg`** — `ament_python` package hosting the RL env, the EKF-input-gate wrapper, and a stand-in release driver. SAC training and `RLNavigation-v1` are still TODO.
    - `rl_navigation_pkg/`
        - `envs/my_env.py`: `RLNavigation-v0` (MVP env, ADR-001). Lives until `RLNavigation-v1` is end-to-end.
        - `envs/sac_env.py`: `RLNavigation-v1` (SAC env, ADR-005/008/009/010/011/012). Built up A-1 → A-3; A-4 (collision/divergence terminate) still TODO. 15-dim obs, 2-dim Box action with σ=exp(a·3), 50 ms set_parameters ack deadline, soft Gazebo reset via subprocess gz CLI in reset().
        - `agents/`: SB3 agent training and inference wrappers (planned).
        - `nodes/ekf_input_gate.py`: ADR-011/014 wrapper. Sits between raw EKF inputs and `ekf_filter_node`; applies σ × nominal-diagonal at release.
        - `nodes/release_driver.py`: Interim 10 Hz Trigger client clocking the gate. Redundant once `RLNavigation-v1.step()` is the sole release driver; kept in launch so non-RL navigate_to_pose runs still get /odom flowing.
        - `nodes/env_smoke_test.py`: Runs `RLNavigation-v0` for 50 random steps via `ros2 run rl_navigation_pkg env_smoke_test`.
        - `nodes/v1_smoke_test.py`: Runs `RLNavigation-v1` for 2 episodes × 25 random-action steps with reset() between, via `ros2 run rl_navigation_pkg v1_smoke_test`. Verifies obs decode + action wiring + reset path.
        - `nodes/reward_probe.py`: 1 Hz standalone probe comparing `/ground_truth_pose` vs `/odom`; validates the reward subtrahend path (ADR-004:39-40) before integrating into `RLNavigation-v1`.
    - `config/`
        - `ekf_phase1.yaml`: ADR-006 Phase 1 EKF + gate config (single file, dispatched by node name).
        - `slam_toolbox_params.yaml`: ADR-007 online_async config; phase-invariant.
        - `nav2_params.yaml`: ADR-013 plugin selection; phase-invariant.

A project-local `.venv/` (Python 3.12) lives at the workspace root and is used for ML deps (torch, gymnasium, stable-baselines3). ROS 2 itself comes from the system `/opt/ros/jazzy` install — when running ROS nodes you typically need both environments active.

## Common commands

All commands assume cwd is the workspace root and that `/opt/ros/jazzy/setup.zsh` has been sourced.

```zsh
# Build everything (or a single package)
colcon build --symlink-install
colcon build --packages-select custom_robot_description

# Source the overlay (must be done in every new shell that runs nodes)
source install/setup.zsh

# Run the simulation (gz-sim + spawn robot + ros_gz bridges)
ros2 launch custom_robot_description spawn_robot.launch.py

# Regenerate URDF from xacro (the build does NOT do this — must run manually)
xacro src/custom_robot_description/urdf/custom_robot.urdf.xacro \
  > src/custom_robot_description/urdf/custom_robot.urdf

# Tests + lint (rl_navigation_pkg uses ament_flake8 / ament_pep257 / ament_copyright)
colcon test --packages-select rl_navigation_pkg
colcon test-result --verbose
# Single test by name pattern:
colcon test --packages-select rl_navigation_pkg --pytest-args -k test_flake8
```

## Architecture notes

**Sim ↔ ROS 2 boundary.** The robot exists as URDF for ROS-side tools and as Gazebo plugin definitions inside the same xacro (`custom_robot.urdf.xacro`). Inside `<gazebo>` blocks the file declares: a `DiffDrive` plugin listening on `cmd_vel` and publishing wheel odometry on `odom_wheel` (wheel separation 0.160 m, radius 0.033 m; `/odom` itself is post-cutover EKF-owned per ADR-002:52), a `JointStatePublisher`, a `PosePublisher` (sim-only ground-truth, bridged to `/ground_truth_pose` for ADR-005 reward), the RGBD sensor publishing on `camera/rgbd`, the lidar (`type="gpu_lidar"`, ADR-003) on `lidar`, and an IMU on `imu`. Sensor `<gz_frame_id>` overrides are required so frame_ids match the URDF link names (gz-sim's default is model-prefixed). Anything that needs to cross into ROS 2 must be added to the `ros_gz_bridge parameter_bridge` arguments in `spawn_robot.launch.py`, including `/clock` (required when `use_sim_time:True`). Topic contract: ADR-001 for `RLNavigation-v0` (current MVP env), ADR-005 for `RLNavigation-v1` (trained system, planned).

**RL package state.** `rl_navigation_pkg` registers `RLNavigation-v0` (entry point `rl_navigation_pkg.envs.my_env:MyEnv`). `MyEnv` subscribes to `/lidar`, `/imu`, `/odom_wheel` and publishes `/cmd_vel`. Observation is a 24-sector downsampled lidar (`Box`); action is `Discrete(4)` per ADR-002. Reward is a placeholder `0.0` — shaping and the SAC agent are not yet implemented. A smoke test runs via `ros2 run rl_navigation_pkg env_smoke_test`. The trained-system env `RLNavigation-v1` (ADR-005) does not exist yet.

## Known gotchas

- **xacro is not auto-regenerated.** `colcon build` does not run xacro. After editing any `.xacro`, regenerate the `.urdf` manually (see Common commands).
- **Dual environment activation.** Running RL training inside ROS nodes requires both `source /opt/ros/jazzy/setup.zsh` *and* `.venv/bin/activate`. Order matters — activate `.venv` last so its `python` shadows the system one.
- **Bridges are not automatic.** New Gazebo topics are invisible to ROS 2 until added to `ros_gz_bridge` args in `spawn_robot.launch.py`. Adding a sensor to the env requires both bridging it *and* subscribing in `MyEnv` — see ADR-001.
- **gz-sim prefixes sensor and DiffDrive frame_ids with the model name by default.** Without overrides you get `header.frame_id: custom_robot/odom` etc., which won't match `ekf_phase1.yaml`'s `odom_frame: odom` / `base_link_frame: base_footprint` and the EKF silently drops every input. Pin them explicitly: DiffDrive plugin uses `<frame_id>odom</frame_id>` `<child_frame_id>base_footprint</child_frame_id>`; sensor blocks (IMU, lidar, …) use `<gz_frame_id>LINK_NAME</gz_frame_id>` inside `<sensor>`.
- **`use_sim_time:True` requires `/clock` to be bridged.** Any node with `use_sim_time:True` (EKF, gate, future SAC env) blocks at clock=0 until `/clock` flows. Always include `'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'` as the first bridge arg.
- **`robot_state_publisher` is mandatory.** gz-sim does not publish the URDF's static-joint TF chain to ROS 2. Without `robot_state_publisher` in the launch, EKF cannot transform IMU (`imu_link`) into `base_link_frame` and rejects every IMU message. Load `robot_description` via `Command(['cat ', urdf_file])` from the generated URDF.
- **`robot_localization`'s `ekf_node` publishes on `/odometry/filtered` by default.** ADR-002 says `/odom` is EKF-owned post-cutover, so add `remappings=[('odometry/filtered', '/odom')]` on the EKF Node — there is no parameter for the output topic name.
- **`--symlink-install` plus a setup.py edit can leave stale egg-info.** If a node fails with `PackageNotFoundError: No package metadata was found for <package>`, clean and rebuild: `rm -rf build/<pkg> install/<pkg> && colcon build --symlink-install --packages-select <pkg>`.
- **`type="lidar"` silently fails to render in gz-sim Harmonic.** Use `<sensor type="gpu_lidar">` for the SDF lidar block. Both `type="lidar"` and `type="gpu_lidar"` parse, but only the latter triggers the gpu_rays render path.
- **Lidar SDF schema gotchas.** Inside `<lidar>...</lidar>`: use `<samples>` (plural) — not `<sample>` — and place `<range>` as a direct child of `<lidar>`, not nested inside `<scan>`. Wrong placement is silently ignored when `type="lidar"`, but with `type="gpu_lidar"` the missing `<range><min>` becomes `near_clip = 0` and OGRE aborts (`Near clip distance must be greater than zero`).
- **`async_slam_toolbox_node` is a managed lifecycle node in Jazzy.** Plain `Node(...)` in the launch leaves it at `unconfigured` forever — no `/lidar` subscription, no `/map`, no `map → odom` TF, and only `[INFO] Node using stack size ...` printed. Use `LifecycleNode(...)` plus an `EmitEvent(ChangeState(TRANSITION_CONFIGURE))` and an `OnStateTransition(goal_state='inactive')` handler that emits `TRANSITION_ACTIVATE`. Reference: `src/custom_robot_description/launch/spawn_robot.launch.py`.
- **`nav2_msgs` ↔ `fastcdr` ABI mismatch in some apt-installed Jazzy environments.** Symptom: `controller_server` dies on activation with `symbol lookup error: ... libnav2_msgs__rosidl_typesupport_fastrtps_cpp.so: undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEj`. Fix: `sudo apt install --reinstall ros-jazzy-fastcdr ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rosidl-typesupport-fastrtps-cpp ros-jazzy-rosidl-typesupport-fastrtps-c ros-jazzy-nav2-msgs` to get a consistent set. Quick workaround: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (Cyclone DDS doesn't go through fastcdr).
- **bt_navigator BT-XML param: omit, don't blank.** `default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml` must be **absent** from the yaml in Nav2 Jazzy for the package-shipped default to kick in. Setting them to `""` loads an empty tree → `Behavior tree threw exception: Empty Tree`. Setting them to `"$(find-pkg-share nav2_bt_navigator)/..."` pushes the literal substitution string through (yaml does not expand `$(find-pkg-share ...)`) → file-not-found → goal rejected.
- **`while true; ros2 service call ...` is too bursty to drive the gate.** Each invocation forks a python process and re-runs rclpy init, taking 100–300 ms per call. Result: /odom updates are unstable, TF cache is intermittent, and downstream Message Filters (costmap_2d, slam_toolbox) drop scans with `'the timestamp on the message is earlier than all the data in the transform cache'`. Use a persistent Python timer node (`release_driver`) with `client.call_async(...)` instead.
- **`gz service /world/<name>/control` with `reset: {all: true}` tears down sensor plugin entities.** Empirically (2026-05-11) the `all` reset destroys IMU and PosePublisher entities and recreates them, but existing ROS subscribers do not re-discover the new publishers within seconds — `/imu` and `/ground_truth_pose` go silent for the rest of the process while `/lidar` and `/camera/rgbd/image` survive. The downstream EKF then has no IMU input and stops publishing `/odom` too. **Use `reset: {model_only: true}` instead** — it resets all model poses to spawn while keeping sensor plugins running. See `RLNavigation-v1.reset()` for the canonical incantation.

## Working with this repo

### Reading order for new sessions

When starting a new conversation about this project:

1. Read `STATUS.md` first — it tells you what's decided and what's open
2. Read `CLAUDE.md` (this file) for workspace conventions
3. Read ADRs in numeric order, but skip superseded ones unless history is relevant
4. Before implementing anything, verify your understanding against STATUS.md and cite ADR file:line for every claimed decision

### Terminology conventions

To avoid the "sensor" ambiguity that has caused past confusion:

| Term | Meaning |
|---|---|
| Raw sensor | Physical device (LDS-02, D435, IMU chip) |
| EKF input stream | An odometry-shaped message fed into ekf_node |
| σ_xxx | Observation noise covariance for EKF input stream xxx |
| /odom | EKF's filtered pose output (post-cutover) |
| /odom_wheel | DiffDrive's wheel odometry (post-cutover) |

### Citation requirement

When confirming or stating a project decision, always cite the
authoritative source as `<file>:<line>`. Do not paraphrase ADR text
without a citation; if you cannot cite, the decision is not yet made.
**Check ADRs first.** Before any significant modification, scan the ADR list below and read any that look relevant. ADRs capture *why* the current implementation looks the way it does — skipping them risks reintroducing a problem we already solved.

## Architecture Decision Records (ADRs)

ADRs live in `adr/` as small focused markdown files. Each ADR records one decision: its context, the choice made, and the consequences. Keep them short — if an ADR grows past ~1 page, it probably wants splitting.

### When to create a new ADR

Create one when any of the following is true:

- A non-obvious technical choice is made (library, algorithm, data layout, protocol)
- A constraint or convention is introduced that future changes must respect (e.g. "all sensor topics must pass through the EKF wrapper")
- A workaround is added for a hardware, simulator, or upstream-library quirk
- A decision is reversed — supersede the old ADR rather than editing it in place

Do NOT create an ADR for trivial refactors, formatting, or anything that would be obvious from reading the code itself.

### How to create one (Claude follows this procedure)

1. Pick the next free number: list `adr/` and use `NNN` = max existing + 1, zero-padded to 3 digits.
2. Filename: `adr/NNN-short-kebab-title.md` (e.g. `adr/007-dqn-reward-shaping.md`).
3. Use this template:

   ```markdown
   # ADR-NNN: <Title>

   ## Status

   Accepted  <!-- or: Proposed | Superseded by ADR-XXX | Deprecated -->

   ## Context

   What problem or situation prompted this decision? What constraints apply?

   ## Decision

   The choice made, stated clearly. One or two sentences if possible.

   ## Consequences

   What this implies for future work — both positive and negative.
   What rules or invariants future contributors (including Claude) must follow.

   ## Alternatives considered

   Brief — one line each is fine. Why they were rejected.
   ```

4. Add the new file to the ADR index below in this `CLAUDE.md` (filename + one-line summary), in numeric order.
5. If the new ADR makes any existing ADR obsolete, mark the old one's Status as `Superseded by ADR-NNN` rather than deleting it.

### When to update an existing ADR

- Minor clarification or correction → edit in place.
- Decision is reversed or materially changed → write a new ADR that supersedes the old one. Do not rewrite history.

### ADR index

<!-- Keep this list in sync with adr/. One line per ADR, numeric order. -->

- [`001-rl-env-topic-contract.md`](adr/001-rl-env-topic-contract.md) — _(Superseded by ADR-005.)_ MVP env's topic contract: lidar/imu/odom in, cmd_vel out. Retained for history.
- [`002-ros2-system-architecture.md`](adr/002-ros2-system-architecture.md) — Trained-system topology: SAC agent tunes EKF covariance only; Nav2 drives motion; `robot_localization` fuses LiDAR + Camera + IMU. Supersedes original action-space ADR. _(Input enumeration and 3-vector action wording partially superseded by ADR-006.)_
- [`003-lidar-laserscan-message-type.md`](adr/003-lidar-laserscan-message-type.md) — `/lidar` uses `sensor_msgs/msg/LaserScan` (not `PointCloud2`) because LDS-02 is a 2D lidar.
- [`004-agent-nav2-timeline.md`](adr/004-agent-nav2-timeline.md) — Per-cycle ordering at 10 Hz: sensors → agent observes → covariance write → EKF tick → Nav2 cmd_vel → reward. Covariance write must precede the EKF tick.
- [`005-sac-env-topic-contract.md`](adr/005-sac-env-topic-contract.md) — Trained SAC env (`RLNavigation-v1`) topic contract: reads sensors + `/odom` + ground-truth pose; action is a `set_parameters` call to `ekf_filter_node`, not a `/cmd_vel` publish. Supersedes ADR-001.
- [`006-ekf-input-source-phasing.md`](adr/006-ekf-input-source-phasing.md) — Three-phase EKF input rollout: Phase 1 wheel+IMU, Phase 2 +LiDAR-odom, Phase 3 +visual-odom. Action vector grows length 2 → 3 → 4. Partially supersedes ADR-002's input enumeration and action-vector length.
- [`007-nav2-map-source-slam-toolbox.md`](adr/007-nav2-map-source-slam-toolbox.md) — Nav2's map comes from `slam_toolbox` in `online_async` mode, fed by the EKF's filtered pose via TF. Reward stays on filtered pose; the `map → odom` correction is deliberately excluded.
- [`008-sac-observation-vector.md`](adr/008-sac-observation-vector.md) — 15-dim handcrafted observation: lidar quality (4) + camera quality (2) + IMU dynamics (6) + EKF planar-pose covariance (3). Phase-invariant base.
- [`009-sac-action-space.md`](adr/009-sac-action-space.md) — Action is `Box(-1, 1, (N,))` with log-σ interpretation `σ = exp(a · 3)`; observation is `Box(-1, 1, (15,))` per-feature normalised; σ-in-obs excluded; phase transitions discard replay buffer with 0.5× LR warmup.
- [`010-episode-design.md`](adr/010-episode-design.md) — 600-step (60 s) episodes; collision/divergence terminate, timeout/infra-failure truncate; phase-advancement requires rolling reward > −5 + plateau; map rebuilt each episode via Gazebo soft reset.
- [`011-ekf-tick-scheduling.md`](adr/011-ekf-tick-scheduling.md) — A coordinator wrapper node gates EKF input streams, releasing them only after the agent's `set_parameters` ack, to enforce ADR-004:43 without forking `robot_localization`.
- [`012-set-parameters-call-mechanics.md`](adr/012-set-parameters-call-mechanics.md) — `set_parameters` ack deadline is 50 ms; on timeout skip cycle (drop transition); stream-staleness proceeds with latest-buffered (no cycle-skip); infrastructure failures truncate the episode.
- [`013-nav2-planner-controller-selection.md`](adr/013-nav2-planner-controller-selection.md) — Nav2 plugin selection: `NavfnPlanner` + `RegulatedPurePursuitController` + stock BT + standard costmap layers; single phase-invariant `config/nav2_params.yaml`. RPP chosen over DWB for determinism (training stability + Sim-to-real). Closes the planner-selection open question raised in ADR-002.
- [`014-sigma-application-via-ekf-input-gate.md`](adr/014-sigma-application-via-ekf-input-gate.md) — σ application happens inside the EKF input gate wrapper, not in `/ekf_filter_node`. Agent's `set_parameters` target is `/ekf_input_gate`; wrapper multiplies the per-stream nominal-diagonal by σ at release. Partially supersedes ADR-005:29 and ADR-011:27.
