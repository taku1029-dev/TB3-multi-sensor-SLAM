# STATUS.md

A snapshot of where the SAC-tuned-EKF SLAM project is *right now*. For the *why* behind each decision, see `adr/`. For the cold-start orientation that points at what to read first, see this file plus `CLAUDE.md`.

## 1. Current implementation phase

We are in the **post-bring-up phase** (post-MVP env, pre-SAC). The Gymnasium scaffold (`RLNavigation-v0`) coexists with a working ADR-006 Phase 1 EKF chain, a live `slam_toolbox` providing `/map` + `map → odom`, and a live Nav2 stack consuming both. End-to-end navigation works: `ros2 action send_goal /navigate_to_pose ...` returns `SUCCEEDED` with the robot driving to the goal under Nav2's RPP controller. SAC env is still the only un-wired piece.

### What exists today

- `MyEnv` Gymnasium env — `src/rl_navigation_pkg/rl_navigation_pkg/envs/my_env.py`. Id `RLNavigation-v0`. Observation: 24-sector downsampled lidar `Box`. Action: `Discrete(4)` (forward / left / right / stop). Reward: placeholder `0.0`. Subscribes `/lidar`, `/imu`, `/odom_wheel`. Publishes `/cmd_vel`.
- Smoke-test entry — `src/rl_navigation_pkg/rl_navigation_pkg/nodes/env_smoke_test.py`. Runs 50 random steps. Wired as `ros2 run rl_navigation_pkg env_smoke_test`.
- **End-to-end stack wired into `spawn_robot.launch.py`** — `ros2 launch custom_robot_description spawn_robot.launch.py` brings up Gazebo + bridge + `robot_state_publisher` + `ekf_input_gate` + `release_driver` + `ekf_filter_node` + `slam_toolbox` + Nav2 (controller / planner / behavior / bt_navigator + lifecycle_manager). Verified 2026-05-09: navigate_to_pose action SUCCEEDED on a (0.5, 0.0) goal.
- `ekf_input_gate` node — `src/rl_navigation_pkg/rl_navigation_pkg/nodes/ekf_input_gate.py`. Phase 1 streams (wheel + IMU). σ multipliers exposed as ROS 2 parameters. Republishes onto `/ekf_in/odom_wheel` and `/ekf_in/imu` only on `/ekf_input_gate/release` Trigger service call.
- `release_driver` node — `src/rl_navigation_pkg/rl_navigation_pkg/nodes/release_driver.py`. **Interim** 10 Hz Trigger client driving the gate. Replaces an unstable shell `while true` loop. To be superseded by `RLNavigation-v1`'s `step()` per ADR-004:15-41.
- `reward_probe` node — `src/rl_navigation_pkg/rl_navigation_pkg/nodes/reward_probe.py`. **Verification-only** 1 Hz scaffold: subscribes `/ground_truth_pose` + `/odom`, computes the SAC env's planned reward (`-sqrt(dx² + dy²) * scale` per ADR-004:39-40), logs both poses + L2 + yaw error. Run via `ros2 run rl_navigation_pkg reward_probe`. Verified 2026-05-10: at rest L2=0 and frame_ids `example_world` (GT) / `odom` (EKF) confirmed; under Nav2-driven motion both pose channels track and diverge (Phase 1 EKF drift exposed); missing-side resilience verified for both topics.
- `ekf_phase1.yaml` — `src/rl_navigation_pkg/config/ekf_phase1.yaml`. Holds both `ekf_filter_node` (fuses vx/vyaw from wheel + vyaw from IMU; `publish_tf:true`; `world_frame:odom`) and `ekf_input_gate` (nominal diagonals + initial σ=1.0) parameters; ROS 2 dispatches by node name. EKF output is remapped from `odometry/filtered` to `/odom` per ADR-002:52.
- `slam_toolbox_params.yaml` — `src/rl_navigation_pkg/config/slam_toolbox_params.yaml`. `online_async` mapper config; `odom_frame:odom`, `base_frame:base_footprint`, `scan_topic:/lidar`, `max_laser_range:8.0`, `minimum_travel_distance:0.1` (TB3 Burger small steps), `map_update_interval:5.0` (= /map at 0.2 Hz, intentional), `transform_publish_period:0.02` (= map→odom TF at ~50 Hz). One file, phase-invariant per ADR-007:34.
- slam_toolbox is launched as a `LifecycleNode` with auto-`configure → activate` event handlers (Jazzy's `async_slam_toolbox_node` is a managed lifecycle node and does NOT auto-activate as a plain `Node`).
- `nav2_params.yaml` — `src/rl_navigation_pkg/config/nav2_params.yaml`. Plugin selection per ADR-013: NavfnPlanner + RegulatedPurePursuitController + stock BT + standard costmap layers. Both costmaps consume `/lidar` directly via `nav2_costmap_2d::ObstacleLayer`; global costmap consumes slam_toolbox's `/map` via `StaticLayer`. `bt_navigator` has NO `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml` keys — Nav2's built-in default kicks in only when these are omitted entirely.
- Nav2 nodes (`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`) are launched explicitly (not via `nav2_bringup.navigation_launch.py`) so we can omit `velocity_smoother` / `waypoint_follower` etc. that aren't in scope. `lifecycle_manager_navigation` brings them up via `autostart:True`.
- Bridges: `/clock`, `/lidar`, `/imu`, `/odom_wheel`, `/cmd_vel`, `/model/custom_robot/joint_state`, camera topics, plus `/model/custom_robot/pose` remapped to `/ground_truth_pose` (ADR-005 reward).
- DiffDrive plugin in xacro publishes `/odom_wheel` (note: `/odom` is now EKF-owned post-cutover); explicit `<frame_id>odom</frame_id>` and `<child_frame_id>base_footprint</child_frame_id>` overrides gz-sim's model-prefix default.
- IMU sensor in xacro pinned to `<gz_frame_id>imu_link</gz_frame_id>` (100 Hz). Lidar sensor uses `type="gpu_lidar"` with `<gz_frame_id>base_scan</gz_frame_id>`; SDF schema corrected (`<samples>` plural; `<range>` is sibling of `<scan>`, not child).
- `robot_state_publisher` runs in the launch and exposes the URDF static-TF chain (`base_footprint → base_link → imu_link / base_scan / camera_link / wheel_*`) to the EKF, slam_toolbox, and Nav2.
- ADRs 001–014 (see `adr/`).

### What does not exist yet

- `RLNavigation-v1` SAC env (the trained-system env per ADR-005).
- LiDAR-derived odom node (Phase 2) and RGB-D visual odom node (Phase 3) — package choices not pinned. Phase 2/3 ekf yaml variants (`ekf_phase{2,3}.yaml`) also not written yet.
- Reward-loop integration into the SAC env — `reward_probe` confirmed the `/ground_truth_pose` ↔ `/odom` subtraction path works (frame names, units, missing-side handling, motion divergence all verified 2026-05-10), but `MyEnv._compute_reward` still returns `0.0`; the formula will be inlined into `RLNavigation-v1.step()`.
- Episode-reset path for slam_toolbox (ADR-010:62 `clear_map` service call from the env's `reset()`) — slam_toolbox is up but reset hooks come with `RLNavigation-v1`.
- Combined bringup launch separating sim from algorithm — for now everything lives inside `spawn_robot.launch.py`.
- SAC training script and `agents/` package contents.

## 2. Confirmed decisions

Each line cites the ADR `file:line` that supports it.

### Topology and ownership

- Four cooperating ROS 2 actors at runtime: Gazebo, `robot_localization`, Nav2, SAC agent — `adr/002-ros2-system-architecture.md:9`. ADR-007 adds `slam_toolbox` as a fifth actor that supplies Nav2's map.
- The agent performs **only** covariance tuning. It does not publish `/cmd_vel` and does not subscribe to motor commands — `adr/002-ros2-system-architecture.md:41`
- Nav2 owns `/cmd_vel` and reads the corrected pose from the TF tree (`map → odom → base_link`) — `adr/002-ros2-system-architecture.md:42`, `adr/007-nav2-map-source-slam-toolbox.md:21`
- After cutover, `/odom` is owned by the EKF; DiffDrive's wheel odom is remapped to `/odom_wheel` — `adr/002-ros2-system-architecture.md:52`, `adr/005-sac-env-topic-contract.md:22`, `:48`, `adr/006-ekf-input-source-phasing.md:32`

### EKF input phasing (ADR-006)

- EKF input streams are introduced in three phases. IMU is present in every phase — `adr/006-ekf-input-source-phasing.md:21-25`
  - Phase 1: wheel odom (`/odom_wheel`) + IMU (`/imu`)
  - Phase 2: + LiDAR-derived odom (proposed `/odom_lidar`)
  - Phase 3: + RGB-D visual odom (proposed `/odom_visual`)
- Phase advancement is gated on three conditions (SAC convergence, new stream produces valid messages in isolation, EKF runs cleanly with new input and agent paused) — `adr/006-ekf-input-source-phasing.md:46-50`
- Each phase ships its own `config/ekf_phase{1,2,3}.yaml`; selected at launch time — `adr/006-ekf-input-source-phasing.md:38-42`
- ADR-002's three-input enumeration at `:43` and length-3 action wording at `:41` are partially superseded by ADR-006.

### Map source (ADR-007)

- `slam_toolbox` runs in `online_async` mode, providing live SLAM during operation — `adr/007-nav2-map-source-slam-toolbox.md:15`
- Pose source for `slam_toolbox` is the EKF's filtered pose via TF (`odom → base_link`); requires `publish_tf: true` on `ekf_filter_node` — `adr/007-nav2-map-source-slam-toolbox.md:19-20`
- `slam_toolbox` publishes `/map` (consumed by Nav2 global costmap) and `map → odom` TF (correcting accumulated EKF drift) — `adr/007-nav2-map-source-slam-toolbox.md:21-22`
- `slam_toolbox` config is invariant across ADR-006 phases (one file, no per-phase variants) — `adr/007-nav2-map-source-slam-toolbox.md:34`
- `slam_toolbox` runs asynchronous to the ADR-004 10 Hz cycle and does **not** participate in the per-cycle ordering invariant — `adr/007-nav2-map-source-slam-toolbox.md:38-39`

### SAC env contract (ADR-005 supersedes ADR-001)

- Gymnasium id `RLNavigation-v1`, coexists with `RLNavigation-v0` until `v1` is end-to-end working — `adr/005-sac-env-topic-contract.md:13`, `:47`
- Action interface is a **`set_parameters` call** (not a topic publish), targeting **`/ekf_input_gate`** (the wrapper, per ADR-014) rather than `/ekf_filter_node` — `adr/005-sac-env-topic-contract.md:29` (partially superseded by `adr/014-sigma-application-via-ekf-input-gate.md`), `adr/005-sac-env-topic-contract.md:43` (still in force — the action is still a parameter call, just to a different node)
- Env does not subscribe to `/odom_wheel`; env does not bypass `ros_gz_bridge` — `adr/005-sac-env-topic-contract.md:31`

### Action space semantics

- Action is a continuous σ vector written via `robot_localization` dynamic parameters — `adr/002-ros2-system-architecture.md:41`, `adr/005-sac-env-topic-contract.md:29`. Length is phase-dependent per ADR-006 (see above).
- **σ_xxx is noise on the EKF input stream named `xxx`**, never on a raw sensor — `adr/006-ekf-input-source-phasing.md:30-31`. So σ_lidar is noise on `/odom_lidar`, σ_camera on `/odom_visual`, σ_imu on `/imu`, σ_wheel on `/odom_wheel`.

### Per-cycle control timeline

- 10 Hz / 100 ms control rate — `adr/004-agent-nav2-timeline.md:13`
- Six-step ordering inside each cycle: sensor data arrival → agent observes state → agent writes covariance → EKF tick → Nav2 publishes `/cmd_vel` → reward — `adr/004-agent-nav2-timeline.md:15-41`
- Hard invariant: covariance write (step 3) must complete before EKF tick (step 4) **within the same cycle** — `adr/004-agent-nav2-timeline.md:43`, `:47`. Applies only to agent ↔ EKF; `slam_toolbox` is async and outside this invariant — `adr/007-nav2-map-source-slam-toolbox.md:38-39`.
- Reward (sim-only, MVP): `reward = -|EKF_pose - Gazebo_ground_truth| * 10.0` — `adr/004-agent-nav2-timeline.md:39-40`. Reward is on the EKF's **filtered pose**, not on `slam_toolbox`'s **corrected pose** — deliberate, see `adr/007-nav2-map-source-slam-toolbox.md:46`.
- Fully-async architectures (independent timers, no coordination, agent ↔ EKF) explicitly ruled out — `adr/004-agent-nav2-timeline.md:47`

### EKF tick scheduling (ADR-011)

- A coordinator wrapper node sits between EKF input streams and `ekf_node`; the EKF subscribes to the wrapper's *private* republished topics, not raw sensor topics — `adr/011-ekf-tick-scheduling.md:17-22`
- Buffering policy is latest-wins per stream; release is triggered by an agent service call after the `set_parameters` ack — `adr/011-ekf-tick-scheduling.md:20-22`, `:38`
- Agent's per-cycle protocol: observe → `set_parameters` (await ack) → wrapper release → read `/odom` — `adr/011-ekf-tick-scheduling.md:26-29`
- ADR-004:43 and :47 are preserved verbatim; no supersession of ADR-004 — `adr/011-ekf-tick-scheduling.md:31`, `:39`
- Wrapper interface scales with ADR-006 phases (one subscription/republish pair per new EKF input stream); coordination protocol is phase-invariant — `adr/011-ekf-tick-scheduling.md:40`
- Cycle-skip semantics (what to do when a stream has no fresh message) are deferred to ADR-012, not resolved here — `adr/011-ekf-tick-scheduling.md:38`, `:54`

### SAC observation vector (ADR-008)

- 15-dim handcrafted feature vector, phase-invariant base — `adr/008-sac-observation-vector.md:15`
- Block A: lidar quality (mean-/max-sector variance, blockage ratio, valid-return ratio); 8-sector binning — `adr/008-sac-observation-vector.md:17-24`
- Block B: camera quality (mean luminance, log-Laplacian variance) — `adr/008-sac-observation-vector.md:26-31`
- Block C: IMU angular velocity + linear acceleration; orientation excluded — `adr/008-sac-observation-vector.md:33-40`
- Block D: planar EKF pose covariance (σ_xx, σ_yy, σ_yaw); z/roll/pitch and off-diagonals excluded — `adr/008-sac-observation-vector.md:42-50`
- `/ground_truth_pose` is structurally excluded from the observation path — `adr/008-sac-observation-vector.md:52-54`
- `/odom` covariance is meaningful only post-cutover (`/odom` → EKF, wheel-odom → `/odom_wheel`) — `adr/008-sac-observation-vector.md:61`
- ADR-009 confirmed σ-in-obs is **excluded**; observation stays 15-dim, phase-invariant — `adr/009-sac-action-space.md:29-31`

### SAC action space (ADR-009)

- Action is `Box(low=-1, high=1, shape=(N,))` per phase (N=2/3/4); SB3 SAC's tanh-squashed Gaussian maps natively — `adr/009-sac-action-space.md:17`
- Each action element is **log-σ**: env applies `σ = exp(a · 3)`, giving σ ∈ [0.05, 20] — three orders of magnitude with uniform log-space exploration — `adr/009-sac-action-space.md:19`
- σ → matrix: scalar multiplier on the diagonal of each stream's noise covariance; off-diagonals zero; nominal diagonal lives in `config/ekf_phase{1,2,3}.yaml` and is invariant during training — `adr/009-sac-action-space.md:25-27`
- Observation is `Box(low=-1, high=1, shape=(15,))` with per-feature normalisation inside env (table) — `adr/009-sac-action-space.md:35-48`
- Phase transitions: action-head new dim → 0 mean / 0 log_std; Q-head new weight → 0; **discard** replay buffer; 0.5× LR warmup for first 10k steps then linear ramp — `adr/009-sac-action-space.md:54-57`
- σ floor at ≈ 0.05 is implicit via the bounded log-space; no explicit clamp logic — `adr/009-sac-action-space.md:21`

### `set_parameters` call mechanics (ADR-012)

- Per-cycle deadline for the `set_parameters` ack is **50 ms** (tunable starting value) — `adr/012-set-parameters-call-mechanics.md:17-19`
- Timeout behaviour: skip cycle, drop transition, WARN log; from SAC's perspective the failed cycle is invisible — `adr/012-set-parameters-call-mechanics.md:23-30`
- Stream-staleness: wrapper releases latest-buffered every cycle; no cycle-skip; `robot_localization` handles temporal alignment via timestamps — `adr/012-set-parameters-call-mechanics.md:34-42`
- Streams that have *never* produced a message are omitted from the wrapper release; the EKF tolerates missing inputs — `adr/012-set-parameters-call-mechanics.md:38`
- Infrastructure failures (wrapper unreachable, `/odom` silent) terminate the episode with `truncated = True` — `adr/012-set-parameters-call-mechanics.md:46-52`
- The 50 ms deadline is a config tunable, not an architectural invariant; adjusting does not require a new ADR — `adr/012-set-parameters-call-mechanics.md:19`, `:61`

### Episode design (ADR-010)

- Episode length is **600 steps / 60 s** at 10 Hz — `adr/010-episode-design.md:17`
- Termination conditions: collision (lidar < 0.15 m) or EKF divergence (>2.0 m) → `terminated=True`; 600-step timeout or ADR-012 infra failure → `truncated=True` — `adr/010-episode-design.md:19-30`
- Phase-advancement convergence criterion: rolling 50-episode mean reward > −5.0 AND 100-episode improvement rate < 5%, both holding for ≥50 consecutive episodes — `adr/010-episode-design.md:34-40`
- Map persistence: **rebuild each episode**; slam_toolbox starts empty on every reset; aligns with the project's research thesis — `adr/010-episode-design.md:45-47`
- Curriculum: ADR-006 phase rollout only; no goal/environment curriculum — `adr/010-episode-design.md:51`
- Gazebo reset: soft reset via `/world/<name>/control` (`reset_simulation`); ~1–2 s cost — `adr/010-episode-design.md:55-60`
- Reset path is sequenced: Gazebo reset → slam_toolbox map clear → wait for first valid `/odom` → emit initial obs — `adr/010-episode-design.md:73`

### Sensor message types

- `/lidar` uses `sensor_msgs/msg/LaserScan` (not `PointCloud2`); LDS-02 is a 2D planar lidar; do not retype in place — `adr/003-lidar-laserscan-message-type.md:13`, `:17`, `:20`

## 3. Observation / reward topic mapping (per ADR-005:17–23)

Three-way split. Note that an earlier framing collapsed `/odom` into "observation only" — that was wrong.

| Topic | Observation? | Reward? | Notes |
|---|---|---|---|
| `/lidar` | yes | no | quality features (variance, blockage, valid-return) — `adr/008-sac-observation-vector.md:19-26` |
| `/camera/rgbd/image` | yes | no | quality features (luminance, log-Laplacian variance) — `adr/008-sac-observation-vector.md:28-33` |
| `/imu` | yes | no | angular velocity + linear acceleration; orientation excluded — `adr/008-sac-observation-vector.md:35-41` |
| `/odom` | **yes** (covariance fields → `ekf_cov_*`) | **yes** (filtered-pose minuend) | EKF-owned post-cutover, per ADR-002 |
| `/ground_truth_pose` | **no — hard invariant** | yes (pose subtrahend) | sim-only; leaking it into the state trivializes Sim-to-real |
| `map → odom` TF | no | no | `slam_toolbox`-published correction; deliberately excluded from reward (ADR-007:46) |

## 4. Open decisions blocking implementation

All architectural ADRs are now closed (ADRs 008–014 written). The remaining open items are config/package choices made at implementation time, not architectural decisions.

### Implementation-grade open items (no ADR planned)

These were opened by ADRs 006/007 but don't warrant their own ADRs — they are config/package decisions to make at implementation time. Listed here so they aren't lost.

- LiDAR-derived odom package choice — `rf2o_laser_odometry`, `slam_toolbox` lidar-odom output, or another. See `adr/006-ekf-input-source-phasing.md:55`.
- RGB-D visual odom package choice — `rtabmap_odom` or another. See `adr/006-ekf-input-source-phasing.md:56`.
- Whether `slam_toolbox`'s scan-matching output could feed the EKF as an input stream — see `adr/007-nav2-map-source-slam-toolbox.md:50`. **Deferred 2026-05-08** until Phase 1 empirical data informs the choice. Constraints surfaced during the deferral discussion (so the next session doesn't re-derive them): (1) slam_toolbox does not publish `nav_msgs/Odometry` natively — any path needs an adapter; (2) wrapping the existing `map → odom` TF as that adapter source is a circular-dependency trap because slam_toolbox already consumes the EKF's pose as its motion prior (`adr/007-nav2-map-source-slam-toolbox.md:20`); (3) breaking the circularity requires inverting ADR-007's "EKF feeds slam_toolbox motion prior" decision. Could become an ADR when revisited.
- Ratified topic names for `/odom_lidar` and `/odom_visual` (proposed in ADR-006, not chosen).
- `slam_toolbox` parameter values — resolved at v1 in `src/rl_navigation_pkg/config/slam_toolbox_params.yaml` (Phase 1 verified 2026-05-09). Tuning-grade per ADR-007:63 — adjust without an ADR. The TB3-Burger-specific values that diverge from upstream defaults: `minimum_travel_distance:0.1`, `minimum_travel_heading:0.1` (TB3 moves in small increments), `max_laser_range:8.0` (LDS-02 spec), `enable_interactive_mode:false` (training has no GUI hooks).
- Nav2 BT-XML defaults — `default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml` are intentionally **omitted** from `nav2_params.yaml` so Nav2's package-shipped default is used. Setting them to `""` or `"$(find-pkg-share ...)"` both fail (see CLAUDE.md Known gotchas). Tuning-grade.

## 5. Known terminology pitfalls

These have been wrong in earlier framings inside this project. Future contributors (including future Claude sessions) should watch for them.

### "EKF input source" ≠ "raw sensor"

`robot_localization`'s `ekf_node` fuses odometry / IMU / pose messages — never raw `LaserScan` or `Image`. So "the EKF fuses LiDAR" is shorthand for "an unstated `lidar_odom_node` consumes `/lidar` and feeds odometry to the EKF." Likewise σ_xxx is the noise on the *derived* odometry stream, not on the raw sensor.

### Three-way obs/reward split, not two-way

`/odom` does double duty — it is an observation source (`ekf_cov_*` features extracted from its covariance field) **and** a reward source (the filtered-pose minuend in the error term). Don't collapse it into one or the other. See ADR-005:22 and §3.

### `/ground_truth_pose` is reward-only — hard invariant

Leaking ground truth into the agent's state vector trivializes Sim-to-real transfer. The observation-vector ADR (ADR-008) must enforce this exclusion explicitly.

### Filtered pose vs corrected pose — not interchangeable

"Filtered pose" = EKF output in the `odom` frame. "Corrected pose" = post-`slam_toolbox` transform application, in the `map` frame. Reward is on the **filtered** pose (ADR-004:39-40); Nav2 plans on the **corrected** pose via TF (ADR-007:21, `:46`). Don't blur them in implementation, logging, or future ADRs.

### Wheel odom is now an EKF input from Phase 1 (ADR-006)

The earlier ambiguity is resolved: wheel odom (`/odom_wheel`) is in the EKF input set from Phase 1 onward per `adr/006-ekf-input-source-phasing.md:21-25`. ADR-002:43's three-input enumeration is partially superseded — read it together with ADR-006, never alone.

### Action-vector length is phase-dependent (ADR-006)

Phase 1 length 2 `(σ_wheel, σ_imu)`, Phase 2 length 3 `(σ_wheel, σ_imu, σ_lidar)`, Phase 3 length 4 `(σ_wheel, σ_imu, σ_lidar, σ_camera)`. ADR-002:41's "continuous 3-vector" is partially superseded. The Phase N action vector is a strict prefix of Phase N+1's, by design.

### `/odom_wheel` is now committed; `/odom_lidar` and `/odom_visual` are still proposed

ADR-006 commits to `/odom_wheel` for the renamed wheel-odom topic. The Phase 2 and Phase 3 topic names (`/odom_lidar`, `/odom_visual`) are *proposed* there but explicitly not ratified — see `adr/006-ekf-input-source-phasing.md:33-34`. Pick and document before each phase brings the corresponding stream online.

### `MyEnv` (`RLNavigation-v0`) is the *current* live env, not dead code

`MyEnv` remains the runnable smoke-test scaffold until `RLNavigation-v1` runs end-to-end (ADR-005:47). Do not delete during the SAC build-out; cut it over only when the new env is verified working.

### ADR-001 is superseded but still describes today's MVP env

ADR-001 is marked `Superseded by ADR-005`. Read it for what `MyEnv`/`v0` does today; do not read it as the trained-system contract.

### ADR-002 is partially superseded — read with ADR-006

ADR-002 is still the topology source of truth, but its EKF-input enumeration (`:43`) and action-vector length (`:41`) are partially superseded by ADR-006. Always cite ADR-006 alongside ADR-002 for any input-set or action-cardinality claim.
