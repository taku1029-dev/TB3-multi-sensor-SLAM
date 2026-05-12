# STATUS.md

A snapshot of where the SAC-tuned-EKF SLAM project is *right now*. For the *why* behind each decision, see `adr/`. For the cold-start orientation that points at what to read first, see this file plus `CLAUDE.md`.

## 1. Current implementation phase

We are in the **Phase 1 retrain-needed phase** — a 100k-step SAC training run completed 2026-05-12 (~3 hours wall-clock) and was first evaluated as "usable, not converged" via a 4 checkpoints × 5 episodes inference rollout. A second-pass rollout in a corrected env (2026-05-12) **retracted that verdict**: the trained policy is essentially untrained, with **19 of 20 episodes diverging in 8-18 seconds** when the robot actually moves. The original apparent 85% survival rate was an artefact of a Nav2 controller stall: RegulatedPurePursuitController (ADR-013) could not execute paths that start with a large turn from rest, so any goal requiring a >~50° turn from +x (13 of 20 first-pass episodes — both -x goals and +x goals with large |y|) left the robot stationary at origin for the entire 60 s episode. The env counted these as "healthy timeouts" (reward ~-76 each — `|EKF−GT|` stayed ~0.01 m because GT also stayed at origin). SAC's gradient was dominated by these stationary-robot episodes, which neither tested nor trained the σ-tuning behaviour the project actually needs. `ep_rew_mean -550 → -210` during training reflected SAC learning to avoid σ values that confused the EKF on a stationary robot — not σ-tuning for a moving robot. The env is now fixed (sample goal first → spawn robot facing it, ADR-016 candidate); existing `final.zip` is considered degenerate and **Phase 1 must be retrained from scratch** in the corrected env. The previous training-instability metrics (`critic_loss` 15 → 130+ spikes, `ent_coef` 0.11 → 0.20, `actor_loss` 50 → 155) remain relevant *as priors* for the next iteration but no longer describe the system's progress, since the underlying task has changed.

**True survival rate (yaw-fix env, 2026-05-12): ~5%.** 4 checkpoints × 5 episodes; logs under `./sac_runs/phase1_001/inference/csv_yawfix/`. **19/20 episodes diverge** in 82-176 steps (mean ~104 steps ≈ 10 s), each crossing the 2 m `|EKF−GT|` threshold around step 42-52 (~5 s) and being terminated at step 82-176. The single surviving episode (90k ep3, goal (3.04, 2.52)) timed out at 600 steps with reward **-9846** and err_max 1.87 m — just under the divergence threshold, achieved by σ_imu μ = 16.19 (near the [0.05, 20] upper bound, effectively switching off IMU fusion). Survival across training is flat: 30k 0%, 60k 0%, 90k 20% (one barely-survived), final 0%. **No checkpoint produced a stable σ-tuning policy.** Next direction: retrain Phase 1 from scratch in the yaw-fix env, optionally with the previously-flagged cheap tweaks (σ range tightening to [0.1, 10], recent-σ in obs, reward shaping near divergence threshold) — but evaluated from-scratch, not as continuation of `final.zip`.

### What exists today

- `MyEnv` Gymnasium env — `src/rl_navigation_pkg/rl_navigation_pkg/envs/my_env.py`. Id `RLNavigation-v0`. Observation: 24-sector downsampled lidar `Box`. Action: `Discrete(4)` (forward / left / right / stop). Reward: placeholder `0.0`. Subscribes `/lidar`, `/imu`, `/odom_wheel`. Publishes `/cmd_vel`.
- `SACEnv` Gymnasium env — `src/rl_navigation_pkg/rl_navigation_pkg/envs/sac_env.py`. Id `RLNavigation-v1`, the trained-system env per ADR-005. **A-1..A-4 + SAC-4 + SAC-5 + SAC-6 complete** — env-side scaffolding for SAC training is done. Observation: 15-dim `Box(-1, 1)` per ADR-008 (lidar quality 4 + camera quality 2 + IMU dyn 6 + EKF cov 3) with per-feature normalisation per ADR-009:35-48. Action: 2-dim `Box(-1, 1)` per ADR-009 (`σ = exp(a · 3)`). Reward: `-sqrt(dx² + dy²) * 10` per ADR-004:39-40 (subtrahend path verified by `reward_probe`). `step()` runs the per-cycle protocol (ADR-011:26-29 / ADR-012:17-30): `set_parameters` to `/ekf_input_gate` with 50 ms ack deadline → `release` Trigger → 100 ms spin → obs/reward construction. Single `set_parameters` timeouts retry the same action up to 10 times (invisible to SAC per ADR-012:23-30); persistent failure → `truncated=True` with `info['truncated_reason']='set_param_unreachable'`. `/odom` not advancing during the spin → `truncated=True` with `'odom_silent'` (ADR-012:51); a **grace window** (one extra `STEP_PERIOD_S` of spin if the first window saw no fresh `/odom`) was added 2026-05-12 per `adr/015-odom-silent-grace-window.md` to absorb single-threaded-executor scheduling jitter — `/odom` publishes steadily at 10 Hz per `ros2 topic hz`, but 100 Hz IMU + camera + lidar callbacks can starve the `/odom` callback inside a 100 ms window, producing spurious `odom_silent` truncations (~40% of inference episodes pre-fix, **0% across 20 episodes post-fix**). The grace window preserves ADR-012:51's "real EKF outage" intent (truncate still fires after ~200 ms total silence). Collision (lidar < 0.15 m) and EKF divergence (|EKF − GT| > 2 m) → `terminated=True` per ADR-010:21-26. Nav2 random-goal driver per ADR-010:72: each `reset()` cancels the in-flight `NavigateToPose` goal and sends a new uniform-random one (rejection-sampled away from obstacles + spawn within `[-4, 4]^2`); `step()` re-samples mid-episode the moment Nav2 reports goal completion so the robot stays moving for the full 60-s window. **Goal-callback identity check (SAC-6e)** guards `_on_goal_response` / `_on_goal_result` so a late callback from a cancelled goal cannot flip `_goal_done=True` on the *new* episode's goal — without this, every step of episode ≥ 1 used to resample. The latest Nav2 result status (action_msgs.GoalStatus codes + sentinel codes for sent/rejected/cancel_requested) is mirrored on `info['nav_status_code']` and `info['nav_status_label']` for the offline failure-mode logger. `reset()` runs the goal-aware sequence (2026-05-12 yaw-fix, ADR-013 follow-up): **sample random goal first** → compute `spawn_yaw = atan2(gy, gx)` → cancel Nav2 goal → `gz service /world/<name>/control reset: {model_only: true}` → `gz service /world/<name>/set_pose name: "custom_robot" position:{0,0,0} orientation: yaw=spawn_yaw` (set_pose is what actually teleports the dynamically-spawned robot; the `reset:{...}` alone does not) → publish `PoseWithCovarianceStamped` with the same yaw to `/set_pose` so `ekf_filter_node` snaps its filtered pose back to the spawn (SAC-6d) → drop subscriber buffers → wait for fresh sensor refresh (5 s timeout) → send the pre-sampled goal. **Spawning the robot already facing its goal is mandatory**: without it, RegulatedPurePursuitController cannot execute paths that require a large turn from rest, leaving the robot stationary at origin for any goal that's not roughly forward of +x — observed 2026-05-12 as 13/20 episodes silently stuck at origin and counted as "healthy timeouts". The slam_toolbox lifecycle reset that ADR-010:62 specified is intentionally **skipped** per SAC-6 because slam_toolbox is no longer launched during training; the env's `_slam_full_reset` / `_slam_lifecycle_transition` helpers + `lifecycle_msgs.GetState` diagnostic logging are retained but never called from `reset()`. `close()` cancels the active goal so the robot stops when an SB3 trainer or smoke test exits.
- SAC agent — `src/rl_navigation_pkg/rl_navigation_pkg/agents/sac_trainer.py` (training loop with SB3 `SAC` + `CheckpointCallback` + TensorBoard) and `src/rl_navigation_pkg/rl_navigation_pkg/agents/sac_inference.py` (deterministic rollout from a saved checkpoint). Run via `ros2 run rl_navigation_pkg sac_trainer ...` and `ros2 run rl_navigation_pkg sac_inference --checkpoint ...`. SB3 wiring was verified 2026-05-11 with a static-robot smoke run. **First real training run (2026-05-12)**: `ros2 run rl_navigation_pkg sac_trainer --total-timesteps 100000 --save-dir ./sac_runs/phase1_001 --log-interval 1 --checkpoint-interval 10000 --learning-starts 1000`. ~3 hours wall-clock at fps≈7-8. Checkpoints saved every 10k steps; `final.zip` written at the end. Observations: `ep_rew_mean` -550 → -210 (improving direction, but oscillating), `ep_len_mean` U-shape around 130–210 (no convergence toward 600-step truncate), `critic_loss` growing with spikes to 130+ (Q-function not stabilising), `ent_coef` rising 0.11 → 0.20 (SAC demanding more exploration, not less), `actor_loss` monotonically increasing 50 → 155. Verdict: partial learning, no convergence per ADR-010:34-40. **First-pass inference (2026-05-12, pre-yaw-fix env)** evaluated 4 ckpts × 5 episodes (logs in `./sac_runs/phase1_001/inference/{30k,60k,90k,final}.log` and CSVs in `./inference/csv/`) and *appeared* to show survival-to-timeout climbing 60% → 80% → 100% → 80%. **That apparent improvement was an env artefact**: 13/20 "healthy timeouts" had the robot stationary at origin due to the RPP controller stall (see `sac_env.py` entry above). The `failure_analysis` script (see below) plus CSV gt-pose deltas revealed all 13 had robot-movement < 0.01 m. **Second-pass inference (2026-05-12, post-yaw-fix env)** (CSVs in `./inference/csv_yawfix/`) shows the trained policy's true capability: **19/20 episodes diverge in 82-176 steps** (mean ~104), single survivor (90k ep3) hit reward -9846 with err_max 1.87 m and σ_imu near upper bound. `final.zip` is therefore considered degenerate (trained on stationary-robot episodes) and Phase 1 must be retrained from scratch in the yaw-fix env. The original training-instability hypotheses (σ range too wide, 15-dim obs missing recent-σ / velocity, sparse reward signal) remain plausible directions for the retrain.
- Smoke-test entries — `src/rl_navigation_pkg/rl_navigation_pkg/nodes/env_smoke_test.py` (v0; 50 random steps), `src/rl_navigation_pkg/rl_navigation_pkg/nodes/v1_smoke_test.py` (v1; 2 episodes × 100 steps with reset between, logs σ + goal + Block-A/D each step), and `src/rl_navigation_pkg/rl_navigation_pkg/nodes/sac_smoke_test.py` (SAC wiring; 200-step learn run with `learning_starts=50`). Wired as `ros2 run rl_navigation_pkg {env_smoke_test,v1_smoke_test,sac_smoke_test}`.
- **`failure_analysis` script** — `src/rl_navigation_pkg/rl_navigation_pkg/agents/failure_analysis.py`. Reads a directory of `<ckpt>_ep<N>.csv` files produced by `sac_inference --log-dir` and prints (1) a per-episode table with goal_xy, σ statistics, err_max, step where error crossed 1 m / 2 m, dominant Nav2 status, and the status transition trace; (2) an outcome-bucket summary (count, mean steps, mean σ, mean err_max per outcome label). Stdlib-only; safe to re-run on a growing collection. Run via `ros2 run rl_navigation_pkg failure_analysis --csv-dir <dir>`. Surfaced the controller-stall stationary-robot bug (2026-05-12) by exposing `nav_status='accepted'` held for 600 steps + `err_max=0.02 m` on episodes that should have been moving.
- **End-to-end stack wired into `spawn_robot.launch.py`** — `ros2 launch custom_robot_description spawn_robot.launch.py` brings up Gazebo + bridge + `robot_state_publisher` + `ekf_input_gate` + `release_driver` + `ekf_filter_node` + a `tf2_ros` static `map → odom` identity publisher + Nav2 (controller / planner / behavior / bt_navigator + lifecycle_manager). Verified 2026-05-09: navigate_to_pose action SUCCEEDED on a (0.5, 0.0) goal (then on a slam-equipped variant). `slam_toolbox` is **not** launched during SAC training per SAC-6 (2026-05-12); the launch still imports its config and the helper plumbing in case a slam-equipped variant is needed later.
- `ekf_input_gate` node — `src/rl_navigation_pkg/rl_navigation_pkg/nodes/ekf_input_gate.py`. Phase 1 streams (wheel + IMU). σ multipliers exposed as ROS 2 parameters. Republishes onto `/ekf_in/odom_wheel` and `/ekf_in/imu` only on `/ekf_input_gate/release` Trigger service call.
- `release_driver` node — `src/rl_navigation_pkg/rl_navigation_pkg/nodes/release_driver.py`. **Interim** 10 Hz Trigger client driving the gate. Replaces an unstable shell `while true` loop. To be superseded by `RLNavigation-v1`'s `step()` per ADR-004:15-41.
- `reward_probe` node — `src/rl_navigation_pkg/rl_navigation_pkg/nodes/reward_probe.py`. **Verification-only** 1 Hz scaffold: subscribes `/ground_truth_pose` + `/odom`, computes the SAC env's planned reward (`-sqrt(dx² + dy²) * scale` per ADR-004:39-40), logs both poses + L2 + yaw error. Run via `ros2 run rl_navigation_pkg reward_probe`. Verified 2026-05-10: at rest L2=0 and frame_ids `example_world` (GT) / `odom` (EKF) confirmed; under Nav2-driven motion both pose channels track and diverge (Phase 1 EKF drift exposed); missing-side resilience verified for both topics.
- `ekf_phase1.yaml` — `src/rl_navigation_pkg/config/ekf_phase1.yaml`. Holds both `ekf_filter_node` (fuses vx/vyaw from wheel + vyaw from IMU; `publish_tf:true`; `world_frame:odom`) and `ekf_input_gate` (nominal diagonals + initial σ=1.0) parameters; ROS 2 dispatches by node name. EKF output is remapped from `odometry/filtered` to `/odom` per ADR-002:52.
- `slam_toolbox_params.yaml` — `src/rl_navigation_pkg/config/slam_toolbox_params.yaml`. `online_async` mapper config; `odom_frame:odom`, `base_frame:base_footprint`, `scan_topic:/lidar`, `max_laser_range:8.0`, `minimum_travel_distance:0.1` (TB3 Burger small steps), `map_update_interval:5.0` (= /map at 0.2 Hz, intentional), `transform_publish_period:0.02` (= map→odom TF at ~50 Hz). One file, phase-invariant per ADR-007:34. Currently unused by the launch (SAC-6), retained for future slam-equipped variants.
- `nav2_params.yaml` — `src/rl_navigation_pkg/config/nav2_params.yaml`. Plugin selection per ADR-013: NavfnPlanner + RegulatedPurePursuitController + stock BT. **Global costmap is rolling-window obstacle-only** (SAC-6 2026-05-12): no `StaticLayer`, `track_unknown_space:false`, `width/height:12`. With slam_toolbox out of the launch there is no `/map` publisher; the global planner relies on `/lidar` for obstacles and a static `map → odom` identity TF for the global frame. Restoring slam means flipping `rolling_window:false`, putting back `static_layer`, and setting `track_unknown_space:true`. Local costmap is unchanged. `bt_navigator` still has NO `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml` keys — Nav2's built-in default kicks in only when these are omitted entirely.
- Nav2 nodes (`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`) are launched explicitly (not via `nav2_bringup.navigation_launch.py`) so we can omit `velocity_smoother` / `waypoint_follower` etc. that aren't in scope. `lifecycle_manager_navigation` brings them up via `autostart:True`.
- Bridges: `/clock`, `/lidar`, `/imu`, `/odom_wheel`, `/cmd_vel`, `/model/custom_robot/joint_state`, camera topics, plus `/model/custom_robot/pose` remapped to `/ground_truth_pose` (ADR-005 reward).
- DiffDrive plugin in xacro publishes `/odom_wheel` (note: `/odom` is now EKF-owned post-cutover); explicit `<frame_id>odom</frame_id>` and `<child_frame_id>base_footprint</child_frame_id>` overrides gz-sim's model-prefix default.
- IMU sensor in xacro pinned to `<gz_frame_id>imu_link</gz_frame_id>` (100 Hz). Lidar sensor uses `type="gpu_lidar"` with `<gz_frame_id>base_scan</gz_frame_id>`; SDF schema corrected (`<samples>` plural; `<range>` is sibling of `<scan>`, not child).
- `robot_state_publisher` runs in the launch and exposes the URDF static-TF chain (`base_footprint → base_link → imu_link / base_scan / camera_link / wheel_*`) to the EKF, slam_toolbox, and Nav2.
- ADRs 001–016 (see `adr/`).

### What does not exist yet

- **SAC-7: re-enable dynamic SLAM for real-robot deployment.** SAC-6 (2026-05-12) bypassed slam_toolbox entirely for Phase 1 training — Nav2's global costmap now reads /lidar via a rolling-window ObstacleLayer and the `map → odom` TF is a static identity. This means Nav2 plans against the EKF's view of the world (no slam-correction), which is fine for training because the reward is on the EKF pose itself (ADR-007:46). For sim-to-real, however, dynamic SLAM matters: long-term EKF drift would push Nav2 plans further and further off the real-world map. Re-enabling slam_toolbox requires solving the "second lifecycle cycle fails" problem documented in SAC-5/SAC-6 (likely via `clear_changes` instead of full cleanup, or via process kill+respawn between episodes). The env still carries `_slam_full_reset` / `_slam_lifecycle_transition` helpers + GetState diagnostic logging, unused but ready for re-enablement.
- **Phase advancement automation** — per-phase LR warmup callback (ADR-009:54-57) and rolling-50-episode-mean / plateau-detection convergence checker (ADR-010:34-40). Currently operator-checked via TensorBoard. Cheap once Phase 1 empirical thresholds are pinned.
- LiDAR-derived odom node (Phase 2) and RGB-D visual odom node (Phase 3) — package choices not pinned. Phase 2/3 ekf yaml variants (`ekf_phase{2,3}.yaml`) also not written yet.
- ros_gz_bridge **service** bridging — `RLNavigation-v1.reset()` currently shells out to the `gz` CLI for the world-control and set-pose services. A persistent rclpy client (via `ros_gz_interfaces/srv/ControlWorld` once the launch bridges that service) would remove subprocess overhead from every reset.
- Combined bringup launch separating sim from algorithm — for now everything lives inside `spawn_robot.launch.py`.
- **Phase 1 retrain from scratch in the yaw-fix env.** `final.zip` and the 10 mid-training checkpoints were trained on the degenerate stationary-robot task and are unfit for continuation. The retrain itself is mechanically straightforward (`sac_trainer --total-timesteps 100000 --save-dir ./sac_runs/phase1_002 ...`). What's open is which env/reward tweaks to ship before the retrain — see next bullet.
- **Convergent Phase 1 policy.** Likely directions, ordered by cheapness, informed by the yaw-fix inference data (`./sac_runs/phase1_001/inference/csv_yawfix/`): tighten σ range from `[0.05, 20]` to `[0.1, 10]` (the surviving 90k ep3 needed σ_imu ≈ 16 — near the upper bound — to barely avoid divergence, suggesting the high end is destabilising); add recent-σ (last 1–3 actions) and base velocity to obs (rapid 10 s-to-divergence dynamics suggest per-step σ matters more than aggregates); reward shaping for divergence proximity (smooth penalty proportional to |EKF−GT|² when between 1 m and 2 m, before the hard threshold trips); simplify goal sampler (constant near-origin goal for first ~30k steps then random) so early training has a non-trivial gradient. Pick directions after one baseline retrain shows where the gradient is going.
- **Phase advancement convergence criterion needs revision.** ADR-010:34-40 set convergence as rolling-50-episode mean reward > -5.0 — that threshold was derived from the now-known-degenerate task where stationary episodes scored -76. With the yaw-fix env producing real motion and 5-second-to-divergence dynamics, the achievable reward range is very different. The criterion should be redefined after one retrain run produces empirical reward distributions.

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
- `RLNavigation-v1` `COV_MAX` (Block D normalisation ceiling) — set to `10` m²/rad² in `sac_env.py`, deviating from ADR-009:47's stated `[0, 1]` bound. Justified empirically 2026-05-11: Phase 1 EKF (wheel + IMU only, no absolute pose correction) accumulates pose covariance past 1 m² within ~30 s under default σ; with [0, 1] all of Block D saturates and provides no learning signal. ADR-009:85-86 explicitly opens σ-range tuning, and the same precedent applies to the obs cov range — tuning-grade. σ_yy still saturates at 10 (cov_yy >10 m² because TB3 wheel odom only reports vx, leaving y position unobserved by wheels) — expected behaviour, the agent's job is to learn to lower σ_imu enough to constrain it. Revisit if SAC training shows the y dimension never de-saturates.
- `RLNavigation-v1` `reset()` Gazebo soft reset — uses `reset: {model_only: true}` not `{all: true}`. Empirically 2026-05-11: `all: true` tears down sensor plugin entities (IMU and PosePublisher silently went dark; lidar and camera survived) and the new publishers were not re-discovered by existing subscribers within 5 s. `model_only` resets model poses while preserving sensors/scene/plugins. See CLAUDE.md Known gotchas.

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
