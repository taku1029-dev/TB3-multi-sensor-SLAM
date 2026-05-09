# ADR-007: Nav2 map source via slam_toolbox

## Status

Accepted. Updates ADR-002's actor count from 4 to 5 because of slam_toolbox.

## Context

ADR-002 puts Nav2 in the loop as the navigation stack reading the EKF's filtered pose and writing `/cmd_vel`, but does not specify where Nav2's global costmap gets its map from. The viable options are a pre-built static map served by `nav2_map_server`, no map at all (local-planner-only operation), or live SLAM running alongside the agent.

The project's thesis (per `CLAUDE.md` project overview — "multi-sensor SLAM system") is that the RL agent dynamically tunes the EKF *while it is performing SLAM*. A static, pre-built map would make the EKF's role as a SLAM filter vestigial — the map is already known, so filter accuracy stops mattering for the planner. A no-map setup removes the planning-quality signal that downstream behaviour (and the implicit reward landscape) depends on. Live SLAM is the only choice that keeps the research question well-posed.

## Decision

Nav2's global costmap is fed by `slam_toolbox` running in `online_async` mode — concurrent SLAM during operation, not a separate pre-mapping run.

### Data flow

- `slam_toolbox` reads `/lidar` (the raw `LaserScan` per ADR-003) and the TF tree.
- It localises against scans using the EKF's filtered pose as the motion prior — taken via the `odom → base_link` transform that `robot_localization` publishes (requires `publish_tf: true` on the `ekf_filter_node`).
- It publishes `/map` for Nav2's global costmap.
- It publishes the `map → odom` TF that corrects accumulated drift in the filtered pose. This is the "corrected pose" the planner sees, distinct from the EKF's "filtered pose" in the `odom` frame.
- Nav2 reads the TF tree (`map → odom → base_link`) for localisation; it does not subscribe to `/odom` directly.

### Configuration

- New file: `config/slam_toolbox_params.yaml`. Specific values are out of scope for this ADR (see Open questions).
- New file / change: integration into the combined bringup launch flagged as missing in `adr/005-sac-env-topic-contract.md:43-48` — `slam_toolbox` joins `spawn_robot.launch.py` + EKF + Nav2 + agent.
- One config file. `slam_toolbox` does not need per-phase variants under ADR-006: it consumes the EKF's filtered pose regardless of how many input streams the EKF fuses internally.

### Scheduling

`slam_toolbox` runs on its own internal scheduling (scan processing as scans arrive; map updates typically ~1 Hz). This is asynchronous to the ADR-004 10 Hz agent cycle and does **not** participate in the per-cycle ordering invariant from `adr/004-agent-nav2-timeline.md:43`. The agent does not interact with `slam_toolbox` directly — only Nav2 does. The ADR-004 step-3-before-step-4 rule applies only to the agent ↔ EKF interaction.

## Consequences

- **EKF must publish TF.** `robot_localization`'s `publish_tf: true` is mandatory; without it, `slam_toolbox` has no motion prior. Verify this when wiring `config/ekf_phase1.yaml` (ADR-006).
- **Two distinct poses exist in the system.** "Filtered pose" = EKF output in the `odom` frame. "Corrected pose" = post-`slam_toolbox` transform application, expressed in the `map` frame. These are distinct and must not be blurred in implementation, logging, or future ADRs.
- **The reward (ADR-004) measures filtered pose, not corrected pose.** `adr/004-agent-nav2-timeline.md:39-40` defines reward as `-|EKF_pose - Gazebo_ground_truth| * 10.0` — an EKF-frame quantity. The `map → odom` correction from `slam_toolbox` is deliberately excluded. The agent is tuning the EKF; rewarding it on the post-SLAM-corrected pose would let the agent free-ride on `slam_toolbox`'s loop closure rather than improve the filter itself. This is a deliberate scoping choice, not an oversight.
- **Map quality scales with EKF quality.** Across ADR-006 phases the EKF's filtered pose grows more accurate (Phase 2 adds LiDAR-derived odom, Phase 3 adds visual odom), so `slam_toolbox`'s motion prior improves and map quality should follow. `slam_toolbox` itself is invariant across phases, but its output is not.
- **Nav2 localises off the TF tree.** Any future change to the pose source (e.g. swapping `robot_localization` for a custom EKF) must continue to publish `odom → base_link`; the Nav2 boundary stays the TF tree, not a topic.
- **Sim-to-real parity.** `slam_toolbox` consumes `LaserScan` and works identically against simulated and real LDS-02 scans, so this layer is not a Sim-to-real porting risk.

## Open questions

Each item below is the scope of a future decision; ADR-007 does not resolve them.

- Exact `slam_toolbox` parameters — resolution, loop-closure thresholds, scan-buffer length, solver choice. Content of `config/slam_toolbox_params.yaml`.
- Map persistence across episode resets — rebuild from scratch each episode, or carry the map forward? Defer to the planned episode-design ADR. Affects training distribution and curriculum design.
- Whether `slam_toolbox`'s scan-matching odometry output could itself feed the EKF as an input stream, replacing or supplementing a separate LiDAR-odom node in ADR-006 Phase 2. Would simplify the topology if it works, but is a separate decision and depends on what `slam_toolbox` actually exposes.

## Alternatives considered

- **Pre-built static map via `nav2_map_server`** — rejected: defeats the SLAM premise of the project. The EKF's role as SLAM filter becomes vestigial when the map is fixed before training, undermining the research question.
- **No map (local planner only)** — rejected: removes the planning-quality signal that the agent's reward landscape implicitly depends on, and constrains the kinds of behaviours Nav2 can produce.
- **`slam_toolbox` in `mapping` (offline) mode** — rejected: requires a separate mapping run before training and produces a static artefact, which collapses to the pre-built-map case for the duration of any training run.
