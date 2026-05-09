# ADR-013: Nav2 planner / controller selection

## Status

Accepted. Closes the open question raised in `adr/002-ros2-system-architecture.md` ("Nav2 planner / controller / behavior tree / costmap layers").

## Context

ADR-002 puts Nav2 in the loop with `/cmd_vel` as its output boundary (`adr/002-ros2-system-architecture.md:42`) but does not specify *which* Nav2 plugins run inside the stack. ADR-007 names `slam_toolbox` as the map source feeding Nav2's global costmap (`adr/007-nav2-map-source-slam-toolbox.md:21`) but is silent on planner/controller selection.

The choice matters for three reasons:

1. **Reward-signal integrity.** ADR-004 defines reward as filtered-pose-vs-ground-truth error (`adr/004-agent-nav2-timeline.md:39-40`); the EKF integrates over whatever trajectory Nav2 commands. A non-deterministic or heavily randomized planner couples Nav2 stochasticity into the agent's reward signal, muddying attribution to EKF quality.
2. **Sim-to-real parity.** Plugin choices must work identically on simulated and real LDS-02 scans + TurtleBot3 Burger hardware. No learned components, no Ackermann-only assumptions.
3. **Phase invariance.** Per `adr/002-ros2-system-architecture.md:42` ("fixed config"), Nav2 plugins must not vary across ADR-006 EKF input phases. The agent's adaptation surface is the EKF; the navigation layer is held constant so phase comparisons stay controlled.

Constraints:

- Differential-drive kinematics (TurtleBot3 Burger, wheel separation 0.160 m, radius 0.033 m — `CLAUDE.md` Architecture notes).
- 2D `LaserScan` input from LDS-02 — `adr/003-lidar-laserscan-message-type.md:13`.
- Small footprint, indoor environments.
- Map from `slam_toolbox` as `nav_msgs/OccupancyGrid` on `/map` — `adr/007-nav2-map-source-slam-toolbox.md:21`.

## Decision

A single `config/nav2_params.yaml`, invariant across ADR-006 phases, with the following plugin selections:

| Layer | Plugin | Rationale |
|---|---|---|
| Global planner | `nav2_navfn_planner::NavfnPlanner` | Deterministic Dijkstra/A* on the occupancy grid. Diff-drive-agnostic. Default for indoor TB3, well-tested across the Nav2 ecosystem. |
| Local controller | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` (RPP) | Deterministic (no sampling, unlike DWB/MPPI), fast, designed for differential drive in tight indoor spaces. Minimal tuning surface. |
| Behavior tree | Stock `navigate_to_pose_w_replanning_and_recovery.xml` | No project-specific recovery behaviors needed yet. Customization deferred until a concrete failure mode demands it. |
| Global costmap layers | `static_layer` (from `/map`) + `obstacle_layer` (from `/lidar`) + `inflation_layer` | Standard Nav2 indoor diff-drive set. The `static_layer` ingests `slam_toolbox`'s live map per ADR-007. |
| Local costmap layers | `obstacle_layer` (from `/lidar`) + `inflation_layer` | Rolling window for reactive avoidance; no static layer at the local level. |

### What this ADR does *not* pin

- Specific plugin parameter values (lookahead distances, inflation radii, obstacle decay, replan rate, costmap resolution, footprint, etc.). Those live in `config/nav2_params.yaml` and are tuning-grade — adjusting them does not require a new ADR.
- The behavior-tree XML contents — the stock BT shipped with the project's Nav2 version is acceptable; following upstream changes is fine.
- Goal-pose source. How the SAC env chooses goals is decided in [ADR-010](010-episode-design.md): uniform-random within world bounds, rejection-sampled against obstacles, re-sampled mid-episode (`adr/010-episode-design.md:45`, `:72`, `:80`). This ADR specifies how Nav2 plans toward whichever goal is given.

## Consequences

- **Deterministic planning chain.** With NavfnPlanner + RPP, given the same costmap and goal, the same `/cmd_vel` sequence results. Variance in the agent's reward across episodes is attributable to EKF quality (modulo Gazebo's own determinism), not planner stochasticity. Preserves the reward-attribution invariant ADR-004 depends on.
- **No new ADR for parameter tuning.** Nav2 parameter values are config-grade, not architectural; treat changes to `config/nav2_params.yaml` like any other config edit.
- **Sim-to-real-clean.** All chosen plugins are deterministic numerical algorithms with no learned components. Porting from sim to TB3 hardware involves only the standard costmap-resolution / footprint / lookahead retuning, not architectural changes.
- **Phase-invariant by design.** Plugin selection does not change across ADR-006 phases. Tuning may still be needed if Phase 2/3 EKF outputs change Nav2's effective input quality, but plugins themselves stay fixed.
- **MPPI / DWB ruled out for the default path.** If the project ever needs adaptive velocity-window or sampling-based control (e.g. for Phase-3 dynamic obstacles), that would warrant a successor ADR with explicit reward-attribution analysis.
- **Recovery behaviors are stock.** Project-specific recovery (e.g. "trigger SAC reset on stuck") is *not* added here. If training reveals a recovery gap, write a successor ADR rather than ad-hoc-customizing the BT.

## Open questions

- **Replan rate vs ADR-004 cycle.** Nav2's global replanner runs on its own timer (typical 1 Hz); ensure it does not race the agent's 10 Hz cycle in any way that violates `adr/004-agent-nav2-timeline.md:43`. Likely fine because Nav2 reads the TF tree, not the agent's per-cycle covariance write, but worth verifying at Phase 1 bring-up.

## Alternatives considered

- **`SmacPlanner2D` instead of `NavfnPlanner`** — rejected for the default. Smac is a strong choice but its differentiator (Ackermann / Hybrid-A* support) is unused on a differential-drive TB3. NavfnPlanner is simpler and has more deployment history on this class of robot.
- **`DWBController` instead of RPP** — rejected. DWB's trajectory-sampling introduces minor non-determinism (sample selection, tie-breaking) that (a) complicates reward attribution to EKF quality during SAC training, and (b) adds a Sim-to-real porting surface (sample distributions and tie-breaking can behave differently between simulator and on-robot compute). RPP gives strictly deterministic output for the same goal + costmap state — a property the project values explicitly for training stability and Sim-to-real.
- **`MPPI` controller** — rejected. Sampling-based MPC injects exactly the planner-side stochasticity this ADR is trying to avoid in the reward signal. Also expensive on the Raspberry Pi 4 target.
- **`TEB` (Timed Elastic Band)** — rejected. Computationally heavier than RPP and brings little marginal benefit for a small-footprint diff-drive robot in indoor environments.
- **Custom behavior tree with RL-aware recovery** — deferred. No identified failure mode justifies the complexity yet; revisit if specific recovery behaviors prove needed during training.
- **Per-phase Nav2 plugin variation** — rejected as a category. Violates `adr/002-ros2-system-architecture.md:42` ("fixed config") and would make the agent's learning signal vary with Nav2 changes, defeating the controlled-experiment premise of ADR-006's phasing.
