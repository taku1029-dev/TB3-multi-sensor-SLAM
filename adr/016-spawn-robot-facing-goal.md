# ADR-016: Spawn the robot facing its goal at reset

## Status

Accepted. Partially supersedes `adr/010-episode-design.md:73` (the "fixed start pose" line) and refines `adr/013-nav2-planner-controller-selection.md` (RPP selection's implicit assumption that the controller would handle any robot-to-goal orientation at reset).

## Context

`adr/010-episode-design.md:73` specifies a fixed start pose at each episode reset, with the spawn pose treated as a single config value. Until 2026-05-12 the env implemented this with identity quaternion in both `gz set_pose` and the EKF `/set_pose` snap — the robot was always respawned facing +x.

`adr/013-nav2-planner-controller-selection.md` selected `NavfnPlanner` + `RegulatedPurePursuitController` (RPP) over DWB for determinism (training stability + sim-to-real). RPP's documented behaviour is "regulated pure pursuit": it follows a lookahead point that walks along the planned path. RPP does **not** drive in reverse by default and its rotation-from-rest behaviour is limited compared to e.g. DWB's full trajectory rollouts.

The first inference rollout of the 100k-trained policy (2026-05-12, 4 ckpts × 5 episodes) appeared to show a **strongly trending learning curve**: survival-to-timeout rose 60% → 80% → 100% → 80% across checkpoints, with stable -70 to -77 reward on surviving episodes — interpreted as "policy is usable, not yet converged."

The `failure_analysis` script (added in the same session — see `src/rl_navigation_pkg/rl_navigation_pkg/agents/failure_analysis.py`) consumed the per-episode CSVs and revealed:

- **13 of 20 "healthy timeout" episodes had robot-movement < 0.01 m** over the full 60 s episode. The robot was stuck at origin; reward ~-76 reflects 600 × 0.01 m × 10 = ~-76 from `|EKF − GT|` staying near zero only because GT also stayed at origin.
- All 13 stuck episodes had goals requiring a turn of more than ~50° from +x (both x<0 goals and +x-but-large-|y| goals).
- The Nav2 launch log showed the same pattern across these episodes: `controller_server: Failed to make progress` → `behavior_server: Spin failed` → `behavior_server: wait completed` → retry, with `bt_navigator: Goal canceled` only when the env's `cancel_active_goal()` fired at episode boundary 60 s later.

In short: RPP cannot execute paths that start with a large in-place turn from rest, so any goal that's not approximately forward of the robot's spawn orientation leaves the robot stationary. The env counted these as "healthy timeouts" because there was no infra failure, no collision, and no EKF divergence — `|EKF − GT|` stayed near zero because nothing moved. SAC's gradient during training was dominated by these stationary-robot episodes, which neither tested nor trained the σ-tuning behaviour the project actually needs.

A second-pass inference in the corrected env (this ADR's decision applied) measured the trained policy's **true** survival as **5%** (1/20 episodes barely survived; 19 diverged in 8-18 seconds).

## Decision

At `RLNavigation-v1.reset()`, sample the random goal **first**, compute the spawn heading from the spawn point (origin) to the goal, and apply it everywhere the robot's pose is set:

```python
goal_xy = self._sample_random_goal()
spawn_yaw = atan2(goal_xy[1], goal_xy[0])
self._node.cancel_active_goal()
self._gz_reset_world(yaw=spawn_yaw)          # gz set_pose orientation
self._node.reset_ekf_pose(yaw=spawn_yaw)     # /set_pose snap to EKF
self._drop_buffers()
self._wait_for_initial_obs(RESET_TOPIC_WAIT_S)
self._node.send_goal(*goal_xy)               # the same pre-sampled goal
```

The yaw → quaternion conversion is `qz = sin(yaw/2), qw = cos(yaw/2)` (rotation about z only; z-axis up by ROS convention). The fix is applied in `src/rl_navigation_pkg/rl_navigation_pkg/envs/sac_env.py`'s `reset()`, `_gz_reset_world(yaw)`, and `reset_ekf_pose(yaw)`.

Mid-episode goal resampling (`_send_random_goal` called from `step()` when the previous Nav2 goal completes) does **not** re-set the robot's orientation. The robot is moving when this fires, so it has both heading and momentum — RPP's stall mode is specifically rest + large turn, which is not the mid-episode condition.

## Consequences

- **ADR-010:73's "fixed start pose"** is no longer fixed. The spawn position remains the same (origin), but the spawn orientation is now a per-episode function of the goal sampler. This is consistent with ADR-010's spirit (curriculum stays out of episode bounds) — the spawn yaw is not a curriculum parameter, it's a precondition for the navigation task being well-posed under RPP.
- **ADR-013 RPP selection survives unchanged.** This ADR resolves the RPP-reset interaction without revisiting RPP's training-stability + sim-to-real rationale.
- **`final.zip` and all 10 mid-training checkpoints from the 100k run** (under `./sac_runs/phase1_001/`) were trained in the broken env where ~65% of episodes had a stationary robot. The trained policy's σ choices reflect a stationary-robot data distribution, not a moving-robot one. Per the 5% true-survival measurement, the policy is unusable in the corrected env. **Phase 1 must be retrained from scratch.**
- **Apparent training metrics** (`ep_rew_mean -550 → -210` over 100k steps) reflected SAC learning to avoid σ values that confused the EKF on a stationary robot — not σ-tuning for a moving robot. These metrics no longer describe the system's progress on the actual task.
- **The convergence criterion in `adr/010-episode-design.md:34-40`** (rolling-50-ep mean reward > -5.0) was calibrated against the broken env where stationary "healthy timeouts" scored -76. With the yaw-fix env producing real motion and 5-second-to-divergence dynamics, the reward distribution is fundamentally different. The threshold should be re-derived empirically after one retrain run.
- **Sim-to-real transfer** needs an analogue of the spawn-yaw step on the physical robot. Options:
  1. Manual pre-rotation before each real-robot evaluation episode (operationally cheap, but doesn't address autonomous deployment).
  2. A wrapper node that rotates the robot toward the goal before forwarding the `NavigateToPose` action to Nav2 (closest to the sim behaviour).
  3. Train a policy that handles non-forward starts by relaxing this ADR for a fraction of training episodes (more general but harder to learn).
  This question is not blocking Phase 1 and is deferred until the retrained policy is being prepared for real-robot deployment.

## Alternatives considered

- **Configure RPP to handle in-place turns** (`allow_reversing: true`, smaller `min_lookahead_dist`, etc.). Rejected: changes ADR-013's parameter set and trades determinism for behaviour we don't control well. Reset-time yaw fix is single-line at the env and doesn't touch the controller.
- **Switch planner to `SmacPlannerHybrid`** which outputs paths with explicit in-place rotations. Rejected: ADR-013 selected `NavfnPlanner` for determinism + simplicity; SmacHybrid is heavier and has its own tuning surface.
- **Switch controller to `DWBLocalPlanner`** which has rotation behaviour built into its cost function. Rejected: ADR-013 chose RPP over DWB for training stability + sim-to-real determinism; would re-open a closed decision.
- **Pre-rotate via direct `/cmd_vel` publish** before sending the goal to Nav2. Rejected: bypasses Nav2's velocity ownership (ADR-002:42 — Nav2 owns `/cmd_vel` post-cutover); a fragile workaround that conflicts with the controller server.
- **Constrain the goal sampler to +x half-plane only**. Rejected: trivialises the navigation problem, biases the robot's spatial experience asymmetrically, and doesn't address the underlying RPP-from-rest limitation.

## Open questions

Deferred, not blocking this ADR:

- **Mid-episode resample yaw.** When Nav2 reports the current goal complete and `_send_random_goal` fires a fresh goal, the robot's current heading may not be toward the new goal. Empirically the robot has momentum at this point and RPP seems to manage, but this has not been verified by data — the post-fix inference data shows almost no mid-episode resamples (19/20 episodes diverge before the first goal completes). Revisit if Phase 1 retraining shows mid-episode stalls.
- **Real-robot deployment yaw step** (see Consequences). Options enumerated; pick at sim-to-real time.
- **Whether to surface `spawn_yaw` in the observation vector** (ADR-008's 15-dim base). The policy might benefit from knowing the goal direction since σ choices could be heading-conditional (e.g., during a long straight drive vs frequent turns). Out of scope for Phase 1; ADR-008 explicitly excludes goal-direction features for a reason (sim-to-real generalisation), and adding spawn_yaw would partially undo that.
