# ADR-010: Episode design

## Status

Accepted

## Context

`adr/004-agent-nav2-timeline.md:50` explicitly deferred episode boundaries (length, termination conditions) to a future ADR. `adr/006-ekf-input-source-phasing.md:48` opened the convergence-criterion question that gates phase advancement. `adr/007-nav2-map-source-slam-toolbox.md:64` opened the map-persistence-across-resets question. Together with curriculum strategy and Gazebo reset mechanics, these are the remaining knobs needed before the SAC training script can be written.

This ADR is the last architectural decision blocking implementation. After it lands, Phase 1 bringup (DiffDrive remap, EKF config, ADR-011 wrapper, `RLNavigation-v1`, ground-truth bridge) and the training script have no design dependencies left.

## Decision

### Episode length: 600 steps (60 s)

Each episode runs 600 steps at the ADR-004 10 Hz cycle rate, giving 60 s of simulation time per episode. Long enough for EKF drift to surface meaningfully under different σ choices; short enough that the replay buffer accumulates diverse experience without being dominated by long single-episode trajectories.

### Termination conditions

| Condition | Type | Threshold |
|---|---|---|
| Collision | `terminated=True` (failure) | min lidar range < 0.15 m |
| EKF divergence | `terminated=True` (failure) | `\|EKF_pose - GT\|` > 2.0 m |
| Episode timeout | `truncated=True` | 600 steps elapsed |
| ADR-012 infrastructure failure | `truncated=True` | wrapper unreachable / `/odom` silent (per `adr/012-set-parameters-call-mechanics.md:46-52`) |

The Gymnasium `terminated` vs `truncated` distinction matters: `terminated` is a real failure signal SAC bootstraps as a terminal state; `truncated` is bounded-time and SAC bootstraps from the value function. Collision and divergence are real failures; timeout and infra issues are not.

The reward at the terminating step follows ADR-004's formula `-|EKF_pose - GT| * 10.0` — no extra terminal-state shaping. SAC's value-function bootstrapping handles the rest.

### Phase-advancement convergence criterion

Per `adr/006-ekf-input-source-phasing.md:48`, advancing Phase N → N+1 requires SAC training on Phase N to have "converged". Concretely, this means **both** of:

1. **Absolute threshold.** Rolling 50-episode mean reward > −5.0 (i.e. ≤ 0.5 m average pose error per step).
2. **Plateau detection.** 100-episode reward improvement rate < 5% (i.e. the rolling mean isn't trending up meaningfully).

Both must hold simultaneously for at least 50 consecutive episodes. The threshold value (−5.0) is a starting calibration; empirical Phase 1 bringup will inform whether to tighten or relax. Adjusting is a config-tune, not a new ADR.

The other ADR-006:46-50 conditions still apply on top of this: new EKF input stream produces valid messages in isolation, and the new Phase N+1 EKF config runs cleanly with the agent paused.

### Map persistence: rebuild each episode

`slam_toolbox` starts with an empty map on every episode reset. The robot starts at a fixed pose; Nav2 picks goals from a uniform random distribution within the world bounds. Each episode is a self-contained SLAM problem — the agent learns to tune the EKF such that the filter's contribution to map quality is good across many fresh starts.

This aligns most directly with the project's research thesis (EKF tuning matters for SLAM accuracy, not just for tracking within a known map). Carry-forward and hybrid options were considered and rejected — see Alternatives.

### Curriculum: ADR-006 phase rollout only

No additional curriculum on goals or environment complexity. ADR-006's Phase 1 → 2 → 3 sensor rollout is already a substantial curriculum on the EKF's input set. Layering goal/environment curricula on top is premature complexity that can be revisited if Phase 1 fails to converge.

### Gazebo reset: soft reset via the world-control service

Each episode reset uses Gazebo's `/world/<name>/control` service (`reset_simulation` request). This:

- Teleports the robot to its fixed start pose.
- Resets the simulation clock.
- Clears sensor histories.
- Costs ~1–2 s per reset.

slam_toolbox's map is also cleared at reset (per the rebuild-each-episode decision above) — the env calls slam_toolbox's `clear_map` service or restarts the node, depending on which is faster at implementation time.

Hard reset (kill and restart Gazebo) was rejected: ~10 s per episode × thousands of episodes is hours of pure reset cost.

## Consequences

- **`RLNavigation-v1`'s `reset()`** calls the Gazebo world-control service AND clears slam_toolbox's map AND re-seeds Nav2's goal sampler. The training script does not handle these — they live inside the env.
- **Episode boundary is at 600 steps**; the SAC training script's `max_episode_steps` (or equivalent SB3 callback) must agree.
- **Convergence is checked by the training script**, not the env. The env emits per-step rewards; the training script computes rolling means and decides when phase advancement is allowed.
- **Phase advancement is operator-gated**, not automatic. The convergence criterion is a *necessary* condition, not sufficient — operator must also verify the ADR-006:46-50 conditions before running the head-expansion procedure from `adr/009-sac-action-space.md:54-57`.
- **Random goal sampler** for Nav2 lives in the env's reset path; goal selection is *not* part of the agent's action space. Goals are uniformly random within world bounds, with rejection sampling for goals that fall inside obstacles.
- **The fixed start pose** is a config value (`config/episode_params.yaml` or similar) — not hard-coded in the env. Different worlds may want different start poses.
- **slam_toolbox map clearing** must complete before the env emits the first observation of the new episode; otherwise Block D's covariance reflects stale filter state. The reset path is therefore: Gazebo reset → slam_toolbox clear → wait for first valid `/odom` after reset → emit initial obs.

## Alternatives considered

- **Shorter episodes (300 steps / 30 s).** Rejected: insufficient time for filter drift to differentiate σ choices meaningfully — the reward signal would be too noisy across short windows.
- **Longer episodes (1200+ steps / 2+ min).** Rejected for Phase 1: long episodes risk the replay buffer being dominated by single-trajectory correlations, slowing SAC's off-policy learning. Revisit if Phase 2/3 dynamics warrant.
- **Goal-reached as a termination condition.** Rejected: the agent's task is filter tuning *during* navigation, not navigation itself. Nav2 owns goal-reaching; new goals are sampled mid-episode. There's no episode-level "success" tied to reaching a goal.
- **Reward-plateau-only convergence criterion** (no absolute threshold). Rejected: plateau detection alone can flag convergence at low absolute performance — passing phase advancement to a poorly-performing checkpoint. The threshold guard catches this.
- **Threshold-only convergence criterion** (no plateau check). Rejected symmetrically: an early-spike in performance could pass the threshold transiently. Plateau detection ensures the level is actually stable.
- **Carry-forward map persistence.** Rejected: training distribution shifts as the map matures — the agent learns against a non-stationary problem, and the EKF's contribution to map *building* is harder to isolate. Misaligned with the research thesis.
- **Hybrid map persistence (per-block resets).** Rejected: adds an extra config knob and bookkeeping complexity for marginal benefit; rebuild-each-episode is the conceptually cleanest baseline.
- **Goal curriculum** (close goals first, far goals later). Rejected for now: ADR-006's sensor curriculum is already curriculum enough. Layered curricula compound debugging difficulty.
- **Environment curriculum** (open world first, cluttered later). Rejected for the same reason; revisit only if Phase 1 fails to converge in the chosen world.
- **Hard Gazebo restart per episode.** Rejected: prohibitive reset-time cost over a multi-thousand-episode training run.

## Open questions

Deferred, not blocking this ADR:

- **Exact threshold values** (collision distance 0.15 m, divergence 2.0 m, convergence reward −5.0, plateau slope 5%) — empirical calibration at Phase 1 bringup. Config tunes, not new ADRs.
- **Goal sampler distribution** (uniform-random vs Halton sequence vs hand-curated goal set) — implementation choice; uniform-random is the starting default.
- **Whether `RLNavigation-v1`'s `reset()` should restart slam_toolbox** vs call its `clear_map` service — depends on which is faster and more reliable in practice; benchmark at bringup.
- **Fixed start pose per world** — content of the episode config; deferred to implementation.
- **World-bounds metadata** — needed by the goal sampler; should live in the world's SDF or a parallel config file.
