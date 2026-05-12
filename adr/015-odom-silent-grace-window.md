# ADR-015: `odom_silent` detection grace window

## Status

Accepted. Partially supersedes ADR-012:51 (the 100 ms threshold for declaring `/odom` silent).

## Context

`adr/012-set-parameters-call-mechanics.md:51` specifies:

> `/odom` not received within 100 ms of cycle start → Terminate episode with `truncated = True`.

`RLNavigation-v1.step()` implements this by sampling `latest_odom` before a 100 ms `_spin_for(STEP_PERIOD_S)` and comparing object identity afterwards (`is odom_before`). Same object = no new `/odom` callback fired during the spin = `odom_silent` infra failure.

During the 100k-checkpoint inference rollout (2026-05-12) this detection fired spuriously in ~40% of episodes (e.g. 30k retry pre-fix: 2 of 5 episodes truncated at step 2 and step 158 with `odom_silent`; final.zip verification: truncated at step 595). Diagnostic evidence:

- `ros2 topic hz /odom` shows **10.001 Hz with std-dev 0.11 ms** — `/odom` publishing is steady, no real EKF outage.
- The env runs a **single-threaded rclpy executor** consuming ~15–20 callbacks per 100 ms window: IMU @ 100 Hz (≈10 msgs), `/lidar`, `/camera/rgbd/image`, `/odom`, `/ground_truth_pose`, plus Nav2 goal callbacks.
- `_spin_for` runs `rclpy.spin_once(timeout_sec=0.01)` in a 100 ms loop, processing one ready callback per iteration. With callbacks outnumbering iterations, the `/odom` callback can sit in the executor queue past the deadline even when the message has arrived.

The 100 ms threshold therefore conflates two distinct events: real `/odom` outage (architectural failure mode ADR-012:51 intends to catch) and callback-scheduling jitter (executor implementation detail). Distinguishing them inside the env keeps the infra-failure signal honest.

## Decision

After the nominal `_spin_for(STEP_PERIOD_S)`, if `/odom` has not advanced, spin for one additional `STEP_PERIOD_S` (~100 ms) before declaring `odom_silent`. Total worst-case detection latency: ~200 ms.

Implementation in `src/rl_navigation_pkg/rl_navigation_pkg/envs/sac_env.py`'s `step()`, immediately after `_spin_for`:

```python
if odom_before is not None and self._node.latest_odom is odom_before:
    grace_deadline = time.monotonic() + STEP_PERIOD_S
    while (self._node.latest_odom is odom_before
           and time.monotonic() < grace_deadline):
        rclpy.spin_once(self._node, timeout_sec=0.01)
odom_silent = (
    odom_before is not None
    and self._node.latest_odom is odom_before
)
```

The grace window is gated on the failed-once condition: when `/odom` arrives inside the first 100 ms (the common case) `step()` returns at the original cadence. The extra spin only fires when truncation is about to trigger.

## Consequences

- **ADR-012:51's infra-failure intent is preserved.** Real `/odom` outages — EKF process crash, gate node unreachable, `release_driver` stalled — still trip `truncated = True` once the 200 ms total window elapses.
- **Spurious `odom_silent` truncations eliminated.** Empirical: 0 occurrences across 20 inference episodes spanning 4 checkpoints, vs. ~40% pre-fix.
- **`step()` wall-clock is bimodal:** ~100 ms when `/odom` arrives inside the first window (the dominant mode), ~200 ms in the grace path. SB3 observes a slightly slower environment in the latter; no correctness implication.
- **ADR-004:13's 10 Hz cycle is a target, not an invariant in the grace path.** When the grace fires, the effective rate for that cycle drops to ~5 Hz. The agent ↔ EKF covariance-write ordering (ADR-004:43) is unaffected — `set_parameters` and release still complete before the spin.
- **Root cause (single-threaded executor) is bypassed, not fixed.** Future timing-sensitive checks added inside `step()` will face the same callback-scheduling jitter. If a second such check arises, prefer a `MultiThreadedExecutor` or per-topic callback groups over stacking grace windows.
- **Detection latency for real outages doubles** (100 ms → 200 ms). Acceptable: a real `/odom` outage causes the EKF to stop updating, which causes EKF divergence (ADR-010:21-26) within seconds — `odom_silent` is the early-warning signal, not the only catch.
- **`STEP_PERIOD_S` remains 100 ms.** ADR-004:13's 10 Hz contract is intact; only the failure-detection sampling window doubles.

## Alternatives considered

- **`MultiThreadedExecutor` with per-topic callback groups.** Rejected for now: larger blast radius (callback-ordering invariants, GIL contention with PyTorch inference inside SAC.predict), and grace window solves the immediate problem with five lines. Revisit if callback scheduling causes additional issues.
- **Raise `STEP_PERIOD_S` from 100 ms to 150 ms.** Rejected: still couples the spin duration to the detection threshold, and changes the 10 Hz contract (ADR-004:13) rather than refining the failure check. A 150 ms cycle that happens to be unlucky still misfires.
- **Timestamp comparison instead of object identity** (`latest_odom.header.stamp > odom_before.header.stamp`). Rejected: doesn't address the failure mode. If the callback never fires inside the window, `latest_odom` is unchanged regardless of comparison method.
- **Remove `odom_silent` detection entirely.** Rejected: real EKF outages must surface — a silent EKF gives the agent nonsensical observations and rewards, training would corrupt.
- **Larger grace (3× or 5× `STEP_PERIOD_S`).** Rejected: 2× is empirically sufficient (0 misfires in 20 episodes). Revisit if 2× proves insufficient under heavier callback load (e.g., Phase 2 adds `/odom_lidar`, Phase 3 adds `/odom_visual`).

## Open questions

Deferred, not blocking this ADR:

- **Phase 2/3 re-validation.** Adding `/odom_lidar` and `/odom_visual` callbacks will further saturate the single-threaded executor. If `odom_silent` returns under that load, the right response is `MultiThreadedExecutor` (see Alternatives), not extending the grace window.
- **Whether to log when grace fires** so the operator can monitor scheduling-jitter rates. Tuning-grade.
