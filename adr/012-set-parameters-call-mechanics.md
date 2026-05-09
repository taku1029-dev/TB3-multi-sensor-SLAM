# ADR-012: `set_parameters` call mechanics

## Status

Accepted

## Context

`adr/005-sac-env-topic-contract.md:43` specifies the agent's per-cycle action is a `set_parameters` service call to `/ekf_filter_node` with "a per-cycle deadline" — but does not pin the value. `adr/011-ekf-tick-scheduling.md` makes this load-bearing: the agent must receive the param-set ack before triggering the wrapper's release service, so the deadline directly determines whether the per-cycle protocol completes inside `adr/004-agent-nav2-timeline.md:13`'s 100 ms budget.

`adr/011-ekf-tick-scheduling.md:54` also deferred cycle-skip semantics — what to do when a stream has no fresh sensor message — to this ADR. Together with timeout behaviour and the broader failure-handling matrix, this ADR closes the per-cycle protocol's edge cases so `RLNavigation-v1`'s `step()` can be written.

## Decision

### Per-cycle deadline: 50 ms

The env awaits the `set_parameters` ack with a **50 ms timeout**. This leaves the remaining ~50 ms of the 100 ms cycle for wrapper release + EKF tick + `/odom` read + Nav2 + reward.

The 50 ms is a starting value, not an architectural constant. Phase 1 bringup may show ROS 2 service-call latency is consistently <10 ms (tighten) or routinely brushes the deadline (consider why before relaxing). Tuning is a config change, not a new ADR.

### Timeout behaviour: skip cycle

If `set_parameters` does not ack within 50 ms:

- The wrapper release is *not* triggered for this cycle.
- The EKF runs no measurement update this cycle; its predict-only step on the internal timer still fires.
- The env does *not* return from `step()` until the next cycle completes successfully — from SAC's perspective, the failed cycle is invisible (no transition recorded).
- The skip is logged at WARN level with the elapsed time, so monitoring can flag rate spikes.

Failed `set_parameters` calls reflect ROS 2 latency or service-side stalls, not the agent's σ choice. Recording them as transitions would teach SAC spurious correlations between σ and reward. Penalising via reward is misleading because the agent has no recourse against environmental latency. Skip-and-log keeps the training signal clean.

### Stream-staleness: proceed with latest-buffered, no cycle-skip

The ADR-011 wrapper always releases its buffered messages on the agent's release call, regardless of each message's age. `robot_localization`'s `ekf_node` natively handles temporal alignment via message timestamps; a stale buffered message contributes less weight relative to fresh ones, which is the correct behaviour.

Concrete policy:

- **Stream has never produced a message** — wrapper omits it from the release. The EKF tolerates missing inputs.
- **Stream produced a message at some point but is now stale** — wrapper releases the buffered copy. Staleness (age vs cycle start) is logged for diagnostics; does not gate the cycle.
- **No per-stream staleness threshold; no cycle-skip on staleness.**

Streams run at heterogeneous rates — IMU >100 Hz, wheel-odom ~50 Hz, lidar-odom ~5–10 Hz (LDS-02), visual-odom ~30 Hz (D435). Cycle-skipping on any-stream staleness would halve the effective training rate once Phase 2 brings slower streams online. Trusting `robot_localization`'s temporal handling matches how the upstream package is designed.

### Broader failure handling

| Failure | Behaviour |
|---|---|
| `set_parameters` timeout (no ack within 50 ms) | Skip cycle; drop transition; WARN log. |
| `set_parameters` rejected (service returned failure) | Skip cycle; ERROR log with rejection reason. |
| Wrapper release service unreachable | Terminate episode with `info["TimeLimit.truncated"] = True`. |
| `/odom` not received within 100 ms of cycle start | Terminate episode with `truncated = True`. |
| Sensor topic silent for the cycle (no fresh msg) | Wrapper buffered-fallback handles it; no env action needed. |

## Consequences

- **`RLNavigation-v1`'s `step()` uses an async `rclpy.parameter` client** with a 50 ms timeout, per `adr/005-sac-env-topic-contract.md:43`.
- **Skip-cycle is invisible to SAC** — `step()` simply doesn't return until a good cycle completes. The replay buffer never sees the dropped transition. SB3 needs no special handling.
- **The skip-cycle frequency is an operations signal**, not a training signal. >1% sustained suggests a system-level problem (overloaded ROS 2 graph, slow service-side handler, etc.). Monitor in the training script's logging.
- **The ADR-011 wrapper needs no staleness-threshold logic.** Proceed-with-buffered is the default. This simplifies the wrapper implementation.
- **Infrastructure-failure terminations** (wrapper unreachable, `/odom` silent) surface as truncated episodes in SAC. The reward at the terminating step is the last computed value; this is acceptable because the failures are rare and the episode resets soon afterward.
- **The 50 ms deadline is a tunable**, not an invariant. Adjusting it does not require a new ADR.

## Alternatives considered

- **Tighter deadline (e.g. 20 ms).** Rejected for the starting value: ROS 2 service-call round-trip can spike to 20–30 ms under graph load, which would cause spurious cycle-skips. Tighten only after measurement shows headroom.
- **No timeout (block until ack).** Rejected: a stalled service would freeze the cycle and break the 10 Hz rate, cascading into Nav2 starvation and EKF tick-rate drift.
- **Penalize timeouts via reward.** Rejected: the agent has no recourse against ROS 2 latency; shaping reward for environmental noise teaches spurious correlations.
- **Retry `set_parameters` once on timeout.** Rejected: a timeout already consumed half the cycle budget; retrying would push the cycle past 100 ms. Skip-and-wait is cleaner than a partial retry that pushes the rate off.
- **Cycle-skip on any stream staleness.** Rejected: heterogeneous stream rates mean staleness is the common case for slower streams, so cycle-skipping would gut training rate. `robot_localization`'s native temporal handling is the right tool.
- **Per-stream staleness thresholds with adaptive cycle-skip.** Rejected: complex, hard to tune correctly across phases, and unnecessary given the upstream filter's design.

## Open questions

Deferred, not blocking this ADR:

- **Empirical tuning of the 50 ms deadline** during Phase 1 bringup — config value, not new ADR.
- **Skip-cycle alert threshold** (1% suggested) — operations-runbook concern, not architecture.
- **Whether to emit a synthetic neutral transition on skip cycles** to keep the replay buffer balanced if skip rate is non-trivial — revisit only if monitoring shows the issue.
