# ADR-011: EKF tick scheduling mechanism

## Status

Accepted

## Context

`adr/004-agent-nav2-timeline.md:43` establishes a hard invariant: *"the agent's covariance write at step 3 must complete before the EKF tick at step 4 within the same cycle"*. `adr/005-sac-env-topic-contract.md:29` defines the agent's action as a `/ekf_filter_node/set_parameters` service call, and `adr/005-sac-env-topic-contract.md:43` requires the observation builder to wait for the parameter-set acknowledgement before the next EKF tick. `adr/004-agent-nav2-timeline.md:47` reinforces the boundary by explicitly ruling out fully-async architectures where the agent and EKF spin on independent timers without coordination.

The upstream constraint that makes this awkward: stock `robot_localization`'s `ekf_node` runs a predict step on its internal `frequency` timer and a measurement update whenever a sensor message arrives on a subscribed topic. There is no public service to trigger a tick on demand. So if sensor messages flow unrestricted into the EKF while the agent's `set_parameters` call is still in flight, the EKF can run a measurement update with stale covariance — applying the agent's intended σ only on the *next* tick. The failure is silent: per `adr/004-agent-nav2-timeline.md:9`, the policy just trains worse, with no crash and no log line pointing at the cause.

Something must therefore coordinate σ-write completion against EKF tick timing. This ADR picks that mechanism.

## Decision

A **coordinator wrapper node** sits between sensor topics and the EKF. The wrapper:

- Subscribes to every EKF input stream the current `adr/006-ekf-input-source-phasing.md` phase uses — Phase 1: `/imu`, `/odom_wheel`; Phase 2 adds `/odom_lidar`; Phase 3 adds `/odom_visual`.
- Buffers the latest message per stream (latest-wins).
- Republishes buffered messages onto **private** topics that the EKF subscribes to, *only* after the agent's `set_parameters` ack has been received in the current cycle.
- Exposes a release-trigger service that the agent calls after its `set_parameters` ack.

`ekf_node` runs vanilla on its internal timer; no fork. The agent's per-cycle protocol becomes:

1. Observe state (the agent subscribes to raw topics directly — observation does not flow through the wrapper).
2. Call `/ekf_filter_node/set_parameters`, await ack.
3. Call the wrapper's release service.
4. Read `/odom` — the EKF's filtered pose now reflects the fresh σ applied to the released measurement.

The wrapper preserves both `adr/004-agent-nav2-timeline.md:43` and `:47` strictly: the EKF sees no input until σ has been written, and the agent ↔ EKF interaction stays synchronous.

## Consequences

- **New node** under `src/rl_navigation_pkg/rl_navigation_pkg/nodes/` (proposed name `ekf_input_gate.py`). Implementation is out of scope here.
- **The EKF configs at `config/ekf_phase{1,2,3}.yaml` (per `adr/006-ekf-input-source-phasing.md:38-42`) must subscribe to the wrapper's *private* republished topics**, not the raw sensor topics. Private-topic naming is an implementation-time choice.
- **The release-trigger service is part of the per-cycle protocol.** `RLNavigation-v1`'s `step()` (per ADR-005) calls it after the `set_parameters` ack and before reading `/odom`.
- **Buffering policy: latest-wins per stream.** If a stream produced no fresh message this cycle, the wrapper holds the previous one and logs the staleness for diagnostics. Cycle-skip semantics — whether the agent should pause or proceed when a stream is stale — are deferred to ADR-012.
- **ADR-004 is unchanged.** No supersession flag is needed; both `:43` and `:47` survive verbatim.
- **The wrapper interface scales with ADR-006 phase transitions.** Each new EKF input stream adds a subscription/republish pair. The coordination protocol itself does not change across phases.
- **The wrapper is a critical-path component.** It needs its own unit tests, and its single-cycle latency must fit inside the 100 ms ADR-004 budget. If hop latency ever proves prohibitive at runtime, the fallback is documented under Alternatives.

## Alternatives considered

- **Fork & patch `robot_localization`.** Add a `/ekf_filter_node/tick` service to a forked `ekf_node` that runs one filter step on demand. Rejected: fork-maintenance burden, upstream divergence accumulates with every `robot_localization` release, and Sim-to-Real would mean a patched binary on every real-robot flash — at odds with the "use unmodified upstream" hygiene the project favours.
- **High-frequency free-running EKF with bounded skew.** Run `ekf_node` at 100 Hz; let the agent's 10 Hz σ writes apply to whichever EKF tick fires next (≤10 ms staleness). Rejected: would explicitly relax `adr/004-agent-nav2-timeline.md:47`, requiring ADR-004 to be flagged as partially superseded. `adr/004-agent-nav2-timeline.md:9` was written precisely to warn against silent stale-σ ticks; bounded staleness is the same class of bug at smaller scale. Held in reserve as the fallback if the wrapper's complexity ever becomes disproportionate.

## Open questions

Deferred, not blocking this ADR:

- **Wrapper node language** — Python (consistent with `rl_navigation_pkg`) vs C++ (lower hop latency). Decide at implementation.
- **Per-cycle deadline** for the `set_parameters` + release sequence — owned by ADR-012.
- **Cycle-skip semantics** when a stream has no fresh sensor message — owned by ADR-012.
- **Private-topic naming convention** for the wrapper's republished streams — implementation-time choice.
