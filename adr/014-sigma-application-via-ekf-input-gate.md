# ADR-014: σ application via the EKF input gate wrapper

## Status

Accepted. Partially supersedes [ADR-005](005-sac-env-topic-contract.md) at `adr/005-sac-env-topic-contract.md:29` (table row naming `/ekf_filter_node/set_parameters` as the action interface) and [ADR-011](011-ekf-tick-scheduling.md) at `adr/011-ekf-tick-scheduling.md:27` (per-cycle protocol step 2). In both, the agent's `set_parameters` target is the **EKF input gate wrapper (`/ekf_input_gate`)**, not `/ekf_filter_node`. ADR-005 and ADR-011 are otherwise in force.

## Context

ADR-005:29 names `/ekf_filter_node/set_parameters` as the agent's action interface. ADR-009:25-27 specifies σ as a per-stream scalar multiplier on each stream's measurement-noise covariance diagonal, with off-diagonals zero and nominal diagonals stored in `config/ekf_phase{1,2,3}.yaml`. ADR-011 introduces the EKF input gate wrapper that buffers and releases sensor inputs to enforce the ADR-004:43 ordering invariant.

The architectural gap, surfaced when drafting `config/ekf_phase1.yaml`: stock `robot_localization` does not expose per-stream measurement-noise as a runtime-tunable parameter. It reads measurement covariances from the message itself. The closest standard parameter (`process_noise_covariance`) is process-wide, not per-stream — using it would contradict ADR-009's per-stream framing. Forking `robot_localization` to add per-stream noise parameters was rejected upfront in `adr/011-ekf-tick-scheduling.md:45` ("upstream divergence accumulates with every release; Sim-to-real would mean a patched binary on every flash").

ADR-005:52's alternatives note ("`robot_localization` already accepts dynamic parameters; an extra wrapper is plumbing without payoff") was written under the assumption that the relevant per-stream parameters exist upstream. They don't. The wrapper from ADR-011 already sits between sensor topics and the EKF, so the natural place to apply σ is at its republish step.

## Decision

The EKF input gate wrapper (`ekf_input_gate` node) owns σ application. Concretely:

- **σ values are wrapper parameters.** One `double` parameter per stream (`sigma_wheel`, `sigma_imu`, future `sigma_lidar`, `sigma_camera`). The agent updates them via `set_parameters` on `/ekf_input_gate`.
- **The agent's per-cycle protocol changes only in target.** Step 2 of `adr/011-ekf-tick-scheduling.md:24-29` becomes "Call `/ekf_input_gate/set_parameters`, await ack". Steps 1, 3, 4 unchanged.
- **σ application happens at release.** When the agent calls the wrapper's release service, the wrapper reads the current σ parameters, multiplies the nominal per-stream diagonals (from `config/ekf_phase{1,2,3}.yaml` per ADR-009:27) by σ, writes the result into the latest-buffered message's covariance fields, zeros all off-diagonal entries (per ADR-009:25), and publishes on the private topics that `ekf_filter_node` subscribes to.
- **`ekf_filter_node` runs vanilla.** No `set_parameters` call to it is made by the agent or the wrapper. `robot_localization` stays unforked, consistent with `adr/011-ekf-tick-scheduling.md:45`.
- **Naming convention.** Stream names match the σ subscripts in ADR-009 (`wheel`, `imu`, `lidar`, `camera`), not the topic names. Topic names live in `raw_topic_<stream>` / `private_topic_<stream>` parameters.

## Consequences

- **Stock `robot_localization` stays unmodified.** Sim-to-real and upstream-update hygiene preserved per `adr/011-ekf-tick-scheduling.md:45`.
- **σ application is local and unit-testable.** The wrapper's covariance-rewrite logic is a pure function of (message, σ, nominal diagonal) — independent of the EKF, easily mocked in tests.
- **The wrapper's parameter-callback latency is now on the per-cycle budget.** Verify it fits inside the 50 ms `set_parameters` deadline from `adr/012-set-parameters-call-mechanics.md:17-19`. A Python wrapper's parameter callback is typically sub-millisecond on a Pi 4, so this should be comfortable, but bench it during Phase 1 bringup.
- **The wrapper carries the nominal diagonals.** They live under the `ekf_input_gate:` block in `config/ekf_phase{1,2,3}.yaml` (already present in `ekf_phase1.yaml`).
- **Adding a Phase 2/3 stream** means: declare a new `sigma_<stream>` parameter, add `raw_topic_<stream>` / `private_topic_<stream>` params, add a sub/pub pair, and add a covariance-rewrite branch. The protocol is unchanged.
- **`adr/005-sac-env-topic-contract.md:52`** ("`robot_localization` accepts dynamic parameters") is factually incorrect for the per-stream measurement-noise case. Treat it as historical context, not a current claim.
- **Future ADRs and STATUS.md cites** must reference ADR-005:29 / ADR-011:27 *together with ADR-014*. Reading either alone gives a stale answer about the `set_parameters` target.

## Alternatives considered

- **`set_parameters` on `/ekf_filter_node` with a custom `robot_localization` fork** — rejected. Adds the fork-maintenance burden ADR-011:45 already ruled out, and the per-stream measurement-noise parameter doesn't exist upstream to set in the first place.
- **Reinterpret σ as `process_noise_covariance` modulation** — rejected. `process_noise_covariance` is process-wide (Q matrix in the EKF), not per-stream measurement noise (R matrix). Folding all streams' σ into one process-noise matrix collapses ADR-009's per-stream action structure, which is the entire point of the action space.
- **σ values as part of the release-service request payload** — rejected. Mixes data plane (covariance values) with control plane (release trigger), and breaks the ADR-005:43 invariant that the action is a parameter service call (preserved by ADR-014, just with a different target).
- **Apply σ at message receipt instead of at release** — rejected. The agent's set_parameters call must take effect on the *next* EKF measurement update (ADR-004:43). Applying σ at receipt would buffer messages with stale σ; releasing them later would feed the EKF stale-σ inputs even though set_parameters succeeded.
