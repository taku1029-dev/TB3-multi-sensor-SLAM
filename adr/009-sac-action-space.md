# ADR-009: SAC action space and observation normalisation

## Status

Accepted

## Context

`adr/002-ros2-system-architecture.md:41` and `adr/006-ekf-input-source-phasing.md:21-23` establish that the agent's action is a continuous σ vector — phase-dependent length 2 → 3 → 4 — written into the EKF's per-sensor noise covariance via `/ekf_filter_node/set_parameters` per `adr/005-sac-env-topic-contract.md:29`. What is *not* yet specified is the action `Box(low, high)`, the σ-value interpretation (linear vs log-scale), the σ → matrix mapping, the head-expansion mechanics for cross-phase checkpoint reuse, and the observation `Box(low, high)` bounds. ADR-008 also explicitly deferred the σ-in-observation question (`adr/008-sac-observation-vector.md:78`) and the observation normalisation bounds (`adr/008-sac-observation-vector.md:77`) to this ADR.

`adr/006-ekf-input-source-phasing.md:54-58` opens the head-expansion mechanics question but defers the specifics (initialization scheme, replay-buffer handling, LR schedule) to a later ADR — this one.

## Decision

### Action shape and interpretation

The action vector is `Box(low=-1, high=1, shape=(N,))` where N is phase-dependent (Phase 1: 2, Phase 2: 3, Phase 3: 4) per ADR-006. This is the idiomatic SAC range; SB3 SAC's tanh-squashed Gaussian policy outputs map natively to it.

Each action element `a` is **log-σ**: the env applies `σ = exp(a · 3)` before writing the EKF parameter, giving σ ∈ [exp(-3), exp(3)] ≈ [0.05, 20]. This covers three orders of magnitude with uniform exploration in log-space — covariances often span that range and linear-σ exploration would over-sample large σ.

The implicit floor at σ ≈ 0.05 is the EKF numerical stability guard. No explicit clamp logic is needed; the bounded log-space provides it.

### σ → matrix mapping

Each σ_xxx is a **scalar multiplier on the diagonal** of stream xxx's noise covariance matrix in `config/ekf_phase{1,2,3}.yaml`. The static diagonal pattern (which pose/twist/accel components each stream tracks, and their relative magnitudes) is set in the EKF config and is invariant during training. Off-diagonals stay zero.

The env builds the 15×15 matrix as `nominal_diagonal * σ²` and writes it via `set_parameters`. The implementation can either send the full matrix per cycle or use `robot_localization`'s parameter-update API in whatever form is cheapest; this is a downstream choice.

### σ-in-observation: excluded

The observation stays at ADR-008's 15-dim base, **phase-invariant**. SAC's Q(s, a) already conditions on action, and Block D (EKF filtered-pose covariance) gives the agent feedback on the *result* of its recent σ choices — adding raw σ to the state would be redundant. Phase transitions therefore touch only the action head, not the observation head.

### Observation normalisation bounds

The observation is `Box(low=-1, high=1, shape=(15,))` with **per-feature normalisation inside the env**. Each feature is scaled into [-1, 1] (or [0, 1], packed into the broader range) before insertion. Out-of-range values are clipped:

| Block | Feature | Source range | Normalisation |
|---|---|---|---|
| A | mean-sector variance | [0, max_range²] ≈ [0, 144 m²] | divide by max_range² → [0, 1] |
| A | max-sector variance | same | same |
| A | blockage ratio | [0, 1] | identity |
| A | valid-return ratio | [0, 1] | identity |
| B | mean luminance | [0, 1] | identity (already normalised in ADR-008) |
| B | log-Laplacian variance | typically [0, 20] | divide by 20 → [0, 1], clip |
| C | angular velocity x/y/z | [-10, 10] rad/s | divide by 10 → [-1, 1], clip |
| C | linear acceleration x/y/z | [-20, 20] m/s² | divide by 20 → [-1, 1], clip |
| D | σ_xx, σ_yy | [0, 1] m² | identity, clip |
| D | σ_yaw | [0, 1] rad² | identity, clip |

### Cross-phase checkpoint reuse

Per `adr/006-ekf-input-source-phasing.md:54-58`, training transitions Phase N → N+1 by extending the action space and fine-tuning. Concretely:

1. **Action-head init.** New action dim's policy mean output → 0 (so σ_new = exp(0) = 1, a neutral scale); policy log_std → 0 (medium exploration).
2. **Q-head init.** Q networks take (s, a) where a's dimensionality grew. Weights connecting the new action dim → 0; preserves Q-values for the old action subspace at the moment of expansion.
3. **Replay buffer.** **Discard** at the phase boundary. Phase N+1 has a different EKF input set (new sensor odometry stream entered the loop), so old transitions don't transfer cleanly even with action-padding. Re-fill the buffer with Phase N+1 data before resuming gradient updates.
4. **Learning-rate warmup.** 0.5× the default LR for the first 10 000 environment steps after head expansion, then linear ramp to full LR over the next 10 000. Lets the new action dim stabilise before the rest of the policy is updated at full pace.

## Consequences

- **`RLNavigation-v1`'s `action_space`** is `Box(low=-1, high=1, shape=(N,))` with N = 2 in Phase 1.
- **`RLNavigation-v1`'s `observation_space`** is `Box(low=-1, high=1, shape=(15,))` with the per-feature normalisation table above; observation does not change across ADR-006 phases.
- **The env's `step()` must apply** `σ = exp(a · 3)` on action before constructing the EKF noise covariance matrix and calling `set_parameters`.
- **`config/ekf_phase{1,2,3}.yaml` must define the nominal diagonal pattern** for each stream — the relative magnitudes that the agent's σ scales. This is part of those configs' implementation and must not be tuned during training.
- **Replay-buffer discard at phase boundaries** means each phase pays a fresh-buffer-warmup cost. Plan for this in the training script: at phase transitions, run pure exploration (`learning_starts` re-applied) before resuming SAC updates.
- **The 0.5× LR warmup is part of the training script**, not the env. SAC implementations using SB3 should swap in a custom LR schedule callback at phase transitions.
- **The σ → matrix mapping decision constrains the EKF config format** — diagonals only, off-diagonals zero. If a future EKF input stream needs cross-correlation in its noise prior, that's a new ADR.

## Alternatives considered

- **Linear-σ action.** Direct `Box(low=σ_min, high=σ_max)` over physical σ values. Rejected: SAC's Gaussian policy explores uniformly in action-space, which would over-sample large σ when the useful range spans orders of magnitude. Log-space exploration is more natural for covariance scales.
- **Per-dimension σ (one action per matrix entry, not per stream).** Rejected: action dimensionality balloons (e.g. 15 entries × N streams) for marginal expressiveness gain — most off-diagonal entries should stay zero in practice, and the per-stream scalar already gives the agent the magnitude knob it needs.
- **Include current σ in observation.** Rejected per the explicit decision above; SAC's Q already action-conditions, and ADR-008's Block D gives result-of-action feedback.
- **Mixed-bound observation `Box`** (some [0, 1], some [-1, 1] per feature). Rejected: layout is messier and offers no learning advantage over a single-range Box with per-feature normalisation. SB3 SAC handles either, but uniformity is easier to debug.
- **Pad replay buffer at phase boundaries** (keep old transitions with neutral-default values for new action dims). Rejected: Phase N+1's reward landscape differs from Phase N's because a new EKF input stream is now active; padded transitions teach the policy that "neutral-default σ on the new stream produced these old rewards", which is wrong. Discarding is cleaner.
- **No LR warmup at phase transitions.** Rejected: head expansion injects un-trained weights into a previously-converged policy; full-LR updates risk destabilising the converged dimensions before the new ones catch up.

## Open questions

Deferred, not blocking this ADR:

- **Concrete EKF nominal diagonals** per stream — content of `config/ekf_phase{1,2,3}.yaml`. Implementation-time choice driven by sensor specs.
- **Replay-buffer warmup size and `learning_starts` value** at phase transitions — tune empirically when Phase 2 bringup happens.
- **Whether the LR warmup should also apply to the Q-network** specifically, or just the policy — implementation-time call; SB3's default ties the two so this is likely moot.
- **σ-range tuning** — the ±3 log-space exponent (giving σ ∈ [0.05, 20]) is a starting point. If Phase 1 training shows the agent saturates at the bounds, widen; if it never approaches them, narrow.
