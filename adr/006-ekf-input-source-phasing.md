# ADR-006: EKF input source phasing

## Status

Accepted. Partially supersedes [ADR-002](002-ros2-system-architecture.md): specifically the three-input enumeration at `adr/002-ros2-system-architecture.md:43` (wheel odometry is now an explicit EKF input stream from Phase 1) and the length-3 action-vector wording at `adr/002-ros2-system-architecture.md:41` (action cardinality now varies by phase, ending at length 4 in Phase 3). ADR-002 is otherwise still in force; do not edit it in place.

## Context

ADR-002 defined the trained-system topology with the EKF fusing three input streams: LiDAR-derived odometry, RGB-D visual odometry, and IMU. The strict reading excluded wheel odometry, departing from the canonical `robot_localization` "wheel + IMU" baseline that practitioners rely on for first-light bring-up.

Going straight to a three-stream fusion also conflates many possible failure modes — xacro and bridge wiring, EKF config, the derived-odometry nodes themselves, the agent's per-cycle `set_parameters` write, and the ADR-004 timeline — into one debugging surface. We need an incremental path that lets us verify each subsystem before stacking the next on top, while still ending at the ADR-002 target topology.

## Decision

EKF input streams are introduced in three phases. IMU is present in every phase. Each phase has a fixed action-vector layout, EKF config file, and topic set. Phase advancement is gated, not automatic.

### Phases

| Phase | EKF input streams | Action vector | Goal |
|---|---|---|---|
| 1 | wheel odom (`/odom_wheel`), IMU (`/imu`) | `(σ_wheel, σ_imu)` — length 2 | Bring `robot_localization` into the loop with the canonical two-stream baseline. Verify EKF fusion mechanics, the agent's `set_parameters` write path, and the ADR-004 per-cycle timeline end-to-end before adding complexity. |
| 2 | + LiDAR-derived odom (`/odom_lidar`, proposed) | `(σ_wheel, σ_imu, σ_lidar)` — length 3 | Add an environment-anchored EKF input stream. Validate that scan-matching odometry stabilizes the filter against wheel slip. |
| 3 | + RGB-D visual odom (`/odom_visual`, proposed) | `(σ_wheel, σ_imu, σ_lidar, σ_camera)` — length 4 | Reach the full ADR-002 fusion configuration. Validate that texture-based odometry complements LiDAR in feature-poor environments. |

The action vector is ordered by the order of stream introduction, so Phase N's action prefix is identical to Phase N−1's. This keeps the head-expansion at phase transitions a strict append, not a re-layout.

### Naming and terminology

Per `CLAUDE.md` "Terminology conventions":

- σ_xxx is the noise on the EKF input stream named `xxx`, never on a raw sensor. σ_lidar is the noise on `/odom_lidar`, not on `/lidar`. σ_camera is the noise on `/odom_visual`, not on `/camera/rgbd/image`. σ_imu is the noise on the `/imu` stream that the EKF consumes directly.
- Wheel odom stays on `/odom_wheel` (per `adr/002-ros2-system-architecture.md:52` consequences). The DiffDrive `/odom` → `/odom_wheel` remap must land before Phase 1 brings the EKF online, so the EKF can own `/odom`.
- LiDAR-derived odom stream: proposed topic `/odom_lidar`. Not ratified — see Open questions.
- Visual odom stream: proposed topic `/odom_visual`. Not ratified — see Open questions.

### Per-phase EKF config

Each phase ships its own config file under `config/`:

- `config/ekf_phase1.yaml` — `odom0_config` (wheel) + `imu0_config`
- `config/ekf_phase2.yaml` — adds `odom1_config` (LiDAR-derived)
- `config/ekf_phase3.yaml` — adds `odom2_config` (visual)

The active config is selected at launch time. Bringup launch files take a phase argument; do not branch inside a single config file.

### Phase advancement criteria

Advancing from Phase N to Phase N+1 requires all of:

1. SAC training on Phase N has converged. The convergence criterion is deferred to the planned episode-design ADR.
2. The new EKF input stream is producing valid messages on its topic in isolation, verified independently of the EKF and the agent.
3. The Phase N+1 EKF config runs cleanly with the new input enabled but the agent paused (covariances at config defaults). Only after this isolation check does agent training resume.

### SAC checkpoint reuse across phase boundaries

A phase change extends the action space (length grows by 1 in transitions 1→2 and 2→3). A trained SAC checkpoint cannot be loaded directly into a model with a larger action space.

Recommended transition: train Phase N to convergence, freeze the policy and value networks, expand the action head and any Q-heads to the Phase N+1 dimension (initialize the new outputs to neutral defaults), and fine-tune. The exact mechanics — initialization scheme, replay-buffer handling, learning-rate schedule on resume — are deferred (see Open questions).

The observation-space `ekf_cov_*` features are also expected to grow per phase as new EKF input streams contribute additional covariance entries. That dimensionality is not pinned here; flagged for the planned observation-vector ADR.

## Consequences

- **Cite ADR-006 alongside ADR-002 for any input-set or action-cardinality claim.** ADR-002's input enumeration (`:43`) and action-vector length (`:41`) are now phase-dependent. Reading ADR-002 alone gives a stale answer.
- **`STATUS.md`'s "wheel odometry is excluded from EKF inputs" note is no longer true** once ADR-006 is in force. STATUS.md needs updating; out of scope for this ADR's edits, flagged here.
- **Three EKF config files** must be maintained in parallel. Drift between them is a real risk — keep them side-by-side and review Phase N+1's diff against Phase N's whenever either is touched.
- **Phase 1 is a milestone, not just a stepping stone.** Even if Phases 2–3 slip, Phase 1 alone validates the ADR-004 timeline, the `set_parameters` plumbing, and the EKF-as-`/odom`-owner cutover.
- **Bring-up risk is front-loaded into Phase 1.** The DiffDrive remap, EKF launch wiring, per-cycle timeline enforcement, and the agent's first real `set_parameters` call all happen here. Plan accordingly.
- **Phase transitions are non-trivial training events**, not just config swaps — head expansion plus fine-tuning, with replay-buffer behaviour TBD. Budget time for them.

## Open questions

Each item below is the scope of a future ADR; ADR-006 does not resolve them.

- Per-phase σ bounds — exact `Box(low, high)` values for each component. (Planned ADR-009.)
- Linear vs log-scale σ output. Covariances span orders of magnitude; log-scale is often more natural. (Planned ADR-009.)
- LiDAR-derived odom package — `rf2o_laser_odometry`, `slam_toolbox` lidar-odom output, or another. Affects `/odom_lidar`'s message rate, frame conventions, and covariance behaviour.
- Visual odom package — `rtabmap_odom` or another. Affects `/odom_visual`'s rate and covariance behaviour, and overlaps with the SLAM-source decision in planned ADR-007.
- Cross-phase SAC checkpoint reuse mechanics — head-expansion initialization, replay-buffer handling, learning-rate schedule on resume.
- Final ratified topic names for `/odom_lidar` and `/odom_visual` (proposed but not chosen here).
- Convergence criterion that gates phase advancement (planned ADR-010).

## Alternatives considered

- **Single-shot full fusion (Phase 3 only) from day one** — rejected: stacks all bring-up risks at once. A failure could be in bridges, xacro plugin config, EKF config, derived-odom nodes, agent param-write timing, or Nav2, making root-cause isolation expensive.
- **Two-phase plan (wheel+IMU → full fusion)** — rejected: the LiDAR-odom and visual-odom validation steps are independently informative, especially for assessing behaviour in feature-poor environments. Splitting them costs little and yields more diagnostic signal.
- **Drop wheel odom entirely (the strict ADR-002 reading)** — rejected: forfeits the canonical `robot_localization` baseline, makes Phase 1 simultaneously a bring-up *and* a research result, and raises the chance that early failures are misattributed.
- **Hold action-vector length fixed at 4 across all phases, masking unused components** — rejected: makes Phase 1 carry dead action dimensions the agent must learn to ignore, polluting early training. Head-expansion at phase boundaries localizes the disruption.
