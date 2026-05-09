# ADR-008: SAC observation vector specification

## Status

Accepted

## Context

`adr/005-sac-env-topic-contract.md:15-23` enumerates the SAC env's subscriptions: `/lidar`, `/camera/rgbd/image`, `/imu`, `/odom`, and the sim-only `/ground_truth_pose`. STATUS.md §3 establishes the three-way split — `/lidar`, `/camera/rgbd/image`, `/imu`, and `/odom` are observation sources; `/ground_truth_pose` is **reward-only, hard invariant** (leaking it into state trivialises Sim-to-Real). What is *not* yet specified is which features are extracted from each topic and how they map into the SAC `observation_space`. Without this pinned, `RLNavigation-v1` cannot be written.

The agent's role per `adr/002-ros2-system-architecture.md:41` is to tune EKF input-stream noise covariances. The observation should therefore encode signals the agent needs to *decide whether each input stream is currently reliable* — sensor-quality cues, not geometric scene content. Raw arrays (full lidar ranges, raw images) would force a CNN encoder and over-couple the policy to scene geometry instead of sensor quality. This ADR commits to **handcrafted quality features**.

## Decision

The Phase 1 observation vector is **15 dimensions**, ordered as four blocks. The 15-dim base is **invariant across `adr/006-ekf-input-source-phasing.md` phases** — `/odom` output covariance is always 6×6 regardless of how many input streams feed the EKF, so Block D does not scale. ADR-009 owns the decision on whether to append a phase-dependent σ block (length 2/3/4) on top of this base.

### Block A — lidar quality (4 dims)

Operating on `/lidar` (`sensor_msgs/msg/LaserScan` per `adr/003-lidar-laserscan-message-type.md:13`), with 8-sector binning under the hood:

1. **Mean-sector variance** — average of per-sector range variance across the 8 sectors. Overall return noise.
2. **Max-sector variance** — worst sector's range variance. Captures localised noise the mean would hide.
3. **Blockage ratio** — fraction of returns at `min_range` or `max_range`. Saturation indicates occlusion or no-return conditions.
4. **Valid-return ratio** — fraction of finite, in-range returns. Complements blockage; very low = sensor effectively dead.

### Block B — camera quality (2 dims)

Operating on `/camera/rgbd/image` (`sensor_msgs/msg/Image`):

1. **Mean luminance** — exposure quality. Mean of the Y channel after RGB→YUV conversion. Normalised to `[0, 1]`.
2. **Laplacian variance** — focus / motion-blur metric. Variance of a 3×3 Laplacian over the luminance channel, log-scaled before insertion.

### Block C — IMU dynamics (6 dims)

Operating on `/imu` (`sensor_msgs/msg/Imu`):

1. Angular velocity (x, y, z) — 3 dims.
2. Linear acceleration (x, y, z) — 3 dims.

Orientation quaternion is excluded: drift is already captured in Block D, and including the quaternion would couple observations to absolute heading — irrelevant for σ-tuning and harmful for cross-episode generalisation.

### Block D — EKF filtered-pose covariance (3 dims)

Operating on `/odom.pose.covariance` (EKF-owned per `adr/005-sac-env-topic-contract.md:22`):

1. **σ_xx** — entry [0, 0], variance of x.
2. **σ_yy** — entry [1, 1], variance of y.
3. **σ_yaw** — entry [5, 5], variance of yaw.

The `z`, `roll`, `pitch` diagonals are excluded — for a planar TurtleBot3 they're pinned by the ground and carry no useful variance signal. Off-diagonal correlations are excluded too; the diagonal-only planar subset is the minimal informative set.

### Hard invariant — `/ground_truth_pose` is excluded

Per STATUS.md §3 and `adr/005-sac-env-topic-contract.md:23`, `/ground_truth_pose` is reward-only. It must never enter the observation. The env class must enforce this structurally — no `/ground_truth_pose` reference in the observation-building code path.

## Consequences

- **`RLNavigation-v1`'s `observation_space`** is `Box(shape=(15,))` for Phase 1, ordered Block A → B → C → D. ADR-009 may extend; the 15-dim prefix stays stable.
- **Lidar binning is fixed at 8 sectors for feature extraction**, independent of the MVP env's 24-sector raw downsampling. `RLNavigation-v1` does its own binning.
- **Camera processing requires OpenCV** inside the env for RGB→YUV conversion and Laplacian filtering. Already implied by `CLAUDE.md`'s "Software Stack" line.
- **Block D depends on `/odom` being EKF-owned.** Pre-cutover (DiffDrive still on `/odom` per `adr/005-sac-env-topic-contract.md:48`), the covariance fields are wheel-odom estimates, not filtered-pose estimates — Block D is meaningful only after the rename to `/odom_wheel`. Phase 1 bringup must land the cutover before training starts.
- **Normalisation bounds** (`Box(low, high)`) are deferred to ADR-009 — coupled to action bounds since both feed SAC's policy network.
- **Phase transitions do not change Block A–D layouts.** The action-head expansion in `adr/006-ekf-input-source-phasing.md:54-58` requires no observation-head change unless ADR-009 opts in to per-phase σ-in-obs.

## Alternatives considered

- **Raw lidar ranges + raw RGB image as observation.** Rejected: forces a CNN encoder, couples the policy to scene geometry, inflates the state space, and adds nothing the handcrafted features don't already cover for σ-tuning.
- **Full 6×6 `/odom` covariance (21 unique entries).** Rejected: includes pinned dimensions and off-diagonal correlations a planar agent has no reason to learn from.
- **Full diagonal of `/odom` covariance (6 entries).** Rejected for the same pinned-dimensions reason; the 3-entry planar subset is minimal-informative.
- **IMU orientation included.** Rejected: ties observation to absolute heading; drift signal already in Block D.
- **Per-input-stream covariance entries (one per EKF input, scaling with phase).** Rejected for ADR-008's base block: would couple observation dimensionality to ADR-006 phases without concrete benefit at this stage. ADR-009 may revisit when the σ-in-obs question is decided.

## Open questions

Deferred, not blocking this ADR:

- **Normalisation bounds** for each block — owned by ADR-009.
- **Whether to append current σ vector** to the observation — owned by ADR-009.
- **Lidar sector count** — 8 chosen as a starting point; if Phase 1 training shows lidar features are uninformative, revisit as a config tweak, not a new ADR.
- **Camera resampling resolution** for the brightness/blur computation — implementation-time choice.
