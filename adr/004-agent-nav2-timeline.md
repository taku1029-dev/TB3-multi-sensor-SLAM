# ADR-004: Per-cycle control timeline

## Status

Accepted

## Context

ADR-002 introduces three independent ROS 2 actors that run concurrently in the trained system: the SAC agent, the `robot_localization` EKF, and Nav2. Without an explicit ordering inside each control cycle, it is ambiguous whether a fresh covariance update takes effect *before* the EKF tick (it must), and whether Nav2 is steering using the freshly-corrected pose (it must). Misordered updates do not crash anything — the policy just trains worse — so the failure mode is silent. Documenting the timeline keeps everyone (including future Claude sessions) from reintroducing this class of bug.

## Decision

Each control cycle (10 Hz, 100 ms) runs in this fixed order:

```
At time t (every 100 ms):

  1. Sensor data arrival
     LiDAR  → /lidar
     Camera → /camera/rgbd
     IMU    → /imu

  2. Agent observes state
     state = [lidar_variance, camera_brightness,
              ekf_cov_x, ekf_cov_y, ...]

  3. Agent updates EKF covariance
     ekf.set_covariance(σ_lidar, σ_camera, σ_imu)

  4. EKF tick with the new covariance
     publishes filtered pose (e.g. /odom — see ADR-002 on
     remapping wheel odom away from this topic)

  5. Nav2 path tracking
     reads filtered pose, publishes /cmd_vel
     → TurtleBot3 moves

  6. Agent computes reward
     odometry_error = |EKF_pose - Gazebo_ground_truth|
     reward = -odometry_error * 10.0
```

The agent's covariance write at step 3 must complete *before* the EKF tick at step 4 within the same cycle.

## Consequences

- Implementation must enforce step 3 → step 4 ordering. Practically: gate the agent's tick on a fresh sensor batch, write covariance synchronously, then run the EKF spin. This rules out fully-async architectures where the agent and EKF spin on independent timers without coordination.
- 10 Hz is the chosen rate for all three actors in the MVP. Faster cycles inflate SAC's replay buffer with redundant near-identical transitions; slower cycles delay reaction to environmental changes.
- Reward is **dense, per-step, and bounded** — well-suited to SAC's off-policy continuous-control learning. The `* 10.0` scaling is a starting value; revisit during reward tuning, but keep it documented if changed.
- Episode boundaries (length, termination conditions) are **not** decided here — they live in the training script and a future ADR on episode/curriculum design.
- Ground-truth pose is sim-only (Gazebo). Sim-to-real transfer needs a substitute reward — out of scope for MVP, flagged so it isn't forgotten.
- Decoupling the rates later (e.g. agent at 5 Hz, EKF at 30 Hz) is a contained change as long as covariance writes remain synchronous within each agent cycle.

## Alternatives considered

- Asynchronous covariance updates (agent writes whenever its tick fires; EKF spins independently) — rejected: opens a race window where the EKF runs with stale covariance, making the learning signal ambiguous.
- Reward computed only at episode end — rejected: SAC benefits substantially from dense rewards; sparse-reward continuous control is much slower to converge.
- Use EKF innovation magnitude as the reward instead of ground-truth error — rejected for MVP because Gazebo gives clean ground truth, but this is the candidate substitute when porting to real hardware.
