# ADR-005: SAC env topic contract

## Status

Accepted. Supersedes [ADR-001](001-rl-env-topic-contract.md).

## Context

ADR-002 moved `/cmd_vel` ownership to Nav2 and `/odom` ownership to the `robot_localization` EKF, and ADR-004 specified that the agent's per-cycle action is a covariance write into the EKF (not a topic publish). ADR-001's contract — written for the MVP env where `MyEnv` published `/cmd_vel` directly — no longer reflects what the trained SAC env does. This ADR replaces it.

## Decision

The SAC env (Gymnasium id `RLNavigation-v1`, to coexist with the MVP `RLNavigation-v0` until the SAC env is end-to-end working) reads from these topics and writes via a parameter service.

### Subscriptions (env reads)

| Topic | Purpose | Type |
|---|---|---|
| `/lidar` | raw sensor signal for the state vector (variance, blockage features) | `sensor_msgs/msg/LaserScan` |
| `/camera/rgbd/image` | raw sensor signal for the state vector (brightness, motion-blur features) | `sensor_msgs/msg/Image` |
| `/imu` | raw sensor signal for the state vector | `sensor_msgs/msg/Imu` |
| `/odom` | EKF filtered pose; used for reward (vs ground truth) and for `ekf_cov_*` state features extracted from the message's covariance field | `nav_msgs/msg/Odometry` (owned by EKF per ADR-002) |
| `/ground_truth_pose` (sim-only, new bridge) | reward target | `geometry_msgs/msg/PoseStamped` (bridged from Gazebo) |

### Writes (env actuates)

| Interface | Purpose |
|---|---|
| `/ekf_filter_node/set_parameters` | dynamic parameter updates for sensor covariances `(σ_lidar, σ_camera, σ_imu)` — the agent's action, applied per ADR-004 step 3 |

The env does **not** publish `/cmd_vel`, does **not** subscribe to `/odom_wheel`, and does **not** read raw gz transport topics directly.

### Out of the env's contract (handled by other actors)

| Topic | Owner | Note |
|---|---|---|
| `/cmd_vel` | Nav2 | written based on EKF pose |
| `/odom_wheel` | DiffDrive | renamed from MVP's `/odom` (see ADR-002 consequences) |
| Lidar-derived odom, RGB-D visual odom | sensor odometry nodes feeding the EKF | env never sees these directly |

## Consequences

- **The agent's action is a parameter service call, not a topic publish.** Observation builders must wait for the parameter set to acknowledge before the next EKF tick (per ADR-004 timeline). Implementations should use `rclpy.parameter` async clients with a per-cycle deadline.
- **Add a Gazebo ground-truth pose bridge** to `spawn_robot.launch.py` (proposed topic `/ground_truth_pose`) — it doesn't exist yet because the MVP env doesn't need it. Sim-only by definition; the sim-to-real reward replacement is out of scope here (flagged in ADR-004).
- **`ekf_cov_*` state features come from the `/odom` covariance fields**, not from a separate parameter read. Cheaper per cycle and avoids racing the agent's own param writes.
- **The two-step sensor-addition rule from ADR-001 survives**: bridging the topic in `spawn_robot.launch.py` *and* subscribing in the env are both required. New EKF input sensors additionally require updating the EKF config.
- **`MyEnv` (`RLNavigation-v0`) is kept** as a runnable scaffold and smoke-test target until `RLNavigation-v1` runs end-to-end. Don't delete it during the SAC env build-out; cut it over once the new env is verified.
- **Topic-name collision risk**: Both the MVP and SAC environments name the EKF/wheel-odom topic `/odom` at different times. The cutover from MVP→SAC must include the DiffDrive remap to `/odom_wheel` (see ADR-002 consequences).

## Alternatives considered

- Publish covariance on a custom topic that an EKF wrapper subscribes to — rejected: `robot_localization` already accepts dynamic parameters; an extra wrapper is plumbing without payoff.
- Have the env subscribe to `/odom_wheel` and recompute pose error itself — rejected: defeats the purpose of putting the EKF in the loop.
- Subscribe to gz transport topics directly, bypassing `ros_gz_bridge` — rejected: ROS 2 is the integration boundary; bypassing it couples the env to Gazebo internals and breaks portability.
- Keep ADR-001 in place and add a separate "trained system topics" doc — rejected: two ADRs claiming to define the env's contract is exactly the kind of ambiguity ADRs exist to prevent.
