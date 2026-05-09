# ADR-001: RL env topic contract

## Status

Superseded by [ADR-005](005-sac-env-topic-contract.md). The contract below applied to the MVP env (`MyEnv` / `RLNavigation-v0`) where the env directly published `/cmd_vel`. Once the SAC env replaces `MyEnv`, ADR-005 is the live contract; this ADR is retained for history.

## Context

The RL agent needs a stable, explicit contract with the simulator: which ROS 2 topics it observes, which it actuates on, and where odometry comes from. Previously the xacro carried four `gz-sim-triggered-publisher-system` blocks that mapped WASD keystrokes to `/cmd_vel`, and odometry was a side effect of the `DiffDrive` plugin without an explicit topic name. Both made the contract ambiguous and risked the agent fighting a human-controlled keyboard publisher on the same topic.

## Decision

The MVP env reads from these topics and publishes to one:

| Topic | Direction | Message type |
|---|---|---|
| `/lidar` | observed | `sensor_msgs/msg/LaserScan` (see ADR-003) |
| `/imu` | observed | `sensor_msgs/msg/Imu` |
| `/odom` | observed | `nav_msgs/msg/Odometry` |
| `/cmd_vel` | actuated | `geometry_msgs/msg/Twist` |

Odometry is published by the `DiffDrive` plugin with an explicit `<odom_topic>odom</odom_topic>` so the topic name is part of the contract rather than left to plugin defaults. The four keyboard `TriggeredPublisher` plugins are removed from the xacro; manual driving during sim debugging should use `ros2 topic pub /cmd_vel ...` or a teleop tool, not the in-xacro keyboard mapping.

## Consequences

- The env has exactly one writer of `/cmd_vel`. No human/agent contention.
- Adding a new sensor to the observation space is a two-step change: bridge it in `spawn_robot.launch.py` *and* register it in `MyEnv`. Either alone is silently broken.
- The `/odom` topic name is fixed by xacro config, not by the plugin's default. Replacing odometry later (e.g. with a fused EKF estimate) means publishing onto the same `/odom` from a different source — the env doesn't change.
- Manual sim driving via WASD no longer works out of the box; that is acceptable since it was a debug aid and `cmd_vel` is trivially scriptable.

## Alternatives considered

- Keep the keyboard plugins and remap them off `/cmd_vel` for training — rejected as fragile; one missed remap reintroduces contention.
- Add a separate `gz-sim-odometry-publisher-system` plugin alongside `DiffDrive` — rejected as redundant; `DiffDrive` already integrates wheel odometry and accepts an explicit `<odom_topic>`, so a second publisher would be two sources of truth on the same channel.
