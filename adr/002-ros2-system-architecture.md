# ADR-002: ROS 2 system architecture

## Status

Accepted. Supersedes the original `002-rl-env-action-space-mvp.md` (Discrete(4) navigation action space for DQN), which is no longer the project's direction.

## Context

The project's research thesis is that an RL agent dynamically adjusts EKF sensor covariance to overcome static-EKF limitations. For that signal to be well-posed, the agent must do *only* covariance tuning — not also learn to drive. That implies the runtime hosts four cooperating components: the Gazebo simulator, an EKF that fuses LiDAR + Camera + IMU, a navigation stack that drives the robot from the EKF's filtered pose, and the RL agent that tunes the EKF in the loop. Without this written down, every future session re-litigates whether the agent should drive, where the EKF lives, or whether Nav2 is in scope.

## Decision

The ROS 2 runtime topology, with the agent restricted to covariance tuning and Nav2 handling motion:

```
┌─────────────────────────────────────────────┐
│              ROS 2 system                   │
│                                             │
│  ┌──────────┐      ┌──────────────────────┐ │
│  │  Agent   │      │   Nav2 Stack         │ │
│  │  (SAC)   │      │   (fixed config)     │ │
│  │ covariance│     │  - global planner    │ │
│  │   only    │     │  - local planner     │ │
│  └────┬─────┘      └──────────┬───────────┘ │
│       │ σ_lidar                │ /cmd_vel   │
│       │ σ_camera               │            │
│       │ σ_imu                  │            │
│       ↓                        ↓            │
│  ┌─────────────────────────────────────────┐│
│  │              EKF Node                   ││
│  │     (robot_localization package)        ││
│  │  LiDAR + Camera + IMU → pose estimate   ││
│  └─────────────────────────────────────────┘│
│                    ↓                        │
│            TurtleBot3 Burger                │
└─────────────────────────────────────────────┘
```

Concretely:

- **Agent**: `stable_baselines3.SAC` policy. Action is a continuous 3-vector `(σ_lidar, σ_camera, σ_imu)` written into the EKF's per-sensor covariance via `robot_localization` dynamic parameters. The agent does **not** publish `/cmd_vel` and does **not** subscribe to raw motor commands.
- **Navigation**: Nav2 with a fixed (non-learned) configuration. Reads the EKF's filtered pose, writes `/cmd_vel`.
- **EKF**: `robot_localization` `ekf_node` fusing LiDAR-derived odometry, RGB-D visual odometry, and IMU.

## Consequences

- The agent has a single, well-posed learning signal (SLAM accuracy), uncoupled from navigation-policy quality.
- **SAC is required** — covariance scales are continuous; DQN is no longer applicable.
- Three independent ROS 2 processes must coexist at runtime: the agent, `robot_localization`, and Nav2. A combined launch file is needed (to live in `rl_navigation_pkg/launch/`).
- Replacing Nav2 later (e.g. with a learned planner) is contained: the boundary is `/cmd_vel`. Replacing `robot_localization` with a custom EKF is similarly contained: the boundary is the filtered-pose topic plus a covariance-write API.
- ADR-001's "cmd_vel out" wording remains valid for the **MVP env smoke test** (`MyEnv` still publishes `/cmd_vel` directly) but does **not** apply to the **trained system** in this ADR — there, Nav2 owns `/cmd_vel`. This split is intentional during the MVP transition; once the SAC env replaces the MVP env, ADR-001 will be superseded.
- Because the EKF will publish a filtered odometry topic that conventionally is also called `/odom`, the existing `DiffDrive` wheel odometry (currently on `/odom`, see ADR-001) must be remapped (e.g. to `/odom_wheel`) so the EKF's output owns `/odom`. This rename is required when the EKF is wired in; flagged here so it isn't missed.

## Open questions

None remaining. The Nav2 planner / controller / behavior tree / costmap layer question raised here was resolved by [ADR-013](013-nav2-planner-controller-selection.md): `NavfnPlanner` + `RegulatedPurePursuitController` + stock BT + standard costmap layers, in a single phase-invariant `config/nav2_params.yaml`.

## Alternatives considered

- Single SAC agent doing navigation + covariance tuning — rejected: two coupled learning signals, much harder to train, dilutes the research contribution.
- Custom Python EKF inside `MyEnv` — rejected: reimplements `robot_localization` and couples filter math to learning-loop code.
- Two-stage training (navigation policy first, freeze, then covariance) — rejected for now; revisit if Nav2's behavior turns out too rigid.
- Discretized covariance bins for DQN (the original ADR-002) — rejected: continuous σ space avoids quantization artifacts and matches SAC's strengths.
