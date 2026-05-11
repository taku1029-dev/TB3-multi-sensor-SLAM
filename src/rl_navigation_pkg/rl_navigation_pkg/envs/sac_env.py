"""RLNavigation-v1 — SAC env for dynamic EKF covariance tuning.

Topic contract per ADR-005, observation per ADR-008, action per ADR-009,
per-cycle protocol per ADR-011, set_parameters mechanics per ADR-012,
episode design per ADR-010.

A-1 + A-2 + A-3 + A-4 + SAC-4 cut:
  - Subscribes /lidar /imu /camera/rgbd/image /odom /ground_truth_pose
  - 15-dim observation, per-feature normalised to [-1, 1] (ADR-009:35-48)
  - 2-dim action Box(-1, 1) — Phase 1 (sigma_wheel, sigma_imu)
  - step() applies σ = exp(a · 3) via /ekf_input_gate/set_parameters with a
    50 ms ack deadline (ADR-012:17-19); on success triggers
    /ekf_input_gate/release. Single timeouts retry the same action up to
    SET_PARAM_MAX_RETRIES times — invisible to SAC per ADR-012:23-30. All
    retries failing → truncated=True with info['truncated_reason']
    ='set_param_unreachable' (ADR-012:50).
  - /odom-silent detection: if latest_odom does not advance during the
    100 ms spin, truncated=True with 'odom_silent' (ADR-012:51).
  - reward = -|EKF − GT| * 10 (ADR-004:39-40, verified by reward_probe).
  - terminated=True on collision (lidar < 0.15 m) or EKF divergence
    (|EKF − GT| > 2.0 m) per ADR-010:21-26.
  - reset() runs the ADR-010:73 sequence: Gazebo soft reset (subprocess gz
    CLI, interim) → wait for full sensor refresh → emit initial obs.
  - Nav2 random-goal driver per ADR-010:72: reset() cancels any in-flight
    NavigateToPose goal then sends a fresh uniform-random goal within world
    bounds (rejection-sampled away from obstacles + spawn). step() detects
    goal completion and re-samples mid-episode so the robot stays moving
    for the full 60-s window.

Not yet wired:
  - slam_toolbox clear hook (the existing map persists across episodes;
    acceptable for Phase 1 because reward is on EKF pose, not the corrected
    pose — ADR-007:46).
  - SAC training loop / SB3 wiring lives outside this module.
"""

from __future__ import annotations

import subprocess
import time
from math import log, sqrt

import gymnasium as gym
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from gymnasium import spaces
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, LaserScan
from std_srvs.srv import Trigger

OBS_DIM = 15
ACTION_DIM = 2  # Phase 1: (sigma_wheel, sigma_imu)
LIDAR_SECTORS = 8
LIDAR_RANGE_MAX = 8.0
IMU_GYRO_MAX = 10.0
IMU_ACCEL_MAX = 20.0
LAPLACIAN_LOG_MAX = 20.0
# ADR-009:47 specifies [0, 1] m^2 / rad^2; raised to 10 because Phase 1 EKF
# (wheel + IMU only, no absolute pose correction) accumulates pose covariance
# past 1 m^2 within ~30 s under default σ. Tuning-grade per ADR-009:85-86.
COV_MAX = 10.0
REWARD_SCALE = 10.0
STEP_PERIOD_S = 0.1
MAX_EPISODE_STEPS = 600
CAM_DOWNSAMPLE_TARGET = (160, 120)  # (W, H); only applied if image is larger
ACTION_LOG_SCALE = 3.0  # σ = exp(a · 3) per ADR-009:19
SET_PARAM_TIMEOUT_S = 0.05  # ADR-012:17-19
GATE_NAMESPACE = '/ekf_input_gate'
GATE_WAIT_TIMEOUT_S = 10.0
GZ_WORLD_NAME = 'example_world'  # matches worlds/example_world.sdf <world name=...>
GZ_ROBOT_NAME = 'custom_robot'   # matches `-name custom_robot` in spawn_robot.launch.py
GZ_RESET_TIMEOUT_S = 3.0
RESET_TOPIC_WAIT_S = 5.0
COLLISION_DISTANCE_M = 0.15  # ADR-010:23
DIVERGENCE_DISTANCE_M = 2.0  # ADR-010:24
SET_PARAM_MAX_RETRIES = 10  # ~500 ms total retry budget per cycle (ADR-012:23-30)

# Nav2 random-goal driver (ADR-010:72). World bounds give 1 m wall margin from
# the 10x10 m arena in worlds/example_world.sdf. Obstacle centres + radius are
# hard-coded to match that SDF — when the world changes, update both. Goals
# closer than MIN_GOAL_DIST_FROM_SPAWN to (0,0) are rejected so trivial 0-cm
# navigation tasks don't dominate.
NAV_ACTION_NAME = '/navigate_to_pose'
NAV_WAIT_TIMEOUT_S = 10.0
WORLD_BOUND_X = (-4.0, 4.0)
WORLD_BOUND_Y = (-4.0, 4.0)
GOAL_FORBIDDEN_CENTERS = (
    (1.5, 2.0),    # pillar_ne
    (1.0, -2.0),   # pillar_se
    (-2.0, -1.5),  # pillar_sw
    (-2.0, 2.0),   # box_nw
)
GOAL_OBSTACLE_RADIUS_M = 0.5  # robot radius 0.105 + obstacle radius 0.2 + slack
MIN_GOAL_DIST_FROM_SPAWN = 1.0
GOAL_REJECT_BUDGET = 100
GOAL_FALLBACK = (2.0, 0.0)  # known-clear point if rejection sampling exhausts

# slam_toolbox lifecycle reset (ADR-010:62 — rebuild map each episode).
# slam_toolbox does not expose a clear_map service, so we cycle the lifecycle:
# active → inactive → unconfigured → inactive → active rebuilds the pose graph
# and the occupancy grid from scratch.
SLAM_LIFECYCLE_NAMESPACE = '/slam_toolbox'
SLAM_LIFECYCLE_TRANSITION_TIMEOUT_S = 3.0
SLAM_LIFECYCLE_RESET_SEQUENCE = (
    Transition.TRANSITION_DEACTIVATE,
    Transition.TRANSITION_CLEANUP,
    Transition.TRANSITION_CONFIGURE,
    Transition.TRANSITION_ACTIVATE,
)


class _V1Node(Node):
    def __init__(self) -> None:
        super().__init__('rl_navigation_v1_env')
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.latest_scan: LaserScan | None = None
        self.latest_imu: Imu | None = None
        self.latest_image: Image | None = None
        self.latest_odom: Odometry | None = None
        self.latest_gt: PoseStamped | None = None

        self.create_subscription(LaserScan, '/lidar', self._on_scan, sensor_qos)
        self.create_subscription(Imu, '/imu', self._on_imu, sensor_qos)
        self.create_subscription(Image, '/camera/rgbd/image', self._on_image, sensor_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(PoseStamped, '/ground_truth_pose', self._on_gt, 10)

        # ADR-014: σ application target is /ekf_input_gate, not /ekf_filter_node.
        # ADR-011:26-29: set_parameters → release per cycle.
        self.set_param_client = self.create_client(
            SetParameters, f'{GATE_NAMESPACE}/set_parameters'
        )
        self.release_client = self.create_client(Trigger, f'{GATE_NAMESPACE}/release')

        # ADR-010:72 random-goal driver. State machine:
        #   _send_future is set → callback consumes it, sets _result_future
        #   _result_future is set → callback fires _goal_done = True
        # _goal_done=True means step() should resample and call send_goal again.
        self.nav_client = ActionClient(self, NavigateToPose, NAV_ACTION_NAME)
        self._send_future = None
        self._result_future = None
        self._goal_handle = None
        self._goal_done = True  # no goal in flight at startup
        self._current_goal_xy: tuple[float, float] | None = None

        # ADR-010:62: rebuild slam_toolbox each episode via lifecycle.
        self.slam_state_client = self.create_client(
            ChangeState, f'{SLAM_LIFECYCLE_NAMESPACE}/change_state'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _on_imu(self, msg: Imu) -> None:
        self.latest_imu = msg

    def _on_image(self, msg: Image) -> None:
        self.latest_image = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _on_gt(self, msg: PoseStamped) -> None:
        self.latest_gt = msg

    def send_goal(self, x: float, y: float) -> None:
        """Send a NavigateToPose goal. Fire-and-forget; completion is observed
        via the future-callback chain set up below."""
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.w = 1.0  # facing +x

        self._goal_done = False
        self._goal_handle = None
        self._result_future = None
        self._current_goal_xy = (float(x), float(y))
        self._send_future = self.nav_client.send_goal_async(msg)
        self._send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        self._send_future = None
        if handle is None or not handle.accepted:
            self.get_logger().warn(
                f'NavigateToPose goal {self._current_goal_xy} rejected by Nav2'
            )
            self._goal_done = True
            self._current_goal_xy = None
            return
        self._goal_handle = handle
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, _future) -> None:
        self._goal_done = True
        self._goal_handle = None
        self._result_future = None

    def cancel_active_goal(self) -> None:
        """Best-effort cancellation. Used at episode reset so the in-flight goal
        does not fight the upcoming Gazebo soft reset."""
        if self._goal_handle is not None and self._goal_handle.accepted:
            self._goal_handle.cancel_goal_async()
        self._send_future = None
        self._result_future = None
        self._goal_handle = None
        self._goal_done = True
        self._current_goal_xy = None


def _lidar_quality(scan: LaserScan | None) -> tuple[float, float, float, float]:
    """Block A — 8-sector lidar quality (ADR-008:17-24)."""
    if scan is None or len(scan.ranges) == 0:
        return 0.0, 0.0, 0.0, 0.0

    arr = np.asarray(list(scan.ranges), dtype=np.float32)
    finite = np.isfinite(arr)
    in_range = finite & (arr > scan.range_min) & (arr < scan.range_max)
    valid_ratio = float(in_range.sum()) / float(arr.size)
    saturated = (~finite) | (arr <= scan.range_min) | (arr >= scan.range_max)
    blockage_ratio = float(saturated.sum()) / float(arr.size)

    sector_size = max(1, arr.size // LIDAR_SECTORS)
    variances: list[float] = []
    for i in range(LIDAR_SECTORS):
        chunk = arr[i * sector_size:(i + 1) * sector_size]
        chunk = chunk[np.isfinite(chunk)]
        variances.append(float(np.var(chunk)) if chunk.size > 1 else 0.0)
    return sum(variances) / len(variances), max(variances), blockage_ratio, valid_ratio


def _camera_quality(image: Image | None) -> tuple[float, float]:
    """Block B — mean luminance + log-Laplacian variance (ADR-008:26-31).

    Manual RGB→Y decoding (ITU-R BT.601) avoids a cv_bridge dependency.
    """
    if image is None or image.height == 0 or image.width == 0 or len(image.data) == 0:
        return 0.0, 0.0

    h, w = image.height, image.width
    enc = image.encoding
    raw = np.frombuffer(bytes(image.data), dtype=np.uint8)
    if enc == 'rgb8' and raw.size >= h * w * 3:
        arr = raw[:h * w * 3].reshape(h, w, 3)
        r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]
    elif enc == 'bgr8' and raw.size >= h * w * 3:
        arr = raw[:h * w * 3].reshape(h, w, 3)
        b, g, r = arr[..., 0], arr[..., 1], arr[..., 2]
    elif enc == 'mono8' and raw.size >= h * w:
        y_full = raw[:h * w].reshape(h, w).astype(np.float32)
        return _luminance_metrics(y_full)
    else:
        return 0.0, 0.0

    y = 0.299 * r.astype(np.float32) + 0.587 * g.astype(np.float32) + 0.114 * b.astype(np.float32)
    return _luminance_metrics(y)


def _luminance_metrics(y: np.ndarray) -> tuple[float, float]:
    target_w, target_h = CAM_DOWNSAMPLE_TARGET
    h, w = y.shape
    if w > target_w * 2 and h > target_h * 2:
        y = y[::max(1, h // target_h), ::max(1, w // target_w)]

    mean_lum = float(y.mean()) / 255.0
    if y.shape[0] < 3 or y.shape[1] < 3:
        return mean_lum, 0.0

    # 4-neighbour Laplacian on the inner pixels.
    center = y[1:-1, 1:-1]
    lap = (4.0 * center
           - y[:-2, 1:-1] - y[2:, 1:-1]
           - y[1:-1, :-2] - y[1:-1, 2:])
    log_lap = log(float(np.var(lap)) + 1e-6)
    return mean_lum, log_lap


def _imu_dynamics(imu: Imu | None) -> tuple[float, float, float, float, float, float]:
    """Block C — 3-axis gyro + 3-axis accel (ADR-008:33-40)."""
    if imu is None:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    return (
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
    )


def _ekf_planar_cov(odom: Odometry | None) -> tuple[float, float, float]:
    """Block D — σ_xx, σ_yy, σ_yaw from /odom.pose.covariance (ADR-008:42-50)."""
    if odom is None:
        return 0.0, 0.0, 0.0
    cov = odom.pose.covariance
    # 6x6 row-major over (x, y, z, roll, pitch, yaw). Diagonal indices: 0, 7, 35.
    return float(cov[0]), float(cov[7]), float(cov[35])


def _normalise(
    a: tuple[float, float, float, float],
    b: tuple[float, float],
    c: tuple[float, float, float, float, float, float],
    d: tuple[float, float, float],
) -> np.ndarray:
    """Per-feature normalisation per ADR-009:35-48."""
    range_sq = LIDAR_RANGE_MAX * LIDAR_RANGE_MAX
    return np.array([
        np.clip(a[0] / range_sq, 0.0, 1.0),
        np.clip(a[1] / range_sq, 0.0, 1.0),
        np.clip(a[2], 0.0, 1.0),
        np.clip(a[3], 0.0, 1.0),
        np.clip(b[0], 0.0, 1.0),
        np.clip(b[1] / LAPLACIAN_LOG_MAX, 0.0, 1.0),
        np.clip(c[0] / IMU_GYRO_MAX, -1.0, 1.0),
        np.clip(c[1] / IMU_GYRO_MAX, -1.0, 1.0),
        np.clip(c[2] / IMU_GYRO_MAX, -1.0, 1.0),
        np.clip(c[3] / IMU_ACCEL_MAX, -1.0, 1.0),
        np.clip(c[4] / IMU_ACCEL_MAX, -1.0, 1.0),
        np.clip(c[5] / IMU_ACCEL_MAX, -1.0, 1.0),
        np.clip(d[0] / COV_MAX, 0.0, 1.0),
        np.clip(d[1] / COV_MAX, 0.0, 1.0),
        np.clip(d[2] / COV_MAX, 0.0, 1.0),
    ], dtype=np.float32)


class SACEnv(gym.Env):
    """RLNavigation-v1 (Phase 1, A-1 cut)."""

    metadata = {'render_modes': []}

    def __init__(self) -> None:
        super().__init__()
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self._node = _V1Node()

        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(OBS_DIM,), dtype=np.float32,
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(ACTION_DIM,), dtype=np.float32,
        )
        self._step_count = 0

        # Block until gate services are available — env is useless without them.
        # Non-fatal so tests/dev runs without /ekf_input_gate can still exercise
        # observation building; step() will then surface set_param_ok=False.
        sp_ready = self._node.set_param_client.wait_for_service(
            timeout_sec=GATE_WAIT_TIMEOUT_S
        )
        rel_ready = self._node.release_client.wait_for_service(
            timeout_sec=GATE_WAIT_TIMEOUT_S
        )
        if not (sp_ready and rel_ready):
            self._node.get_logger().warn(
                f'/ekf_input_gate services not ready within {GATE_WAIT_TIMEOUT_S}s '
                f'(set_parameters={sp_ready}, release={rel_ready}); '
                'step() will skip cycles (set_param_ok=False).'
            )

        nav_ready = self._node.nav_client.wait_for_server(
            timeout_sec=NAV_WAIT_TIMEOUT_S
        )
        if not nav_ready:
            self._node.get_logger().warn(
                f'{NAV_ACTION_NAME} action server not ready within '
                f'{NAV_WAIT_TIMEOUT_S}s; the random-goal driver will be inert '
                'and the robot will sit still.'
            )

    def _spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.01)

    def _all_topics_received(self) -> bool:
        n = self._node
        return all([
            n.latest_scan is not None,
            n.latest_imu is not None,
            n.latest_image is not None,
            n.latest_odom is not None,
            n.latest_gt is not None,
        ])

    def _wait_for_initial_obs(self, timeout_s: float) -> bool:
        """Spin until every subscribed topic has at least one buffered message.

        Returns True on success, False on timeout. ADR-010:73 demands a fresh
        /odom before emitting the initial obs; this also covers the lidar /
        imu / image / GT channels so step 0's obs is meaningful.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if self._all_topics_received():
                return True
        return False

    def _gz_reset_world(self) -> bool:
        """Two-step soft reset (interim subprocess path):

        (1) `/world/<name>/control` with `reset: {model_only: true}` — zeroes
            velocities for all models and resets the SDF-registered ones (walls,
            obstacles) to their initial poses. We avoid `all: true` because it
            tears down sensor plugin entities, leaving IMU/PosePublisher silent
            for the rest of the process (verified 2026-05-11).
        (2) `/world/<name>/set_pose` with `name: <robot>, position: {0,0,0}`
            — explicitly teleports the dynamically-spawned robot back to spawn.
            `ros_gz_sim create` does not register the spawn pose as the
            "initial pose" that WorldControl.reset would restore, so model_only
            reset on its own leaves the robot wherever the previous episode
            ended (observed 2026-05-12).

        Once ros_gz_bridge's service bridge is wired into the launch this
        should be replaced with persistent rclpy clients to remove the
        subprocess overhead from every reset.
        """
        try:
            ctrl = subprocess.run(
                ['gz', 'service',
                 '-s', f'/world/{GZ_WORLD_NAME}/control',
                 '--reqtype', 'gz.msgs.WorldControl',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '500',
                 '--req', 'reset: {model_only: true}'],
                capture_output=True,
                text=True,
                timeout=GZ_RESET_TIMEOUT_S,
            )
            ctrl_ok = ctrl.returncode == 0 and 'true' in ctrl.stdout.lower()
            if not ctrl_ok:
                self._node.get_logger().warn(
                    f'gazebo control reset returned rc={ctrl.returncode} '
                    f'stdout={ctrl.stdout!r} stderr={ctrl.stderr!r}'
                )

            pose_req = (
                f'name: "{GZ_ROBOT_NAME}", '
                'position: {x: 0.0, y: 0.0, z: 0.0}, '
                'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
            )
            pose = subprocess.run(
                ['gz', 'service',
                 '-s', f'/world/{GZ_WORLD_NAME}/set_pose',
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '500',
                 '--req', pose_req],
                capture_output=True,
                text=True,
                timeout=GZ_RESET_TIMEOUT_S,
            )
            pose_ok = pose.returncode == 0 and 'true' in pose.stdout.lower()
            if not pose_ok:
                self._node.get_logger().warn(
                    f'gazebo set_pose for {GZ_ROBOT_NAME} returned '
                    f'rc={pose.returncode} stdout={pose.stdout!r} '
                    f'stderr={pose.stderr!r}'
                )
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            self._node.get_logger().warn(f'gazebo reset failed: {exc}')
            return False
        return ctrl_ok and pose_ok

    def _drop_buffers(self) -> None:
        n = self._node
        n.latest_scan = None
        n.latest_imu = None
        n.latest_image = None
        n.latest_odom = None
        n.latest_gt = None

    def _slam_lifecycle_transition(self, transition_id: int) -> bool:
        """Best-effort lifecycle transition. Returns True iff slam_toolbox
        accepted the transition. False on any failure (service not ready,
        timeout, transition rejected) — the caller logs and continues."""
        if not self._node.slam_state_client.service_is_ready():
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self._node.slam_state_client.call_async(req)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=SLAM_LIFECYCLE_TRANSITION_TIMEOUT_S
        )
        if not future.done():
            return False
        result = future.result()
        return result is not None and bool(result.success)

    def _slam_full_reset(self) -> bool:
        """ADR-010:62: rebuild slam_toolbox each episode by cycling its
        lifecycle. Returns True iff every transition in the sequence
        succeeded; on partial failure, slam_toolbox may be in any state and
        the caller should log but proceed (the topic-wait that follows will
        catch a stuck node by failing to receive fresh /odom)."""
        for transition_id in SLAM_LIFECYCLE_RESET_SEQUENCE:
            if not self._slam_lifecycle_transition(transition_id):
                self._node.get_logger().warn(
                    f'slam_toolbox lifecycle transition {transition_id} failed; '
                    'map may be stale this episode'
                )
                return False
        return True

    def _build_obs(self) -> np.ndarray:
        return _normalise(
            _lidar_quality(self._node.latest_scan),
            _camera_quality(self._node.latest_image),
            _imu_dynamics(self._node.latest_imu),
            _ekf_planar_cov(self._node.latest_odom),
        )

    def _compute_reward(self) -> float:
        gt = self._node.latest_gt
        odom = self._node.latest_odom
        if gt is None or odom is None:
            return 0.0
        dx = odom.pose.pose.position.x - gt.pose.position.x
        dy = odom.pose.pose.position.y - gt.pose.position.y
        return -sqrt(dx * dx + dy * dy) * REWARD_SCALE

    def _check_collision(self) -> bool:
        """ADR-010:23 — minimum lidar return below 0.15 m means imminent contact."""
        scan = self._node.latest_scan
        if scan is None or len(scan.ranges) == 0:
            return False
        arr = np.asarray(list(scan.ranges), dtype=np.float32)
        arr = arr[np.isfinite(arr) & (arr > scan.range_min)]
        if arr.size == 0:
            return False
        return float(arr.min()) < COLLISION_DISTANCE_M

    def _check_divergence(self) -> bool:
        """ADR-010:24 — EKF pose more than 2 m from ground truth."""
        gt = self._node.latest_gt
        odom = self._node.latest_odom
        if gt is None or odom is None:
            return False
        dx = odom.pose.pose.position.x - gt.pose.position.x
        dy = odom.pose.pose.position.y - gt.pose.position.y
        return sqrt(dx * dx + dy * dy) > DIVERGENCE_DISTANCE_M

    def _sample_random_goal(self) -> tuple[float, float]:
        """Uniform-random (x, y) in WORLD_BOUND_*, rejection-sampled away from
        the four obstacles and the spawn point. Returns the configured fallback
        if the rejection budget is exhausted (should not happen with the current
        arena, but keeps the env resilient if obstacles are added)."""
        for _ in range(GOAL_REJECT_BUDGET):
            x = float(self.np_random.uniform(*WORLD_BOUND_X))
            y = float(self.np_random.uniform(*WORLD_BOUND_Y))
            if x * x + y * y < MIN_GOAL_DIST_FROM_SPAWN ** 2:
                continue
            blocked = False
            for ox, oy in GOAL_FORBIDDEN_CENTERS:
                if (x - ox) ** 2 + (y - oy) ** 2 < GOAL_OBSTACLE_RADIUS_M ** 2:
                    blocked = True
                    break
            if not blocked:
                return x, y
        self._node.get_logger().warn(
            f'goal rejection sampling exhausted {GOAL_REJECT_BUDGET} attempts; '
            f'using fallback {GOAL_FALLBACK}'
        )
        return GOAL_FALLBACK

    def _send_random_goal(self) -> tuple[float, float]:
        x, y = self._sample_random_goal()
        self._node.send_goal(x, y)
        return x, y

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        self._step_count = 0

        # ADR-010:73 reset sequence:
        # (1) cancel any in-flight Nav2 goal so /cmd_vel stops fighting the
        #     upcoming Gazebo soft reset
        # (2) Gazebo model_only soft reset → robot back at spawn
        # (3) slam_toolbox lifecycle reset → fresh pose graph + map (ADR-010:62
        #     and ADR-010:43-47 require rebuilding the map each episode; without
        #     this, Nav2 plans against stale map cells and the goal-completion
        #     callback fires on every step from rejected/instant-success goals)
        # (4) drop subscriber buffers and wait for fresh sensor messages
        # (5) send the next random goal so step() inherits a moving robot
        self._node.cancel_active_goal()
        gz_ok = self._gz_reset_world()
        slam_ok = self._slam_full_reset()
        self._drop_buffers()
        topics_ok = self._wait_for_initial_obs(RESET_TOPIC_WAIT_S)
        if not topics_ok:
            self._node.get_logger().warn(
                f'reset: not all topics refreshed within {RESET_TOPIC_WAIT_S}s; '
                'returning partial obs (zeros for missing channels)'
            )
        goal_xy = self._send_random_goal()
        return self._build_obs(), {
            'gz_reset_ok': gz_ok,
            'slam_reset_ok': slam_ok,
            'topics_ready': topics_ok,
            'goal_xy': goal_xy,
        }

    def _apply_action(self, action) -> tuple[bool, float, float]:
        """Push σ to /ekf_input_gate and trigger release.

        Returns (set_param_ok, sigma_wheel, sigma_imu). On ack timeout
        the release is NOT fired (per ADR-012:23-30 the cycle is skipped).
        """
        a = np.asarray(action, dtype=np.float32).reshape(-1)
        a = np.clip(a, -1.0, 1.0)
        sigma_wheel = float(np.exp(a[0] * ACTION_LOG_SCALE))
        sigma_imu = float(np.exp(a[1] * ACTION_LOG_SCALE))

        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name='sigma_wheel',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=sigma_wheel
                ),
            ),
            Parameter(
                name='sigma_imu',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=sigma_imu
                ),
            ),
        ]
        future = self._node.set_param_client.call_async(req)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=SET_PARAM_TIMEOUT_S
        )

        if not future.done():
            self._node.get_logger().warn(
                f'set_parameters ack missed {SET_PARAM_TIMEOUT_S * 1000:.0f}ms '
                'deadline; cycle skipped'
            )
            return False, sigma_wheel, sigma_imu

        result = future.result()
        all_ok = result is not None and all(r.successful for r in result.results)
        if not all_ok:
            self._node.get_logger().warn(
                f'set_parameters returned non-success for σ=({sigma_wheel:.3f},'
                f'{sigma_imu:.3f}); cycle skipped'
            )
            return False, sigma_wheel, sigma_imu

        # Fire release; we do not await it. The gate's response is diagnostic.
        self._node.release_client.call_async(Trigger.Request())
        return True, sigma_wheel, sigma_imu

    def step(self, action):
        # ADR-012:23-30: a single set_parameters timeout is invisible to SAC —
        # retry the same action until it lands or we exhaust the budget. The
        # exhausted case is the ADR-012:50 "wrapper unreachable" infrastructure
        # failure and surfaces as truncated=True.
        set_param_ok = False
        sigma_wheel = sigma_imu = float('nan')
        for _ in range(SET_PARAM_MAX_RETRIES):
            set_param_ok, sigma_wheel, sigma_imu = self._apply_action(action)
            if set_param_ok:
                break

        odom_before = self._node.latest_odom
        self._spin_for(STEP_PERIOD_S)
        # ADR-012:51: /odom not advancing during the cycle is an infra failure.
        # Reference identity works because each rclpy callback delivers a fresh
        # message object.
        odom_silent = (
            odom_before is not None
            and self._node.latest_odom is odom_before
        )

        # ADR-010:72 — re-sample the moment Nav2 reports the current goal
        # finished (succeeded, aborted, or rejected). Keeps the robot moving
        # for the whole episode rather than parking after the first arrival.
        new_goal_xy = None
        if self._node._goal_done:
            new_goal_xy = self._send_random_goal()

        obs = self._build_obs()
        reward = self._compute_reward() if set_param_ok else 0.0
        self._step_count += 1

        # ADR-010:21-26 termination/truncation matrix
        collision = self._check_collision()
        divergence = self._check_divergence()
        terminated = collision or divergence

        timeout = self._step_count >= MAX_EPISODE_STEPS
        infra_failure = (not set_param_ok) or odom_silent
        truncated = timeout or infra_failure

        terminated_reason = (
            'collision' if collision else 'divergence' if divergence else None
        )
        truncated_reason = (
            'set_param_unreachable' if not set_param_ok
            else 'odom_silent' if odom_silent
            else 'timeout' if timeout
            else None
        )

        info = {
            'step': self._step_count,
            'has_scan': self._node.latest_scan is not None,
            'has_imu': self._node.latest_imu is not None,
            'has_image': self._node.latest_image is not None,
            'has_odom': self._node.latest_odom is not None,
            'has_gt': self._node.latest_gt is not None,
            'set_param_ok': set_param_ok,
            'sigma_wheel': sigma_wheel,
            'sigma_imu': sigma_imu,
            'terminated_reason': terminated_reason,
            'truncated_reason': truncated_reason,
            'goal_xy': self._node._current_goal_xy,
            'goal_resampled': new_goal_xy is not None,
        }
        return obs, reward, terminated, truncated, info

    def close(self) -> None:
        # Cancel the in-flight Nav2 goal so the robot stops moving when the
        # trainer / smoke test exits. Wrapped in try/except because rclpy may
        # already be shutting down (e.g. KeyboardInterrupt during learn()).
        try:
            self._node.cancel_active_goal()
        except Exception:  # noqa: BLE001
            pass
        self._node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
