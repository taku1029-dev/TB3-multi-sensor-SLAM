"""Minimal Gymnasium env that drives the simulated TB3 over ROS 2 topics.

This is the MVP loop: lidar/imu/odom_wheel in, cmd_vel out, placeholder reward.
Reward shaping and the DQN agent are intentionally out of scope here.
See ADR-001 (topic contract) and ADR-002 (action space).
"""

from __future__ import annotations

import time

import gymnasium as gym
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from gymnasium import spaces
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

LIDAR_SECTORS = 24
LIDAR_RANGE_MAX = 8.0
STEP_TIMEOUT_S = 1.0
MAX_EPISODE_STEPS = 500

ACTION_TWISTS = {
    0: (0.15, 0.0),    # forward
    1: (0.0, 0.5),     # turn left
    2: (0.0, -0.5),    # turn right
    3: (0.0, 0.0),     # stop
}


class _EnvNode(Node):
    def __init__(self) -> None:
        super().__init__('rl_navigation_env')
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.latest_scan: LaserScan | None = None
        self.latest_imu: Imu | None = None
        self.latest_odom: Odometry | None = None

        self.create_subscription(LaserScan, '/lidar', self._on_scan, sensor_qos)
        self.create_subscription(Imu, '/imu', self._on_imu, sensor_qos)
        self.create_subscription(Odometry, '/odom_wheel', self._on_odom, sensor_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _on_imu(self, msg: Imu) -> None:
        self.latest_imu = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def publish_twist(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)


def _downsample_ranges(ranges: list[float], num_sectors: int, max_range: float) -> np.ndarray:
    if not ranges:
        return np.full(num_sectors, max_range, dtype=np.float32)
    arr = np.asarray(ranges, dtype=np.float32)
    arr = np.where(np.isfinite(arr), arr, max_range)
    arr = np.clip(arr, 0.0, max_range)
    sector_size = max(1, len(arr) // num_sectors)
    sectors = np.array(
        [arr[i * sector_size:(i + 1) * sector_size].min() for i in range(num_sectors)],
        dtype=np.float32,
    )
    return sectors


class MyEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self) -> None:
        super().__init__()
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self._node = _EnvNode()

        self.observation_space = spaces.Box(
            low=0.0, high=LIDAR_RANGE_MAX,
            shape=(LIDAR_SECTORS,), dtype=np.float32,
        )
        self.action_space = spaces.Discrete(len(ACTION_TWISTS))
        self._step_count = 0

    def _spin_until_scan(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        seen_before = self._node.latest_scan
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if self._node.latest_scan is not None and self._node.latest_scan is not seen_before:
                return True
        return self._node.latest_scan is not None

    def _build_obs(self) -> np.ndarray:
        scan = self._node.latest_scan
        if scan is None:
            return np.full(LIDAR_SECTORS, LIDAR_RANGE_MAX, dtype=np.float32)
        return _downsample_ranges(list(scan.ranges), LIDAR_SECTORS, LIDAR_RANGE_MAX)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        self._node.publish_twist(0.0, 0.0)
        self._node.latest_scan = None
        self._spin_until_scan(STEP_TIMEOUT_S)
        self._step_count = 0
        return self._build_obs(), {}

    def step(self, action: int):
        linear, angular = ACTION_TWISTS[int(action)]
        self._node.publish_twist(linear, angular)
        got_scan = self._spin_until_scan(STEP_TIMEOUT_S)
        obs = self._build_obs()
        self._step_count += 1
        terminated = False
        truncated = self._step_count >= MAX_EPISODE_STEPS
        reward = 0.0  # placeholder; reward shaping deferred
        info = {'scan_received': got_scan, 'step': self._step_count}
        return obs, reward, terminated, truncated, info

    def close(self) -> None:
        try:
            self._node.publish_twist(0.0, 0.0)
        except Exception:
            pass
        self._node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
