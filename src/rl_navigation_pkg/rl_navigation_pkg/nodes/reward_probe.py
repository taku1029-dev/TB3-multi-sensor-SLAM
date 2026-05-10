"""Standalone probe for the SAC env's reward subtrahend pipeline.

ADR-004:39-40 specifies `reward = -|EKF_pose - Gazebo_ground_truth| * 10.0`.
This node lets us validate the pose-difference path (frames, units, topic
liveness, quaternion extraction) BEFORE wiring it into RLNavigation-v1's
reward, so a frame-mismatch bug doesn't masquerade as a learning failure.

Subscribes:
  /ground_truth_pose  geometry_msgs/PoseStamped   (Gazebo world frame)
  /odom               nav_msgs/Odometry           (EKF, odom frame)

Frame note: the world (Gazebo) and odom (EKF) frames are not the same
frame in the strict sense, but the robot spawns at (0,0,0) so they
coincide at episode start. Their divergence over time IS the reward
signal — that's the point. No TF lookup needed.
"""

from __future__ import annotations

from math import atan2, sqrt

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    # Planar (Z-axis) yaw. Equivalent to 2 * atan2(qz, qw) when qx≈qy≈0,
    # but the full form is robust to small roll/pitch in ground-truth.
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return atan2(siny_cosp, cosy_cosp)


def _wrap_pi(angle: float) -> float:
    while angle > 3.141592653589793:
        angle -= 2.0 * 3.141592653589793
    while angle < -3.141592653589793:
        angle += 2.0 * 3.141592653589793
    return angle


class RewardProbe(Node):
    def __init__(self) -> None:
        super().__init__('reward_probe')

        self.declare_parameter('gt_topic', '/ground_truth_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('log_period_s', 1.0)
        self.declare_parameter('reward_scale', 10.0)

        gt_topic = str(self.get_parameter('gt_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        period = float(self.get_parameter('log_period_s').value)
        self._reward_scale = float(self.get_parameter('reward_scale').value)

        self._latest_gt: PoseStamped | None = None
        self._latest_odom: Odometry | None = None

        self.create_subscription(PoseStamped, gt_topic, self._on_gt, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'reward_probe: comparing {gt_topic} vs {odom_topic} '
            f'every {period:.2f}s (reward_scale={self._reward_scale})'
        )

    def _on_gt(self, msg: PoseStamped) -> None:
        self._latest_gt = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _tick(self) -> None:
        if self._latest_gt is None and self._latest_odom is None:
            self.get_logger().warn('no /ground_truth_pose AND no /odom yet')
            return
        if self._latest_gt is None:
            self.get_logger().warn('no /ground_truth_pose yet')
            return
        if self._latest_odom is None:
            self.get_logger().warn('no /odom yet')
            return

        gt = self._latest_gt
        odom = self._latest_odom

        gt_x = gt.pose.position.x
        gt_y = gt.pose.position.y
        gt_yaw = _yaw_from_quat(
            gt.pose.orientation.x,
            gt.pose.orientation.y,
            gt.pose.orientation.z,
            gt.pose.orientation.w,
        )

        ekf_x = odom.pose.pose.position.x
        ekf_y = odom.pose.pose.position.y
        ekf_yaw = _yaw_from_quat(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )

        dx = ekf_x - gt_x
        dy = ekf_y - gt_y
        l2 = sqrt(dx * dx + dy * dy)
        yaw_err = _wrap_pi(ekf_yaw - gt_yaw)
        reward = -l2 * self._reward_scale

        self.get_logger().info(
            f'GT[{gt.header.frame_id}] ({gt_x:+.3f}, {gt_y:+.3f}, '
            f'yaw={gt_yaw:+.3f})  '
            f'EKF[{odom.header.frame_id}] ({ekf_x:+.3f}, {ekf_y:+.3f}, '
            f'yaw={ekf_yaw:+.3f})  '
            f'L2={l2:.3f}m  yaw_err={yaw_err:+.3f}rad  reward={reward:+.3f}'
        )


def main() -> None:
    rclpy.init()
    node = RewardProbe()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
