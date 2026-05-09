"""EKF input gate wrapper for the SAC-tuned-EKF SLAM project.

Sits between raw EKF input streams (Phase 1: /odom_wheel, /imu) and
ekf_filter_node. Buffers each stream latest-wins, applies the agent's
per-stream σ multiplier to the message covariance diagonal (with off-diagonals
zeroed per ADR-009:25), and republishes onto private topics
(/ekf_in/odom_wheel, /ekf_in/imu) only when the agent calls
/ekf_input_gate/release.

References:
  - ADR-011: tick scheduling and the wrapper's role.
  - ADR-014: σ application path — this node owns it; the agent's
    set_parameters target is /ekf_input_gate, not /ekf_filter_node.
  - ADR-009: σ semantics (per-stream scalar multiplier; off-diagonals zero).
  - ADR-006: Phase 1 streams are (wheel, imu).
"""

from __future__ import annotations

from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


class EkfInputGate(Node):
    def __init__(self) -> None:
        super().__init__('ekf_input_gate')

        self.declare_parameter('nominal_diagonal_wheel', [0.05, 0.05])
        self.declare_parameter('nominal_diagonal_imu', [0.02])
        self.declare_parameter('raw_topic_wheel', '/odom_wheel')
        self.declare_parameter('raw_topic_imu', '/imu')
        self.declare_parameter('private_topic_wheel', '/ekf_in/odom_wheel')
        self.declare_parameter('private_topic_imu', '/ekf_in/imu')

        # σ multipliers — agent updates these via set_parameters each cycle (ADR-014).
        self.declare_parameter('sigma_wheel', 1.0)
        self.declare_parameter('sigma_imu', 1.0)

        self._nominal_wheel = list(self.get_parameter('nominal_diagonal_wheel').value)
        self._nominal_imu = list(self.get_parameter('nominal_diagonal_imu').value)

        self._latest_wheel: Odometry | None = None
        self._latest_imu: Imu | None = None

        self.create_subscription(
            Odometry,
            self.get_parameter('raw_topic_wheel').value,
            self._on_wheel,
            10,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('raw_topic_imu').value,
            self._on_imu,
            10,
        )

        self._pub_wheel = self.create_publisher(
            Odometry,
            self.get_parameter('private_topic_wheel').value,
            10,
        )
        self._pub_imu = self.create_publisher(
            Imu,
            self.get_parameter('private_topic_imu').value,
            10,
        )

        self.create_service(Trigger, '~/release', self._on_release)

        self.get_logger().info('ekf_input_gate ready (Phase 1: wheel + imu)')

    def _on_wheel(self, msg: Odometry) -> None:
        self._latest_wheel = msg

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg

    def _on_release(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        sigma_wheel = self.get_parameter('sigma_wheel').value
        sigma_imu = self.get_parameter('sigma_imu').value

        released = []
        if self._latest_wheel is not None:
            self._pub_wheel.publish(self._scale_wheel(self._latest_wheel, sigma_wheel))
            released.append('wheel')
        else:
            self.get_logger().warn(
                'release: no /odom_wheel message buffered yet — skipping'
            )

        if self._latest_imu is not None:
            self._pub_imu.publish(self._scale_imu(self._latest_imu, sigma_imu))
            released.append('imu')
        else:
            self.get_logger().warn('release: no /imu message buffered yet — skipping')

        response.success = len(released) > 0
        response.message = ','.join(released) if released else 'no streams ready'
        return response

    def _scale_wheel(self, msg: Odometry, sigma: float) -> Odometry:
        # twist.covariance is a row-major 6x6 over (vx, vy, vz, vroll, vpitch, vyaw).
        # ekf_phase1.yaml's odom0_config enables vx (idx 0) and vyaw (idx 5);
        # diagonal indices in the flat 36-list are 0 and 35.
        out = deepcopy(msg)
        cov = [0.0] * 36
        cov[0] = self._nominal_wheel[0] * sigma   # vx
        cov[35] = self._nominal_wheel[1] * sigma  # vyaw
        out.twist.covariance = cov
        return out

    def _scale_imu(self, msg: Imu, sigma: float) -> Imu:
        # angular_velocity_covariance is a row-major 3x3 over (wx, wy, wz).
        # imu0_config enables yaw rate -> wz, flat-index 8.
        out = deepcopy(msg)
        cov = [0.0] * 9
        cov[8] = self._nominal_imu[0] * sigma  # wz
        out.angular_velocity_covariance = cov
        return out


def main() -> None:
    rclpy.init()
    node = EkfInputGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
