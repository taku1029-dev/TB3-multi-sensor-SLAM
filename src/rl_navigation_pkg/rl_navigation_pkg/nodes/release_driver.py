"""Steady 10 Hz driver for /ekf_input_gate/release.

This node is a stand-in for the SAC env's per-cycle protocol per ADR-004:15-41.
Until RLNavigation-v1 takes over, something has to clock the EKF input gate
deterministically — a shell `while true; ros2 service call ...` loop is too
bursty (each ros2 service call forks a python process), which leaves /odom
and odom->base_footprint TF intermittent. That breaks downstream Message Filters
(costmap_2d, slam_toolbox) with "timestamp earlier than all TF cache data" drops.

Replace with the SAC env's release call once RLNavigation-v1 is wired.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

DEFAULT_PERIOD_S = 0.1   # 10 Hz, matches ADR-004:13


class ReleaseDriver(Node):
    def __init__(self) -> None:
        super().__init__('release_driver')
        self.declare_parameter('period_s', DEFAULT_PERIOD_S)
        self.declare_parameter('service_name', '/ekf_input_gate/release')

        period = float(self.get_parameter('period_s').value)
        service = str(self.get_parameter('service_name').value)

        self._client = self.create_client(Trigger, service)
        self.get_logger().info(f'Waiting for {service} ...')
        self._client.wait_for_service()
        self.get_logger().info(f'release_driver clocking {service} every {period:.3f}s')

        self.create_timer(period, self._tick)

    def _tick(self) -> None:
        # Fire-and-forget; we don't await the response. The gate's response is
        # purely diagnostic (`wheel,imu` etc.) and isn't consumed here.
        self._client.call_async(Trigger.Request())


def main() -> None:
    rclpy.init()
    node = ReleaseDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
