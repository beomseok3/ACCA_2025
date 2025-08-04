#!/usr/bin/env python3
"""
TimeStampRepubNode
==================
GPS(`/ublox_gps_node/fix`)·IMU(`/imu/rotated`)·Twist(`/erp42/twist`)를 받아
**현재 ROS Clock** 으로 `header.stamp` 를 덮어쓰고 동일한 메시지를
`/synced/{gps, imu, twist}` 로 그대로 재퍼블리시합니다.

✅ **ApproximateTimeSynchronizer 없이** 간단히 시간만 맞추고 내보내므로
   센서 간 큰 오프셋이 있어도 EKF 입력이 단일 시간선이 됩니다.

파라미터
---------
- `remap_twist` (bool, default False)
  * True  → Twist 도 `now()` 스탬프로 재퍼블리시
  * False → Twist 는 패스스루 (이미 현재 시각이라 가정)
"""

from __future__ import annotations

import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped


class TimeStampRepubNode(Node):
    """각 센서 메시지의 스탬프를 현재 시간으로 덮어써서 재퍼블리시."""

    def __init__(self) -> None:
        super().__init__("timestamp_repub_node")

        # ---- Parameter ----
        self.declare_parameter("remap_twist", False)
        self.remap_twist: bool = bool(self.get_parameter("remap_twist").value)

        # ---- Subscribers ----
        self.create_subscription(
            NavSatFix,
            "/ublox_gps_node/fix",
            self.gps_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu, "/imu/rotated", self.imu_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            "/erp42/twist",
            self.twist_cb,
            qos_profile_sensor_data,
        )

        # ---- Publishers ----
        self.pub_gps = self.create_publisher(NavSatFix, "/synced/gps", 10)
        self.pub_imu = self.create_publisher(Imu, "/synced/imu", 10)
        self.pub_twist = self.create_publisher(
            TwistWithCovarianceStamped, "/synced/twist", 10
        )

        self.get_logger().info(
            f"TimeStampRepubNode started (remap_twist={self.remap_twist})"
        )

    # ------------------------------------------------------------------
    def _now_stamp(self):
        return self.get_clock().now().to_msg()

    def gps_cb(self, msg: NavSatFix) -> None:
        out = copy.copy(msg)
        out.header.stamp = self._now_stamp()
        self.pub_gps.publish(out)

    def imu_cb(self, msg: Imu) -> None:
        out = copy.copy(msg)
        out.header.stamp = self._now_stamp()
        self.pub_imu.publish(out)

    def twist_cb(self, msg: TwistWithCovarianceStamped) -> None:
        if self.remap_twist:
            out = copy.copy(msg)
            out.header.stamp = self._now_stamp()
            self.pub_twist.publish(out)
        else:
            # 패스스루 (이미 최신 스탬프라 가정)
            self.pub_twist.publish(msg)


def main() -> None:
    rclpy.init()
    node = TimeStampRepubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
