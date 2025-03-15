# pip install pyproj
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import pyproj
import numpy as np


class GPSToUTMKNode(Node):
    def __init__(self):
        super().__init__("gps_to_utmk")
        # GPS 데이터를 구독 (WGS84 좌표계: EPSG:4326)
        self.create_subscription(
            NavSatFix, "ublox_gps_node/fix", self.gps_callback, qos_profile_sensor_data
        )
        # 변환된 좌표를 Odometry 메시지로 publish (UTM-K 좌표계)
        self.publisher = self.create_publisher(
            Odometry, "odometry/gps", qos_profile_sensor_data
        )

        # WGS84 (EPSG:4326) -> UTM-K (EPSG:5179) 변환기 생성, 항상 (lon, lat) 순서를 사용
        self.transformer = pyproj.Transformer.from_crs(
            "EPSG:4326", "EPSG:5179", always_xy=True
        )

    def gps_callback(self, msg):
        # NavSatFix 메시지에서 위도와 경도 추출
        lat = msg.latitude
        lon = msg.longitude

        # pyproj를 사용하여 WGS84 (lon, lat) -> UTM-K (x, y)로 변환
        utmk_x, utmk_y = self.transformer.transform(lon, lat)

        # Odometry 메시지 생성
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        # 프레임 id를 UTM-K 좌표계로 지정 (원하는 이름으로 변경 가능)
        odom_msg.header.frame_id = "odom"

        # 변환된 좌표를 position에 대입
        odom_msg.pose.pose.position.x = utmk_x
        odom_msg.pose.pose.position.y = utmk_y
        # 높이는 별도 센서 데이터가 없다면 0.0으로 설정
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.orientation.x = 0.0
        odom_msg.pose.orientation.y = 0.0
        odom_msg.pose.orientation.z = 0.0
        odom_msg.pose.orientation.w = 1.0

        odom_msg.pose.covariance = np.identity(6)

        # x (m) 공분산
        # odom_msg.pose.covariance[0] = msg.
        # # y (m) 공분산
        # odom_msg.pose.covariance[6] =

        # (필요하다면 orientation 및 covariance도 설정 가능)

        self.publisher.publish(odom_msg)
        self.get_logger().info(
            f"GPS (lat: {lat:.6f}, lon: {lon:.6f}) -> UTM-K (x: {utmk_x:.2f}, y: {utmk_y:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GPSToUTMKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
