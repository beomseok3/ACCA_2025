import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import math as m
import numpy as np

from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from DB import DB


class DBWRITE(Node):
    def __init__(self):
        super().__init__("dbwrite")
        # 고주파 odometry 콜백: "odometry/kf"
        self.sub_local = self.create_subscription(
            Odometry,
            "odometry/kf",
            self.callback_local,
            qos_profile_system_default,
        )

        # MarkerArray 발행용 퍼블리셔 (시각화)
        self.pub_marker_array = self.create_publisher(
            MarkerArray, "domain", qos_profile_system_default
        )

        # DB 쓰기를 위한 타이머 (예: 1초 주기)
        self.db_timer = self.create_timer(1.0, self.write_db_timer_callback)
        # 시각화를 위한 타이머 (예: 1초 주기)
        self.marker_timer = self.create_timer(10, self.domain_for_visu)

        # 경로 데이터 버퍼 (고주파 콜백에서는 append만 수행)
        self.path_x = []
        self.path_y = []
        self.init_x = 0.0
        self.init_y = 0.0
        self.path_yaw = []
        # DB 클래스 초기화
        self.db = DB("loop_wheel.db")
        # 예시: 초기 노드 간 연결(DB에 쓰기)
        self.db.write_db_Node(
            [(f"A1", f"A2", f"A1A2")],
            mission="driving",
        )

        # (지도 원점 설정: 필요에 따라 주석 해제)
        # school
        # self.map_lat_ = 37.4966945
        # self.map_lon_ = 126.9575076
        # kcity-bs
        # self.map_lat_ = 37.2388925
        # self.map_lon_ = 126.77293309999999
        # kcity-ys
        # self.map_lat_ = 37.2392369
        # self.map_lon_ = 126.77316379999999

        self.get_logger().info("DBWRITE node initialized.")

    def callback_local(self, msg):
        # 빠른 연산: 위치와 yaw만 계산하고 리스트에 추가
        if not self.path_x:
            self.init_x = msg.pose.pose.position.x
            self.init_y = msg.pose.pose.position.y

        x = msg.pose.pose.position.x - self.init_x
        y = msg.pose.pose.position.y - self.init_y
        # quat = msg.pose.pose.orientation
        # _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        # 간단하게 중복 여부는 생략하거나 나중에 별도 처리
        self.path_x.append(x)
        self.path_y.append(y)
        self.path_yaw.append(0.0)

    def write_db_timer_callback(self):
        # DB 쓰기와 관련된 연산은 타이머 콜백에서 수행
        if not self.path_x:
            return
        # path 데이터를 zip으로 생성 (여기서만 생성)
        path = list(zip(self.path_x, self.path_y, self.path_yaw))
        # DB에 저장 (여기서는 예시로 speed=5.0 사용)
        self.db.write_db_Path(path, speed=5.0)
        # 선택: 버퍼를 비워 다음 주기까지 중복 쓰기 방지
        # 만약 누적 기록을 원한다면 주석 처리할 수 있음.
        # self.path_x.clear()
        # self.path_y.clear()
        # self.path_yaw.clear()
        self.get_logger().info("DB updated with path data.")

    def domain_for_visu(self):
        # MarkerArray 생성 및 발행 (시각화용)
        marker_array = MarkerArray()
        for i, (x, y, yaw) in enumerate(zip(self.path_x, self.path_y, self.path_yaw)):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "global_path"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, yaw)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)
        self.pub_marker_array.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DBWRITE()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
