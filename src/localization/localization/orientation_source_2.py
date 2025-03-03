import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Quaternion


class MagHeadingNode(Node):
    def __init__(self):
        super().__init__("mag_heading_node")

        # ✅ Odometry 데이터를 저장할 변수
        self.odom_data = None

        # Magnetometer 및 Odometry 구독
        self.mag_sub = self.create_subscription(
            MagneticField, "imu/mag", self.mag_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "odometry/wheel", self.odometry_callback, 10
        )

        # 새로운 Odometry 발행
        self.pub = self.create_publisher(Odometry, "odometry/mag", 10)

        # Roll, Pitch 값 저장용
        self.roll = 0.0
        self.pitch = 0.0

    def imu_callback(self, msg):
        """IMU 데이터를 받아서 Roll, Pitch를 업데이트"""
        q = msg.orientation
        # 쿼터니언 -> 오일러 변환
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            self.pitch = math.copysign(math.pi / 2, sinp)  # 클램핑
        else:
            self.pitch = math.asin(sinp)

    def odometry_callback(self, msg):
        """Odometry 데이터를 받아서 저장 (위치 정보 유지)"""
        self.odom_data = msg  # 최신 Odometry 데이터 저장

    def mag_callback(self, msg):
        """Magnetometer 데이터를 받아 Heading을 계산하고, Odometry를 업데이트"""
        if self.odom_data is None:
            return  # Odometry 데이터가 없으면 업데이트하지 않음

        # Magnetometer X, Y, Z 값
        mx, my, mz = msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z

        # Step 1: Roll 보정
        mx_r = mx
        my_r = my * math.cos(self.roll) - mz * math.sin(self.roll)
        mz_r = my * math.sin(self.roll) + mz * math.cos(self.roll)

        # Step 2: Pitch 보정
        mx_p = mx_r * math.cos(self.pitch) + mz_r * math.sin(self.pitch)
        my_p = my_r
        mz_p = -mx_r * math.sin(self.pitch) + mz_r * math.cos(self.pitch)

        # Step 3: 보정된 X, Y를 이용하여 Heading 계산
        heading = math.atan2(my_p, mx_p)

        # ✅ -π ~ π 범위로 변환
        heading = (heading + math.pi) % (2 * math.pi) - math.pi

        # ✅ Heading 값을 Quaternion으로 변환
        q = self.yaw_to_quaternion(heading)

        # ✅ 새로운 Odometry 메시지 생성
        new_odom = Odometry()
        new_odom.header = self.odom_data.header
        new_odom.child_frame_id = self.odom_data.child_frame_id
        new_odom.pose.pose.position = (
            self.odom_data.pose.pose.position
        )  # 기존 위치 유지
        new_odom.pose.pose.orientation = q  # ✅ 새로운 heading 반영
        new_odom.twist = self.odom_data.twist  # 속도 정보 유지

        # ✅ 새로운 Odometry 메시지 발행
        self.pub.publish(new_odom)

        # ✅ Heading 값 로깅 (디버깅용)
        heading_deg = heading * 180.0 / math.pi
        self.get_logger().info(
            f"Published Updated Odometry with Heading: {heading_deg:.2f} degrees"
        )

    def yaw_to_quaternion(self, yaw):
        """Yaw 값을 Quaternion으로 변환"""
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = MagHeadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
