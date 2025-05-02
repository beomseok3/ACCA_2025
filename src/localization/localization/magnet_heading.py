import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Quaternion
from tf_transformations import *


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
            Odometry, "odometry/navsat", self.odometry_callback, 10
        )

        # self.odom_sub = self.create_subscription(
        #     Odometry, "localization/kinematic_state", self.odometry_callback, 10
        # )
        # 새로운 Odometry 발행
        self.pub = self.create_publisher(Odometry, "odometry/mag", 10)

        # Roll, Pitch 값 저장용
        self.roll = 0.0
        self.pitch = 0.0
        
        self.first = True
        self.initial_odom_yaw = None
        self.initial_mag_yaw = None


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
        self.odom_data = msg

        # 처음 들어온 odom의 yaw 저장
        if self.first and self.initial_mag_yaw is not None:
            self.initial_odom_yaw = self.get_yaw_from_quaternion(
                self.odom_data.pose.pose.orientation
            )
            self.first = False

    def mag_callback(self, msg):
        if self.odom_data is None:
            return

        mx, my, mz = msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z

        # 보정된 heading 계산 (roll, pitch 적용)
        mx_r = mx
        my_r = my * math.cos(self.roll) - mz * math.sin(self.roll)
        mz_r = my * math.sin(self.roll) + mz * math.cos(self.roll)

        mx_p = mx_r * math.cos(self.pitch) + mz_r * math.sin(self.pitch)
        my_p = my_r

        heading = -math.atan2(my_p, mx_p)
        heading = (heading + math.pi) % (2 * math.pi) - math.pi

        # 초기 mag heading 저장
        if self.initial_mag_yaw is None:
            self.initial_mag_yaw = heading
            return  # odom yaw를 기다림

        if self.first:
            return  # odom yaw를 기다림

        # delta_heading 계산
        delta_mag = heading - self.initial_mag_yaw

        # 최종 yaw = odom의 초기 yaw + mag 변화량
        final_yaw = self.initial_odom_yaw + delta_mag

        # -π ~ π 범위로 조정
        final_yaw = (final_yaw + math.pi) % (2 * math.pi) - math.pi

        q = self.yaw_to_quaternion(final_yaw)

        # 새로운 odom 발행
        new_odom = Odometry()
        new_odom.header = self.odom_data.header
        new_odom.child_frame_id = self.odom_data.child_frame_id
        new_odom.pose.pose.position = self.odom_data.pose.pose.position
        new_odom.pose.pose.orientation = q
        new_odom.twist = self.odom_data.twist

        self.pub.publish(new_odom)
        self.get_logger().info(f"Heading: {math.degrees(final_yaw):.2f} deg")


    def yaw_to_quaternion(self, yaw):
        """Yaw 값을 Quaternion으로 변환"""
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q

    def get_yaw_from_quaternion(self, q):
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def main(args=None):
    rclpy.init(args=args)
    node = MagHeadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
