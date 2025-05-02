import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from scipy.fftpack import fft, fftfreq


class VibrationAnalyzer(Node):
    def __init__(self):
        super().__init__("vibration_analyzer")
        self.sub = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)

        self.acc_buffer = deque(maxlen=100)  # 1초 분량
        self.gyro_buffer = deque(maxlen=100)

        self.get_logger().info("Vibration analyzer started.")

    def imu_callback(self, msg):
        acc = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

        self.acc_buffer.append(acc)
        self.gyro_buffer.append(gyro)

        if len(self.acc_buffer) >= self.acc_buffer.maxlen:
            self.analyze_vibration()

    def analyze_vibration(self):
        acc_array = np.array(self.acc_buffer)
        gyro_array = np.array(self.gyro_buffer)

        sample_rate = 100  # Hz
        N = acc_array.shape[0]
        freqs = fftfreq(N, 1 / sample_rate)

        fig, axs = plt.subplots(3, 2, figsize=(10, 8))
        fig.suptitle("Vibration Histogram (Accel & Gyro)", fontsize=16)

        axis_labels = ["x", "y", "z"]

        warning_triggered = False
        warning_logs = []

        for i in range(3):
            acc_data = acc_array[:, i]
            gyro_data = gyro_array[:, i]

            # RMS 계산
            # 평균 제거 후 RMS (진동 세기만 계산)
            acc_rms = np.sqrt(np.mean((acc_data - np.mean(acc_data)) ** 2))
            gyro_rms = np.sqrt(np.mean((gyro_data - np.mean(gyro_data)) ** 2))

            acc_status = None
            gyro_status = None

            if acc_rms > 0.5:
                acc_status = (
                    f"[Accel {axis_labels[i]}] RMS: {acc_rms:.4f} m/s² → ❗ 강한 진동"
                )
            elif acc_rms > 0.2:
                acc_status = (
                    f"[Accel {axis_labels[i]}] RMS: {acc_rms:.4f} m/s² → ⚠️ 중간 진동"
                )

            if gyro_rms > 0.05:
                gyro_status = f"[Gyro  {axis_labels[i]}] RMS: {gyro_rms:.4f} rad/s → ❗ 회전 진동 심함"
            elif gyro_rms > 0.01:
                gyro_status = f"[Gyro  {axis_labels[i]}] RMS: {gyro_rms:.4f} rad/s → ⚠️ 약간의 회전 진동"

            if acc_status:
                warning_logs.append(acc_status)
                warning_triggered = True
            if gyro_status:
                warning_logs.append(gyro_status)
                warning_triggered = True

            # 히스토그램 시각화 (옵션)
            axs[i][0].hist(acc_data, bins=20, color="skyblue", edgecolor="black")
            axs[i][0].set_title(f"Accel {axis_labels[i]}")
            axs[i][0].set_xlabel("m/s²")
            axs[i][0].set_ylabel("Count")

            axs[i][1].hist(gyro_data, bins=20, color="salmon", edgecolor="black")
            axs[i][1].set_title(f"Gyro {axis_labels[i]}")
            axs[i][1].set_xlabel("rad/s")
            axs[i][1].set_ylabel("Count")

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        # plt.show()

        # 경고가 발생했을 때만 로그 출력
        if warning_triggered:
            self.get_logger().info("--------------- ⚠️ 진동 경고 ---------------")
            for log in warning_logs:
                self.get_logger().warn(log)
            self.get_logger().info("--------------- ⚠️ 진동 경고 끝 -------------")


def main():
    rclpy.init()
    node = VibrationAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
