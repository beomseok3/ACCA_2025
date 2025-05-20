import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Temperature

import threading
import queue


class ImuBiasMeasure(Node):

    def __init__(self):
        super().__init__("imu_bias_measure")
        self.subscription = self.create_subscription(
            Imu, "imu/data", self.listener_callback, qos_profile=qos_profile_sensor_data
        )

        # I/O 처리를 위한 큐와 데몬 스레드 생성
        self.io_queue = queue.Queue()
        self.io_thread = threading.Thread(target=self.io_worker, daemon=True)
        self.io_thread.start()

        self.current_time = None
        self.last_time = None
        self.last_angular_velocity = None
        self.total_time = 0.0
        self.bias = 0.0

    def listener_callback(self, msg):
        """
        각속도를 시간에 따라 적분하여 편향을 측정.
        (Tustin 적분법 적용)
        """
        self.get_logger().info(f"imu_data: {msg.linear_acceleration.x}")
        self.current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = self.current_time
            return

        if self.last_angular_velocity is None:
            self.last_angular_velocity = msg.linear_acceleration.x
            return

        dt = self.current_time - self.last_time

        # Tustin (trapezoidal) integration
        self.bias += 0.5 * (msg.linear_acceleration.x + self.last_angular_velocity) * dt
        self.total_time += dt

        self.last_angular_velocity = msg.linear_acceleration.x
        self.last_time = self.current_time

        if self.total_time != 0:
            avg_bias = self.bias / self.total_time
        else:
            avg_bias = 0.0

        self.get_logger().info(f"The average bias is: {avg_bias}")
        self.make_txt(self.total_time, avg_bias)

    def make_txt(self, time_val, value):
        """
        데이터를 큐에 넣어 I/O 스레드가 처리하도록 함.
        """
        data = "{},{}\n".format(time_val, value)
        self.io_queue.put(data)

    def io_worker(self):
        """
        데몬 스레드에서 실행되며, 큐에 저장된 데이터를 파일에 기록.
        """
        file_path = "/home/ps/imu_data/dolge_630_acc_bias.txt"
        with open(file_path, "a") as f:
            while True:
                try:
                    data = self.io_queue.get(timeout=1.0)
                    f.write(data)
                    f.flush()
                except queue.Empty:
                    continue


def main(args=None):
    rclpy.init(args=args)
    node = ImuBiasMeasure()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
