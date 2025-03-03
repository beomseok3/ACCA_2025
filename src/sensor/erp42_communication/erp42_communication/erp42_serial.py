import sys
import rclpy
import serial
import serial.tools.list_ports
import threading
import math as m
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from erp42_msgs.msg import SerialFeedBack, ControlMessage

exitThread = False


class Control:
    def __init__(self, node, port):
        qos_profile = QoSProfile(depth=10)

        self.node = node
        self.port = port  # main()에서 찾은 포트를 전달받음
        self.node.get_logger().info(f"✅ ERP42 연결된 포트: {self.port}")

        # 패킷 기본값 설정
        self.data = bytearray(14)
        self.data[0] = 83  # 'S'
        self.data[1] = 84  # 'T'
        self.data[2] = 88  # 'X'
        self.data[3] = 1  # AORM (Auto or Manual)
        self.data[4] = 0  # ESTOP (Emergency Stop)
        self.data[12] = 13  # ETX0
        self.data[13] = 10  # ETX1
        self.alive = 0

        # ROS 2 Publisher & Subscriber
        self.feedback_pub = node.create_publisher(
            SerialFeedBack, "/erp42_feedback", qos_profile
        )
        self.control_sub = node.create_subscription(
            ControlMessage, "/cmd_msg", self.cmdCallback, qos_profile
        )

        self.feedback_msg = SerialFeedBack()
        self.cmd_msg = ControlMessage()

        # Serial 포트 오픈
        self.ser = serial.Serial(port=self.port, baudrate=115200)

        # 데이터 수신을 위한 Thread 실행
        thread = threading.Thread(target=self.receive_data)
        thread.daemon = True
        thread.start()

    def cmdCallback(self, msg):
        """ROS 2 메시지 콜백 함수"""
        self.cmd_msg = msg

    @staticmethod
    def find_erp42_port():
        """ERP42 포트를 자동 탐색 (USB-to-Serial 변환기만 검사)"""
        baudrate = 115200
        timeout = 2
        wait_time = 10

        ports = [
            port.device
            for port in serial.tools.list_ports.comports()
            if "ttyUSB" in port.device or "ttyACM" in port.device
        ]

        if not ports:
            print(
                "❌ No valid serial ports found (Only checking /dev/ttyUSB* and /dev/ttyACM*)"
            )
            return None

        for port in ports:
            print(f"🔍 Checking {port} for ERP42...")
            ser = serial.Serial(port, baudrate=baudrate)
            # ERP42가 자동으로 피드백을 보내는지 확인
            start_time = time.time()
            while time.time() - start_time < wait_time:
                # TODO 다른 센서와의 구분 필요
                if ser.read():
                    return port

            ser.close()
            print(f"❌ No ERP42 response on {port}")

        return None

    def send_data(self, data=ControlMessage()):
        """ERP42에 명령 패킷 전송"""
        speed = data.speed
        steer = data.steer * 71

        steer = max(min(steer, 1999), -1999)
        self.data[8] = int(steer // 256) if steer >= 0 else int(255 - (-steer // 256))
        self.data[9] = int(steer % 256) if steer >= 0 else int(255 - (-steer % 256))

        self.data[5] = data.gear
        self.data[6] = int(speed // 256)
        self.data[7] = int(speed % 256)
        self.data[10] = data.brake if not data.estop else 200

        self.data[11] = self.alive
        self.ser.write(self.data)

        self.alive = (self.alive + 1) % 256

    def receive_data(self):
        """ERP42에서 데이터 수신"""
        line = []
        while not exitThread:
            try:
                for i in self.ser.read():
                    line.append(i)
                    if i == 83:
                        line = [i]
                    elif i == 10 and len(line) == 18:
                        self.handle_data(line)
                        line = []
                        break
                    elif len(line) >= 18:
                        line = []
                        break
            except Exception as ex:
                print(ex)

    def handle_data(self, line):
        """수신된 패킷을 해석하여 ROS 2 메시지로 변환"""
        feedback_aorm = line[3]
        feedback_estop = line[4]
        feedback_gear = line[5]
        feedback_KPH = (line[6] + line[7] * 256) / 10
        feedback_speed = feedback_KPH * 0.277778

        feedback_DEG = line[8] + line[9] * 256
        feedback_DEG = feedback_DEG - 65536 if feedback_DEG >= 23768 else feedback_DEG
        feedback_steer = m.radians(feedback_DEG / 71.0)

        feedback_brake = line[10]

        data = SerialFeedBack()
        data.mora = feedback_aorm
        data.estop = feedback_estop
        data.gear = feedback_gear
        data.speed = feedback_speed
        data.steer = feedback_steer
        data.brake = feedback_brake

        self.feedback_pub.publish(data)


def main(args=None):
    """ROS 2 노드 실행"""
    rclpy.init(args=args)
    node = rclpy.create_node("erp42_serial")

    # ERP42 포트 탐색
    port = None
    while port is None:
        port = Control.find_erp42_port()
        node.get_logger().info(f"{port}")

        if port is None:
            node.get_logger().warn("🔄 ERP42 포트를 찾을 수 없습니다. 다시 시도 중...")
            time.sleep(2)
    # port = "/dev/ttyUSB0"

    # Control 객체 생성
    control = Control(node=node, port=port)

    # ROS 2 메시지 송신을 위한 루프
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
