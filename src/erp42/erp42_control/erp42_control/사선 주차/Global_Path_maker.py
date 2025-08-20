import rclpy
from rclpy.node import Node
import sqlite3
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import os


class Pose:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0


class PathMaker(Node):
    def __init__(self):
        super().__init__("global_path_maker")

        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.create_timer(0.1, self.timer_callback)

        self.odom = Pose()
        self.current_id = 1

        self.prev_x = None
        self.prev_y = None
        self.move_threshold = 0.05

        self.path_id = "A1A2"
        self.idx = 0
        self.speed = 5

        self.init_db()

    # P_parking_path_2
    # P_return_path_1
    def init_db(self):
        db_path = os.path.join(os.path.dirname(__file__), "U-turn_OUT.db")
        self.conn = sqlite3.connect(db_path)
        self.cursor = self.conn.cursor()

        # Node 테이블 생성
        self.cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS Node(
                Start_point CHAR(4),
                End_point   CHAR(4),
                path_id     CHAR(4) PRIMARY KEY,
                mission     CHAR(10)
            );
            """
        )

        # Path 테이블 생성
        self.cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS Path(
                path_id CHAR(4),
                idx     INTEGER PRIMARY KEY,
                x       REAL,
                y       REAL,
                yaw     REAL,
                speed   REAL,
                FOREIGN KEY(path_id) REFERENCES Node(path_id)
                    ON DELETE CASCADE ON UPDATE CASCADE
            );
            """
        )

        # 기존 idx 불러오기
        self.cursor.execute(
            "SELECT MAX(idx) FROM Path WHERE path_id = ?", (self.path_id,)
        )
        result = self.cursor.fetchone()[0]
        if result is not None:
            self.idx = result + 1
        else:
            self.idx = 0

        self.conn.commit()

    def save_to_db(self):
        self.cursor.execute(
            """
            INSERT INTO Path (path_id, idx, x, y, yaw, speed)
            VALUES (?, ?, ?, ?, ?, ?)
            """,
            (
                self.path_id,
                self.idx,
                self.odom.x,
                self.odom.y,
                self.odom.yaw,
                self.speed,
            ),
        )
        self.conn.commit()
        self.idx += 1

    def has_moved_significantly(self, x, y):
        if self.prev_x is None or self.prev_y is None:
            return True
        distance = math.sqrt((x - self.prev_x) ** 2 + (y - self.prev_y) ** 2)
        return distance > self.move_threshold

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        x, y = position.x, position.y
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = euler_from_quaternion(quaternion)[2]

        if self.has_moved_significantly(x, y):
            self.odom.x = x
            self.odom.y = y
            self.odom.yaw = yaw
            self.save_to_db()
            self.prev_x = x
            self.prev_y = y

    def goal_pose_callback(self, msg):
        self.current_id += 1
        # 필요시 self.path_id 변경 가능
        # self.path_id = f"A{self.current_id}"

    def timer_callback(self):
        self.get_logger().info(
            f"ID: {self.current_id}, "
            f"Pos: ({self.odom.x:.3f}, {self.odom.y:.3f}), Yaw: {self.odom.yaw:.3f}, "
            f"path_id: {self.path_id}, idx: {self.idx}"
        )

    def __del__(self):
        self.conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = PathMaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
