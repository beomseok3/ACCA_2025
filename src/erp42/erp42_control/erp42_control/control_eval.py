#!/usr/bin/env python3
# lateral_error_node.py
#
# ros2 run <your_pkg> lateral_error_node --ros-args \
#        -p pose_topic:=/localization/kinematic_state \
#        -p path_topic:=/global_path          # ★ 파라미터 이름도 바꿈
import math, rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path           # ★ Path import
from std_msgs.msg import Float32, Float32MultiArray

def pose_to_xy(pose_stamped):
    """geometry_msgs/PoseStamped → np.array([x, y])"""
    return np.array([pose_stamped.pose.position.x,
                     pose_stamped.pose.position.y], dtype=float)

class LateralErrorNode(Node):
    def __init__(self):
        super().__init__('lateral_error_node')

        qos = QoSProfile(history=HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                         depth=10,
                         reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE)

        pose_topic = self.declare_parameter(
            'pose_topic', '/localization/kinematic_state').get_parameter_value().string_value
        path_topic = self.declare_parameter(          # ★ path_topic 파라미터
            'path_topic', '/global_path').get_parameter_value().string_value

        # 구독
        self.pose_sub = self.create_subscription(Odometry, pose_topic,
                                                 self.pose_cb, qos)
        self.path_sub = self.create_subscription(Path, path_topic,   # ★ Path 구독
                                                 self.path_cb, qos)

        # 발행
        self.err_pub   = self.create_publisher(Float32,           '/lateral_error',       10)
        self.stats_pub = self.create_publisher(Float32MultiArray, '/lateral_error_stats', 10)

        # 내부 상태
        self.path_pts, self.seg_dirs, self.seg_len = None, None, None
        self.cum_mean = self.cum_sqsum = 0.0
        self.count = 0

    # --------------------------------------------------
    def build_segments(self):
        if self.path_pts is None or len(self.path_pts) < 2:
            return
        seg_vecs = np.diff(self.path_pts, axis=0)
        self.seg_len = np.linalg.norm(seg_vecs, axis=1)
        mask = self.seg_len > 1e-6                  # 0-길이 세그먼트 제거
        self.seg_dirs = (seg_vecs[mask].T / self.seg_len[mask]).T
        self.seg_len  = self.seg_len[mask]

    # --------------------------------------------------
    def path_cb(self, msg: Path):                   # ★ Marker → Path 콜백
        if not msg.poses:
            self.get_logger().warn('Received empty Path – ignoring')
            return
        self.path_pts = np.vstack([pose_to_xy(ps) for ps in msg.poses])
        self.build_segments()
        self.get_logger().info(f'Path received: {len(self.path_pts)} points')

    # --------------------------------------------------
    def pose_cb(self, msg: Odometry):
        if self.seg_dirs is None:   # 아직 Path 안 받음
            return

        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

        v  = p - self.path_pts[:-1][self.seg_len > 1e-6]            # N-1 x 2
        t  = np.einsum('ij,ij->i', v, self.seg_dirs)                # 투영 스칼라
        proj = self.path_pts[:-1][self.seg_len > 1e-6] + \
               np.clip(t, 0.0, self.seg_len)[:, None] * self.seg_dirs

        dists = np.linalg.norm(p - proj, axis=1)
        idx   = int(np.argmin(dists))

        d, dir_vec = p - proj[idx], self.seg_dirs[idx]
        sign = math.copysign(1.0, dir_vec[0]*d[1] - dir_vec[1]*d[0])
        err  = sign * dists[idx]

        # ---- publish error
        self.err_pub.publish(Float32(data=float(err)))

        # ---- accumulate stats
        self.count     += 1
        self.cum_mean  += err
        self.cum_sqsum += err*err

        if self.count % 100 == 0:
            mean = self.cum_mean / self.count
            rms  = math.sqrt(self.cum_sqsum / self.count)
            stats_msg = Float32MultiArray()
            stats_msg.data = [float(self.count), float(mean), float(rms)]
            self.stats_pub.publish(stats_msg)
            self.get_logger().info(f'[{self.count:>6}] mean={mean:.3f} m  rms={rms:.3f} m')

# ------------------------------------------------------
def main():
    rclpy.init()
    node = LateralErrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.count:
            mean = node.cum_mean / node.count
            rms  = math.sqrt(node.cum_sqsum / node.count)
            node.get_logger().info(f'Final  mean={mean:.3f} m  rms={rms:.3f} m')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
