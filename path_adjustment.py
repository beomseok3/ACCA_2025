#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Humble node: DB-backed path post-processor + visualizer.

- Publishes the SQLite path like db_read.py on topics:
  * db_path (nav_msgs/Path)
  * db_markers (visualization_msgs/MarkerArray)
- Collects RViz clicks:
  * /initialpose (PoseWithCovarianceStamped): stored in order
  * /goal_pose (PoseStamped): trigger — ONLY the last two initial poses are used
- Post-process between nearest DB points A,B:
  * middle 70%: straight line
  * first/last 15%: cubic Hermite blends using DB yaw near A/B and line yaw at S/E
  * spacing enforced at exactly `spacing` (default 0.1 m)
- Updates the SQLite DB persistently:
  * Replaces Path rows between indices [a,b] with the modified samples
  * Renumbers the whole Path table idx to remain contiguous [0..N-1]
  * Assigns each new point a path_id mapped from the original segment by arc-length so path_id changes remain consistent
- Republishes the updated DB path & markers.

Run:
  python3 db_path_postprocessor_update_ros2.py \
    --ros-args -p db_path:=/absolute/path/bs_v4.db \
               -p spacing:=0.1 \
               -p frame_id:=map \
               -p path_table:=Path \
               -p node_table:=Node \
               -p initialpose_topic:=/initialpose \
               -p goal_topic:=/goal_pose \
               -p backup_on_update:=true

Note:
- If you already run another node publishing on 'db_path'/'db_markers', stop it to avoid topic conflicts,
  or change the topic names here via remapping.
"""

import math
import sqlite3
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler

# ----------------- math helpers -----------------

def yaw_to_vec(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw), math.sin(yaw)], dtype=float)

def line_dir(p0: np.ndarray, p1: np.ndarray) -> np.ndarray:
    v = np.asarray(p1, float) - np.asarray(p0, float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.array([1.0, 0.0], dtype=float)
    return v / n

def cubic_hermite(p0, p1, m0, m1, t):
    t2 = t * t
    t3 = t2 * t
    h00 =  2*t3 - 3*t2 + 1
    h10 =      t3 - 2*t2 + t
    h01 = -2*t3 + 3*t2
    h11 =      t3 -   t2
    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1

def build_tangent(yaw_dir: float, chord_len: float, tau: float = 0.8) -> np.ndarray:
    return tau * chord_len * yaw_to_vec(yaw_dir)

def cumulative_lengths(xy: np.ndarray) -> np.ndarray:
    if xy.shape[0] == 0:
        return np.zeros((0,), float)
    diffs = np.diff(xy, axis=0)
    seg = np.linalg.norm(diffs, axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    return s

def resample_by_spacing(xy: np.ndarray, spacing: float) -> Tuple[np.ndarray, np.ndarray]:
    """Resample polyline to exact spacing; returns (xy_new, yaw_new)."""
    if xy.shape[0] < 2:
        yaws = np.zeros((xy.shape[0],), float)
        return xy, yaws
    s = cumulative_lengths(xy)
    total = s[-1]
    if total < spacing:
        # keep endpoints
        yaws = np.zeros((xy.shape[0],), float)
        if xy.shape[0] >= 2:
            d = xy[-1] - xy[-2]
            yaws[-1] = math.atan2(d[1], d[0])
        return xy, yaws
    new_s = np.arange(0.0, total + 1e-6, spacing)
    x_new = np.interp(new_s, s, xy[:,0])
    y_new = np.interp(new_s, s, xy[:,1])
    xy_new = np.column_stack([x_new, y_new])
    # yaw from forward diff
    fwd = np.vstack([np.diff(xy_new, axis=0), xy_new[-1] - xy_new[-2]])
    yaw_new = np.array([math.atan2(v[1], v[0]) if np.linalg.norm(v) > 1e-12 else 0.0 for v in fwd])
    return xy_new, yaw_new

# ----------------- DB helpers -----------------

class DBAccessor:
    def __init__(self, db_path: str, path_table='Path', node_table='Node'):
        self.db_path = db_path
        self.path_table = path_table
        self.node_table = node_table

    def backup(self, suffix: str = '.bak'):
        import shutil
        dst = self.db_path + suffix
        shutil.copy2(self.db_path, dst)
        return dst

    def fetch_all(self) -> List[Tuple[str,int,float,float,float,float]]:
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        cur.execute(f"SELECT path_id, idx, x, y, yaw, speed FROM {self.path_table} ORDER BY idx ASC")
        rows = cur.fetchall()
        conn.close()
        return rows

    def replace_all(self, rows: List[Tuple[str,int,float,float,float,float]]):
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        cur.execute(f"DELETE FROM {self.path_table}")
        cur.executemany(
            f"INSERT INTO {self.path_table} (path_id, idx, x, y, yaw, speed) VALUES (?,?,?,?,?,?)",
            rows
        )
        conn.commit()
        conn.close()

# ----------------- Core processor -----------------

class PathData:
    def __init__(self, rows: List[Tuple[str,int,float,float,float,float]]):
        # rows: (path_id, idx, x, y, yaw, speed)
        self.path_id = np.array([r[0] for r in rows])
        self.idx     = np.array([r[1] for r in rows], dtype=int)
        self.xy      = np.column_stack([[r[2] for r in rows], [r[3] for r in rows]]).astype(float)
        self.yaw     = np.array([r[4] for r in rows], dtype=float)
        self.speed   = np.array([0.0 if r[5] is None else r[5] for r in rows], dtype=float)

    def slice(self, a: int, b: int) -> 'PathData':
        s = slice(a, b+1)
        rows = [(self.path_id[i], int(self.idx[i]), float(self.xy[i,0]), float(self.xy[i,1]), float(self.yaw[i]), float(self.speed[i])) for i in range(a, b+1)]
        return PathData(rows)

def hermite_blend_straighten_segment(xy: np.ndarray, yaw: np.ndarray, a: int, b: int, spacing: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns new (xy_resampled, yaw_resampled) for segment [a,b] after:
      - 15% blend (A->S) with yaw(A-1) and straight-line yaw at S
      - 70% straight (S->E)
      - 15% blend (E->B) with straight-line yaw at E and yaw(B+1)
    """
    a, b = sorted([a, b])
    seg_xy = xy[a:b+1]
    n = seg_xy.shape[0]
    if n < 10:
        return seg_xy.copy(), yaw[a:b+1].copy()
    
    ############################################################
    blend_ratio = 0.20   # 양쪽 블렌드 구간 비율
    straight_ratio = 0.60 # 가운데 직선 구간 비율
    ############################################################ 비율 조정 하는 곳

    n_blend = max(2, int(round(n * blend_ratio)))
    n_straight = max(2, int(round(n * straight_ratio)))
    idx_S = a + n_blend
    idx_E = b - n_blend

    P_A = xy[a]
    P_B = xy[b]
    P_S = xy[idx_S]
    P_E = xy[idx_E]

    dir_SE = line_dir(P_S, P_E)
    yaw_S = math.atan2(dir_SE[1], dir_SE[0])

    # Tangents
    yaw_A = yaw[a-1] if a-1 >= 0 else yaw[a]
    yaw_B = yaw[b+1] if b+1 < yaw.shape[0] else yaw[b]

    chord_AS = np.linalg.norm(P_S - P_A)
    chord_EB = np.linalg.norm(P_B - P_E)

    m0_AS = build_tangent(yaw_A, chord_AS, tau=0.8)
    m1_AS = build_tangent(yaw_S, chord_AS, tau=0.8)

    m0_EB = build_tangent(yaw_S, chord_EB, tau=0.8)
    m1_EB = build_tangent(yaw_B, chord_EB, tau=0.8)

    # Dense sampling before uniform resample for stability
    def sample_hermite(p0, p1, m0, m1, step=0.02):
        ts = np.arange(0.0, 1.0 + 1e-9, step)
        return np.vstack([cubic_hermite(p0, p1, m0, m1, t) for t in ts])

    as_xy = sample_hermite(P_A, P_S, m0_AS, m1_AS, step=0.02)
    eb_xy = sample_hermite(P_E, P_B, m0_EB, m1_EB, step=0.02)
    line_xy = np.vstack([P_S, P_E])

    # stitch and resample by spacing
    raw_xy = np.vstack([as_xy[:-1], line_xy, eb_xy[1:]])
    xy_res, yaw_res = resample_by_spacing(raw_xy, spacing)
    return xy_res, yaw_res

def map_new_points_to_old_path_ids(path_ids_old: np.ndarray, xy_old: np.ndarray, new_xy: np.ndarray) -> np.ndarray:
    """
    Assign a path_id to each new point by nearest arc-length mapping on the original segment.
    """
    s_old = cumulative_lengths(xy_old)
    total_old = s_old[-1] if s_old.shape[0] > 0 else 0.0
    if total_old <= 0.0:
        return np.full((new_xy.shape[0],), path_ids_old[0], dtype=object)

    s_new = cumulative_lengths(new_xy)
    s_new_norm = s_new / s_new[-1] if s_new[-1] > 0 else s_new
    targets = s_new_norm * total_old

    # vectorized nearest index via searchsorted
    import bisect
    idxs = np.searchsorted(s_old, targets, side='left')
    idxs = np.clip(idxs, 0, len(s_old)-1)
    pid = path_ids_old[idxs]
    return pid

# ----------------- ROS2 Node -----------------

class DBPathEditor(Node):
    def __init__(self):
        super().__init__('db_path_postprocessor')

        # parameters
        self.declare_parameter('db_path', '/home/acca/db_file/YS/kcity_6th_ys_v1.db')
        self.declare_parameter('path_table', 'Path')
        self.declare_parameter('node_table', 'Node')
        self.declare_parameter('spacing', 0.1)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('backup_on_update', True)

        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value
        self.path_table = self.get_parameter('path_table').get_parameter_value().string_value
        self.node_table = self.get_parameter('node_table').get_parameter_value().string_value
        self.spacing = float(self.get_parameter('spacing').get_parameter_value().double_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.initialpose_topic = self.get_parameter('initialpose_topic').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.backup_on_update = bool(self.get_parameter('backup_on_update').get_parameter_value().bool_value)

        self.db = DBAccessor(self.db_path, self.path_table, self.node_table)

        qos = QoSProfile(depth=10)
        self.pub_path = self.create_publisher(RosPath, 'db_path', qos)
        self.pub_markers = self.create_publisher(MarkerArray, 'db_markers', qos)

        self.sub_initialpose = self.create_subscription(PoseWithCovarianceStamped, self.initialpose_topic, self.cb_initialpose, 10)
        self.sub_goal = self.create_subscription(PoseStamped, self.goal_topic, self.cb_goal, 10)

        # Load and publish initial path
        self.rows = self.db.fetch_all()
        self.path = PathData(self.rows)
        self.clicked = []  # [(x,y), ...]

        self.publish_all()
        self.get_logger().info(f"Loaded DB: {len(self.rows)} points, spacing target={self.spacing} m.")

    # ------------ RViz callbacks ------------
    def cb_initialpose(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.clicked.append((x, y))
        self.get_logger().info(f"Initial pose #{len(self.clicked)} recorded at ({x:.3f},{y:.3f})")

    def cb_goal(self, msg: PoseStamped):
        if len(self.clicked) < 2:
            self.get_logger().warn("Need >= 2 initial poses before goal trigger.")
            return
        p1, p2 = self.clicked[-2], self.clicked[-1]
        self.get_logger().info(f"Goal received. Using last two initial poses: {p1} and {p2}")

        # find nearest A,B indices
        a = self.closest_index(p1[0], p1[1])
        b = self.closest_index(p2[0], p2[1])
        a, b = (a, b) if a <= b else (b, a)
        self.get_logger().info(f"Nearest DB indices: A={a}, B={b} (count {b-a+1})")

        # post-process that segment
        new_xy_seg, new_yaw_seg = hermite_blend_straighten_segment(self.path.xy, self.path.yaw, a, b, self.spacing)

        # map new points -> old path_id by original arc-length positions
        old_pid_seg = self.path.path_id[a:b+1]
        old_xy_seg  = self.path.xy[a:b+1]
        new_pid_seg = map_new_points_to_old_path_ids(old_pid_seg, old_xy_seg, new_xy_seg)

        # speed ramp across new segment
        sp_a, sp_b = float(self.path.speed[a]), float(self.path.speed[b])
        if new_xy_seg.shape[0] <= 1:
            new_spd_seg = np.array([sp_a], float)
        else:
            new_spd_seg = np.linspace(sp_a, sp_b, new_xy_seg.shape[0])

        # assemble new full rows list, renumber idx 0..N-1
        new_rows_before = [(self.path.path_id[i], int(i), float(self.path.xy[i,0]), float(self.path.xy[i,1]), float(self.path.yaw[i]), float(self.path.speed[i])) for i in range(0, a)]
        new_rows_seg    = [(str(new_pid_seg[i]), 0, float(new_xy_seg[i,0]), float(new_xy_seg[i,1]), float(new_yaw_seg[i]), float(new_spd_seg[i])) for i in range(new_xy_seg.shape[0])]
        new_rows_after  = [(self.path.path_id[i], 0, float(self.path.xy[i,0]), float(self.path.xy[i,1]), float(self.path.yaw[i]), float(self.path.speed[i])) for i in range(b+1, self.path.xy.shape[0])]
        merged = new_rows_before + new_rows_seg + new_rows_after

        # renumber idx
        merged_idxed = [(r[0], i, r[2], r[3], r[4], r[5]) for i, r in enumerate(merged)]

        # backup and replace
        if self.backup_on_update:
            bak = self.db.backup('.bak')
            self.get_logger().info(f"Backup created: {bak}")
        self.db.replace_all(merged_idxed)

        # reload memory and publish
        self.rows = self.db.fetch_all()
        self.path = PathData(self.rows)
        self.publish_all()
        self.get_logger().info(f"DB updated and republished. New total points: {self.path.xy.shape[0]}")

    # ------------ utilities ------------
    def closest_index(self, x: float, y: float) -> int:
        d2 = (self.path.xy[:,0]-x)**2 + (self.path.xy[:,1]-y)**2
        return int(np.argmin(d2))

    def publish_all(self):
        # publish path (thin)
        path_msg = RosPath()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id

        now = self.get_clock().now().to_msg()

        markers = MarkerArray()

        # clear all
        delete_all = Marker()
        delete_all.header.frame_id = self.frame_id
        delete_all.header.stamp = now
        delete_all.ns = "db_visual"
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        last_pid = None
        marker_id = 1
        current_line = None

        for i in range(self.path.xy.shape[0]):
            x, y = self.path.xy[i]
            yv = float(self.path.yaw[i])
            pid = self.path.path_id[i]

            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yv)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

            # if path_id changed, flush previous line and add markers
            if last_pid is None or pid != last_pid:
                if current_line is not None:
                    markers.markers.append(current_line)

                current_line = Marker()
                current_line.header.frame_id = self.frame_id
                current_line.header.stamp = now
                current_line.ns = "db_visual/path_line"
                current_line.id = marker_id; marker_id += 1
                current_line.type = Marker.LINE_STRIP
                current_line.action = Marker.ADD
                current_line.pose.orientation.w = 1.0
                current_line.scale.x = 0.2
                current_line.color.r = 0.0
                current_line.color.g = 0.6
                current_line.color.b = 1.0
                current_line.color.a = 0.9
                current_line.lifetime.sec = 0

                # sphere at change
                m = Marker()
                m.header.frame_id = self.frame_id
                m.header.stamp = now
                m.ns = "db_visual/path_change"
                m.id = marker_id; marker_id += 1
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(x)
                m.pose.position.y = float(y)
                m.pose.position.z = 0.15
                m.pose.orientation.w = 1.0
                m.scale.x = 1.0; m.scale.y = 1.0; m.scale.z = 1.0
                m.color.r = 1.0; m.color.g = 0.3; m.color.b = 0.0; m.color.a = 0.9
                m.lifetime.sec = 0
                markers.markers.append(m)

                # text marker
                t = Marker()
                t.header.frame_id = self.frame_id
                t.header.stamp = now
                t.ns = "db_visual/path_change_text"
                t.id = marker_id; marker_id += 1
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = float(x)
                t.pose.position.y = float(y)
                t.pose.position.z = 0.6
                t.pose.orientation.w = 1.0
                t.scale.z = 0.9
                t.color.r = 1.0; t.color.g = 0.8; t.color.b = 0.0; t.color.a = 1.0
                t.text = f"path_id: {pid}"
                t.lifetime.sec = 0
                markers.markers.append(t)

                last_pid = pid

            if current_line is not None:
                pt = Point()
                pt.x = float(x); pt.y = float(y); pt.z = 0.0
                current_line.points.append(pt)

            # speed labels every 5th
            if i % 5 == 0:
                s = Marker()
                s.header.frame_id = self.frame_id
                s.header.stamp = now
                s.ns = "db_visual/speed"
                s.id = marker_id; marker_id += 1
                s.type = Marker.TEXT_VIEW_FACING
                s.action = Marker.ADD
                s.pose.position.x = float(x) + 0.2
                s.pose.position.y = float(y) + 0.2
                s.pose.position.z = 0.4
                s.pose.orientation.w = 1.0
                s.scale.z = 0.25
                s.color.r = 0.1; s.color.g = 0.9; s.color.b = 0.1; s.color.a = 0.95
                s.text = f"{float(self.path.speed[i]):.2f} m/s"
                s.lifetime.sec = 0
                markers.markers.append(s)

        if current_line is not None:
            markers.markers.append(current_line)

        self.pub_path.publish(path_msg)
        self.pub_markers.publish(markers)

def main():
    rclpy.init()
    node = DBPathEditor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()