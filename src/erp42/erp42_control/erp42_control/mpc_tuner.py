#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

# 기준 대각값(고정)
# [Rk_a, Rk_delta, Rdk_a, Rdk_delta, Qk_x, Qk_y, Qk_v, Qk_yaw, Qf_x, Qf_y, Qf_v, Qf_yaw]
BASE_DIAG = [
    0.72, 200.155,      # Rk: [accel, steering_speed]
    0.845,  648,      # Rdk: [Δaccel, Δsteering_speed]
    24.1754, 30.263, 80, 72.5262,   # Qk: [x, y, v, yaw]
    23.04, 23.04, 36.3, 48.4    # Qfk: [x, y, v, yaw]
]

def curvature_from_xy(xs, ys):
    n = len(xs)
    if n < 3:
        return np.zeros(n)
    k = np.zeros(n)
    for i in range(1, n-1):
        x1, y1 = xs[i-1], ys[i-1]
        x2, y2 = xs[i],   ys[i]
        x3, y3 = xs[i+1], ys[i+1]
        a = math.hypot(x2-x1, y2-y1)
        b = math.hypot(x3-x2, y3-y2)
        c = math.hypot(x3-x1, y3-y1)
        area2 = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)
        denom = a*b*c
        if denom > 1e-8:
            k[i] = 2.0 * area2 / denom
    if n >= 3:
        k[0]  = k[1]
        k[-1] = k[-2]
    return k

def fit_dsteer_schedule(v, ddelta_dt):
    # 95퍼센타일 포락선으로 dδ_max(v) = base / (1 + α v^2) 적합
    v = np.asarray(v); d = np.asarray(ddelta_dt)
    mask = np.isfinite(v) & np.isfinite(d) & (v >= 0.0)
    v = v[mask]; d = np.abs(d[mask])
    if v.size < 200:
        return np.deg2rad(30.0), 0.05  # fallback
    bins = np.linspace(0.0, max(0.1, v.max()), 12)
    centers, y95 = [], []
    for i in range(len(bins)-1):
        m = (v >= bins[i]) & (v < bins[i+1])
        if m.sum() < 30:
            continue
        centers.append(0.5*(bins[i]+bins[i+1]))
        y95.append(np.percentile(d[m], 95))
    centers = np.array(centers); y95 = np.array(y95)
    if centers.size < 3:
        return np.deg2rad(30.0), 0.05
    Y = 1.0/np.clip(y95, 1e-6, None)
    X = np.vstack([np.ones_like(centers), centers**2]).T
    beta, *_ = np.linalg.lstsq(X, Y, rcond=None)
    inv_base, alpha_over_base = beta
    base  = 1.0 / max(inv_base, 1e-6)
    alpha = max(alpha_over_base*base, 0.0)
    return base, alpha

def nearest_point_xy(px, py, xs, ys):
    pts = np.stack([xs, ys], axis=1)
    diffs = pts[1:, :] - pts[:-1, :]
    l2s = (diffs**2).sum(axis=1)
    dots = ((np.array([px, py]) - pts[:-1, :]) * diffs).sum(axis=1)
    t = np.clip(dots / np.clip(l2s, 1e-9, None), 0.0, 1.0)
    proj = pts[:-1, :] + (t[:, None] * diffs)
    d2 = ((proj - np.array([px, py]))**2).sum(axis=1)
    i = int(np.argmin(d2))
    return proj[i, 0], proj[i, 1], i

class ParamEstimator(Node):
    def __init__(self):
        super().__init__("mpc_param_estimator")

        # --- 파라미터 ---
        self.declare_parameter("wheelbase", 1.040)
        self.declare_parameter("alat_percentile", 85.0)
        self.declare_parameter("window_sec", 30.0)
        self.declare_parameter("hz", 10.0)
        self.declare_parameter("max_speed", 8.0)
        self.declare_parameter("emit_period", 1.0)

        self.WB   = float(self.get_parameter("wheelbase").value)
        self.Nwin = max(10, int(self.get_parameter("window_sec").value *
                                self.get_parameter("hz").value))
        self.MAX_SPEED  = float(self.get_parameter("max_speed").value)
        self.percentile = float(self.get_parameter("alat_percentile").value)
        self.emit_period= float(self.get_parameter("emit_period").value)

        # --- 버퍼 ---
        self.odom_buf = deque(maxlen=self.Nwin)   # (t,x,y,yaw,v,r)
        self.ref_xy   = None
        self.pred_xy  = None
        self.wp_xy    = None

        # --- 구독 ---
        self.create_subscription(Marker, "/ref_traj_marker",  self.cb_ref,  10)
        self.create_subscription(Marker, "/pred_path_marker", self.cb_pred, 10)
        self.create_subscription(Marker, "/waypoints_marker", self.cb_wp,   10)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.cb_odom, 20)

        # --- 타이머 ---
        self.create_timer(self.emit_period, self.tick)
        self.get_logger().info("ParamEstimator started. Logging diagonals from live estimates.")

    # --------- 콜백들 ---------
    def cb_ref(self, msg: Marker):
        if msg.type != Marker.LINE_STRIP or not msg.points:
            return
        xs = np.array([p.x for p in msg.points], dtype=float)
        ys = np.array([p.y for p in msg.points], dtype=float)
        self.ref_xy = (xs, ys)

    def cb_pred(self, msg: Marker):
        if msg.type != Marker.LINE_STRIP or not msg.points:
            return
        xs = np.array([p.x for p in msg.points], dtype=float)
        ys = np.array([p.y for p in msg.points], dtype=float)
        self.pred_xy = (xs, ys)

    def cb_wp(self, msg: Marker):
        if not msg.points:
            return
        xs = np.array([p.x for p in msg.points], dtype=float)
        ys = np.array([p.y for p in msg.points], dtype=float)
        self.wp_xy = (xs, ys)

    def cb_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        r = msg.twist.twist.angular.z
        self.odom_buf.append((t, x, y, yaw, v, r))

    # --------- 핵심 계산 ---------
    def tick(self):
        if not self.odom_buf or self.ref_xy is None:
            return

        arr = np.array(self.odom_buf, dtype=float)
        t  = arr[:,0]; xs = arr[:,1]; ys = arr[:,2]; yaws = arr[:,3]; v = arr[:,4]; r = arr[:,5]
        v = np.clip(v, 0.01, None)

        # 실제 곡률/가로가속
        kappa_actual = curvature_from_xy(xs, ys)
        alat = (v**2) * np.abs(kappa_actual)
        A_LAT_MAX = float(np.percentile(alat[np.isfinite(alat)], self.percentile)) if alat.size else 2.0
        if not np.isfinite(A_LAT_MAX) or A_LAT_MAX < 0.2:
            A_LAT_MAX = 2.0

        # δ, dδ/dt
        delta_actual = np.arctan(self.WB * r / v)
        ddelta_dt = np.gradient(delta_actual, t) if t.size >= 3 else np.zeros_like(delta_actual)
        base, alpha = fit_dsteer_schedule(v, ddelta_dt)

        # 참조 기반 속도 상한
        ref_xs, ref_ys = self.ref_xy
        kappa_ref = curvature_from_xy(ref_xs, ref_ys)
        eps = 1e-3
        v_cap = np.sqrt(np.maximum(0.0, A_LAT_MAX / (np.abs(kappa_ref) + eps)))
        v_cap = np.clip(v_cap, 0.0, self.MAX_SPEED)

        # 현재 ref 인덱스와 근방
        px, py = xs[-1], ys[-1]
        _, _, i_ref = nearest_point_xy(px, py, ref_xs, ref_ys)
        i0 = max(0, i_ref - 50); i1 = min(len(ref_xs), i_ref + 50)
        v_ref_local = v_cap[i0:i1]
        ref_local_x = ref_xs[i0:i1]; ref_local_y = ref_ys[i0:i1]

        # 횡오차/헤딩오차
        ex, ey, _ = nearest_point_xy(xs[-1], ys[-1], ref_local_x, ref_local_y)
        e_y = math.hypot(xs[-1]-ex, ys[-1]-ey)
        if (i1 - i0) >= 2:
            j = np.clip(i_ref - i0, 1, (i1 - i0) - 1)
            tx = ref_local_x[j] - ref_local_x[j-1]
            ty = ref_local_y[j] - ref_local_y[j-1]
            ref_yaw = math.atan2(ty, tx)
        else:
            ref_yaw = yaws[-1]
        e_psi = math.atan2(math.sin(yaws[-1]-ref_yaw), math.cos(yaws[-1]-ref_yaw))

        # 속도 오차
        v_ref_here = float(np.median(v_ref_local)) if v_ref_local.size else self.MAX_SPEED
        e_v = v[-1] - v_ref_here

        # 진동 지표 (e_y 도함수 RMS)
        if xs.size >= 5:
            ey_series = []
            for k in range(xs.size):
                exk, eyk, _ = nearest_point_xy(xs[k], ys[k], ref_local_x, ref_local_y)
                ey_series.append(math.hypot(xs[k]-exk, ys[k]-eyk))
            ey_series = np.array(ey_series)
            ey_dot = np.gradient(ey_series, t[:ey_series.size])
            OI = float(np.sqrt(np.mean(ey_dot**2)))
        else:
            OI = 0.0

        # 정규화
        OI_ref, Lag_ref, VE_ref = 0.3, 0.2, 0.5
        OI_norm  = np.clip(OI / OI_ref, 0.0, 2.0)
        Lag_norm = np.clip(abs(e_psi) / Lag_ref, 0.0, 2.0)
        VE_norm  = np.clip(abs(e_v) / VE_ref, 0.0, 2.0)

        # 스케일(실시간 추정)
        scales = {
            "Rk_a":      1.0 + 0.10*VE_norm,
            "Rk_delta":  1.0 + 0.30*OI_norm - 0.10*Lag_norm,
            "Rdk_a":     1.0 + 0.15*VE_norm,
            "Rdk_delta": 1.0 + 0.40*OI_norm,
            "Qk_x":      1.0 + 0.30*Lag_norm,
            "Qk_y":      1.0 + 0.40*Lag_norm,
            "Qk_v":      1.0 + 0.50*VE_norm,
            "Qk_yaw":    1.0 + 0.30*Lag_norm,
            "Qf_x":      1.2,
            "Qf_y":      1.2,
            "Qf_v":      1.1,
            "Qf_yaw":    1.1,
        }

        # >>> 최종 대각값 12개 (현재에 맞게 추정된 수치)
        diag_vec = [
            BASE_DIAG[0]  * scales["Rk_a"],
            BASE_DIAG[1]  * scales["Rk_delta"],
            BASE_DIAG[2]  * scales["Rdk_a"],
            BASE_DIAG[3]  * scales["Rdk_delta"],
            BASE_DIAG[4]  * scales["Qk_x"],
            BASE_DIAG[5]  * scales["Qk_y"],
            BASE_DIAG[6]  * scales["Qk_v"],
            BASE_DIAG[7]  * scales["Qk_yaw"],
            BASE_DIAG[8]  * scales["Qf_x"],
            BASE_DIAG[9]  * scales["Qf_y"],
            BASE_DIAG[10] * scales["Qf_v"],
            BASE_DIAG[11] * scales["Qf_yaw"],
        ]

        # ------- 콘솔 로그 (원하신 형식으로) -------
        self.get_logger().info(
            f"A_LAT_MAX={A_LAT_MAX:.2f}, dδ base={math.degrees(base):.1f}°/s, α={alpha:.3f}, "
            f"e_y={e_y:.2f} m, e_psi={math.degrees(e_psi):.1f}°, e_v={e_v:.2f} m/s, OI={OI:.2f}"
        )
        self.get_logger().info(
            "Rk=[{:.6g}, {:.6g}], Rdk=[{: .6g}, {: .6g}], "
            "Qk=[{:.6g}, {:.6g}, {:.6g}, {:.6g}], "
            "Qfk=[{:.6g}, {:.6g}, {:.6g}, {:.6g}]".format(
                diag_vec[0], diag_vec[1],
                diag_vec[2], diag_vec[3],
                diag_vec[4], diag_vec[5], diag_vec[6], diag_vec[7],
                diag_vec[8], diag_vec[9], diag_vec[10], diag_vec[11]
            )
        )
        # (원하면 한 줄로 전체 벡터도)
        # self.get_logger().info(f"DIAGONALS={diag_vec}")

def main():
    rclpy.init()
    node = ParamEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
