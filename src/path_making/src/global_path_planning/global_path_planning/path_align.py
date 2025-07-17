#!/usr/bin/env python3
# align_db_path_to_pose.py
#
# ① DB  에서 path_id 로 경로(x,y,yaw) 읽기
# ② 기준 Pose(위치·방향)와 첫 점의 Pose 를 비교 → Δθ(회전)·t(평행이동) 계산
# ③ Δθ, t 를 전체 경로에 적용 → (x′,y′,yaw′) 생성
# ④ 결과를 같은 DB  안에 새 path_id 로 저장하거나 파일/토픽으로 출력
#
# 사용:
#   python align_db_path_to_pose.py my_route.db A1A2  \
#          --pos -18.18885538761073 -53.23430932878838 \
#          --quat 0 0 -0.08072402688659329 0.9967364904944599 \
#          --new-id B1B2
#
#   (DB 경로·path_id·새 path_id 는 상황에 맞게 수정)
#
import argparse
import numpy as np
from math import cos, sin
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# ----  DB 래퍼 (사용자가 올린 DB.py) -----------------------------------------
from DB import DB                                             # :contentReference[oaicite:0]{index=0}:contentReference[oaicite:1]{index=1}
# ---------------------------------------------------------------------------


def quat_to_yaw(qx, qy, qz, qw):
    """quaternion → yaw(rad)  (roll/pitch 는 무시)"""
    _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
    return yaw


def compute_transform(x_ref, y_ref, yaw_ref,
                      x0, y0, yaw0):
    """
    경로 첫 점(p0) → 기준 Pose 로 옮기기 위한
    (Δθ, 2×2 R, 2×1 t) 반환
    """
    dtheta = yaw_ref - yaw0
    R = np.array([[cos(dtheta), -sin(dtheta)],
                  [sin(dtheta),  cos(dtheta)]])
    t = np.array([x_ref, y_ref]) - R @ np.array([x0, y0])
    return dtheta, R, t


def transform_path(rows, dtheta, R, t):
    """
    rows = [(x, y, yaw), …]  →  (x′, y′, yaw′) 리스트로 변환
    """
    out = []
    for x, y, yaw in rows:
        p_ = R @ np.array([x, y]) + t
        yaw_ = yaw + dtheta
        out.append((p_[0], p_[1], yaw_))
    return out


def main():
    parser = argparse.ArgumentParser(description="Align DB path to target pose")
    parser.add_argument("db_name",      help="SQLite 파일 명")
    parser.add_argument("path_id",      help="원본 path_id (4글자)")
    parser.add_argument("--pos",  nargs=2,  type=float, required=True,
                        metavar=("X", "Y"),
                        help="기준 Pose 의 x y (m)")
    parser.add_argument("--quat", nargs=4, type=float, required=True,
                        metavar=("qx", "qy", "qz", "qw"),
                        help="기준 Pose 의 quaternion (x y z w)")
    parser.add_argument("--new-id",     default=None,
                        help="변환된 경로를 저장할 새 path_id (4글자, 기본: 원본 덮어쓰기)")
    args = parser.parse_args()

    # ---- 0. DB 접속 --------------------------------------------------------
    db = DB(args.db_name)

    # ---- 1. DB → (x,y,yaw) 리스트 -----------------------------------------
    rows = db.read_from_id_to_path(args.path_id)
    if not rows:
        raise RuntimeError(f"path_id '{args.path_id}' 경로가 DB 에 없습니다.")

    # rows[(x,y,yaw)] 구조이므로 첫 점:
    x0, y0, yaw0 = rows[0]

    # ---- 2. 기준 Pose → yaw_ref ------------------------------------------
    yaw_ref = quat_to_yaw(*args.quat)

    # ---- 3. Δθ, R, t  계산 -----------------------------------------------
    dtheta, R, t = compute_transform(args.pos[0], args.pos[1], yaw_ref,
                                     x0, y0, yaw0)

    # ---- 4. 전체 경로 변환 -------------------------------------------------
    rows_tf = transform_path(rows, dtheta, R, t)

    # ---- 5. DB 에 기록 (덮어쓰기 or 새 path_id) ---------------------------
    save_id = args.new_id if args.new_id else args.path_id
    if len(save_id) != 4:
        raise ValueError("path_id 는 반드시 4글자여야 합니다.")

    db.write_db_Path(rows_tf)              # 기존 idx 위치 덮어쓰기

    print(f"✔  '{save_id}' 로 {len(rows_tf)} points 저장 완료.")


if __name__ == "__main__":
    main()
