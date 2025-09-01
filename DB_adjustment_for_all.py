'''
용도
1. global_path 중 일부를 떼어서 local db 복사 제작
2. globla path table의 속도 입력 (missin별 속도 설정 값에 따라)
3. globla path의 speed 변화 부분 삼차 보간

활용에 따라 바꿔야 할 부분 
1. state 클래스 Enum 수정
2. DBExtractor의 global DB와 Local DB 이름수정
3. speed table
'''


import sqlite3
import os
from enum import Enum
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import shutil, pathlib
# ── 파일 맨 위쪽 import 추가
import shutil, datetime


class State(Enum):    
#kcity 본선 대회용 (final - 1012)
    '''    A1A2 = "driving_a"  #13
    A2A3 = "pickup_b"  #9
    A3A4 = "curve_c"  #8
    A4A5 = "curve_d"  #8
    A5A6 = "obstacle_e"  #6
    A6A7 = "curve_f"  #8
    A7A8 = "stop_line_a"  #8
    A8A9 = "stop_line_b"  #8
    A9A10 = "curve_h"  #8
    A10A11 = "traffic_light_i"  #8
    A11A12 = "curve_j"  #8
    A12A13 = "traffic_light_k"  #8
    A13A14 = "driving_l"  #15
    A14A15 = "obstacle_m"  #6
    A15A16 = "curve_n"  #8
    A16A17 = "traffic_light_o"  #8
    A17A18 = "driving_p"  #10
    A18A19 = "delivery_q"  #7 #delivery
    A19A20 = "driving_r"  #8
    A20A21 = "traffic_light_s"  #8
    A21A22 = "driving_t"  #10
    A22A23 = "traffic_light_u"  #8
    A23A24 = "curve_v"  #10
    A24A25 = "driving_w"  #15
    A25A26 = "curve_x"  #11
    A26A27 = "stop_line_c"  #8
    A27A28 = "curve_y"  #8
    A28A29 = "driving_z"  #13
    A29A30 = "traffic_light_A"  #8
    A30A31 = "driving_B"  #16
    A31A32 = "traffic_light_C"  #8
    A32A33 = "driving_D"  #15
    A33A34 = "parking_E"  #6
    A34A35 = "driving_E"  #15'''
    
    """
    ########### 분수대 ##############
    A1A2 = "driving_a"  #13
    A2A3 = "pickup_a"  #9
    A3A4 = "curve_a"  #8
    A4A5 = "driving_b"  #8
    A5A6 = "obstacle_a"  #6
    A6A7 = "driving_c"  #8
    A7A8 = "curve_b"  #8
    A8A9 = "driving_d"  #8
    A9A10 = "delivery_a"  #8
    A10A11 = "curve_c"  #8
    A11A12 = "parking_a"
    A12A13 = "driving_f" 
    ################################
    """
    """         ############### YS 0802 ###########################
    A1A2 = "driving_A" # old(15) new(20)
    A2A3 = "parking_B" # 사선 주차 old(5)
    A3A4 = "curve_C" # old(8) new(11)
    A4A5 = "driving_D" # old(15) new(20)
    A5A6 = "slow_E" # 방지턱 old(12) new(12)
    A6A7 = "curve_F" # old(8) new(11)
    A7A8 = "driving_G" # old(12) new(20)
    #B1B2 = "uturn_H" # old(7) 
    A8A9 = "driving_I" # old(12) new(15)
    A9A10 = "curve_I" # old(12) new(15)
    A10A11 = "obstacle_J" # old(5) new(8)
    ###################  YS ########################### """

    # ############### BS 0802 ###########################
    # A1A2 = "driving_a"       # st(13) mpc(20) -- 40까지?
    # A2A3 = "pickup_b"        # st(9)
    # A3A4 = "driving_c"       # st(13) mpc(20)
    # A4A5 = "traffic_light_d" # st(8) mpc(8)
    # A5A6 = "driving_e"       # st(13) mpc(20)
    # A6A7 = "traffic_light_f" # st(8) mpc(8)
    # A7A8 = "driving_A"       # st(13) mpc(20)
    # A8A9 = "obstacle_b"      # 대형 장애물 # st(6) mpc(6)
    # A9A10 = "curve_h"        # 차선 변경 # st(8) mpc(10)
    # A10A11 = "traffic_light_j" # st(8) mpc(8)
    # A11A12 = "driving_k"     # st(13) mpc(20)
    # A12A13 = "stop_line_l"   # st(10) mpc(10)
    # A13A14 = "curve_m"       # st(8) mpc(10)
    # A14A15 = "stop_line_n"   # st(10) mpc(10)
    # A15A16 = "curve_o"       # st(8) mpc(10)
    # A16A17 = "driving_p"     # st(13) mpc(20)
    # A17A18 = "traffic_light_p" # st(8) mpc(8)

    # # === 여기서부터 1칸씩 뒤로 밀림 ===
    # A18A19 = "driving_q"     # st(13) mpc(20)  ← 새로 추가
    # A19A20 = "curve_r"       # st(13) mpc(20)
    # A20A21 = "delivery_s"    # st(10) mpc(10)
    # A21A22 = "curve_t"       # st(10) mpc(15)
    # A22A23 = "traffic_light_u" # st(8) mpc(8)
    # A23A24 = "driving_v"     # st(10) mpc(15)
    # A24A25 = "traffic_light_w" # st(8) mpc(8)
    # A25A26 = "driving_x"     # st(10) mpc(15)
    # A26A27 = "curve_c"       # st(8) mpc(10)
    # A27A28 = "obstacle_y"    # 소형 # st(8) mpc(8)
    # A28A29 = "curve_z"       # st(8) mpc(10)
    # A29A30 = "driving_C"     # st(13) mpc(20)
    # A30A31 = "parking_A"     # st(5) mpc(5)
    # A31A32 = "driving_B"     # st(13) mpc(20)
    # ###################  BS ###########################


    # (20kph)
    A1A2 = "stanley_A"
    A2A3 = "parking_B"
    A3A4 = "driving_8_C"
    A4A5 = "driving_10_D"
    A5A6 = "stanley_E"
    A6A7 = "driving_H"
    A7A8 = "obstacle_I"
    # A1A2=	'driving_a'
    # A2A3=	"stanley_b"
    # A3A4=	'driving_c'
    # A4A5=	'stanley_d'
    # A5A6=	'driving_e'
    # A6A7=	'stanley_k'



class DBExtractor:
    def __init__(self, path_id, controller="stanley",
                 global_db_dir=os.path.expanduser("~/db_file"),
                 global_db_name="kcity_ys_0831_v_origin.db"):
        self.controller = controller.lower()
        print("Resolved DB path:", os.path.join(os.path.expanduser("~/acca/db_file"),
                                        "bunsudae_0830_v_origin.db"))


        # ① 원본 DB 경로
        # src_path = os.path.join(global_db_dir, global_db_name)

        # # ② stanley → 원본 그대로 / mpc → 사본 만들어 사용
        # if self.controller == "mpc":
        #     base, ext = os.path.splitext(global_db_name)
        #     dst_name  = f"{base}_adjustment{ext}"
        #     dst_path  = os.path.join(global_db_dir, dst_name)

        #     # 이미 있으면 덮어쓰기
        #     if os.path.exists(dst_path):
        #         os.remove(dst_path)
        #     shutil.copy2(src_path, dst_path)          # ★ 사본 생성
        #     self.global_db_path = dst_path
        # else:
        #     self.global_db_path = src_path
        self.global_db_path = os.path.join(global_db_dir, global_db_name)
        # ③ DB 연결
        self.global_conn = sqlite3.connect(self.global_db_path)
        self.global_cur  = self.global_conn.cursor()



        
                # mission 키워드별 속도 정의
        # NEW
        """ self.speed_table = {

            'driving': 25,
            'curve': 13,
            'pickup': 8,
            'delivery': 8,
            'parking': 8,
            'traffic_light': 8,
            'stop_line': 8,
            'obstacle': 8,
        }"""
        
        
        #OLD 

        self.speed_table = {
        "driving": 15,
        "curve": 8,
        "slow_8": 8,
        "slow_10" : 10,
        "parking": 8,
        "obstacle": 7,
        'pickup': 8,
        'delivery': 8,
        'driving_10' : 10,
        'driving_8' : 8,
        "stanley" : 20

         }


        self.checked_global_db = False
        self.checked_local_db = False

        self.global_db_dir = global_db_dir
        self.path_id = path_id
        self.global_db_path = os.path.join(global_db_dir, global_db_name)
        

        '''# 1) 글로벌 DB 존재 확인
        if not os.path.isfile(self.global_db_path):
            raise FileNotFoundError(f"Global DB not found at {self.global_db_path}")

        # 2) 로컬 DB가 이미 존재할 경우, 삭제 또는 재사용 선택
        if os.path.isfile(self.local_db_path):
            answer = input(f"Local DB found at {self.local_db_path}. Delete and recreate? (Y/N): ")
            if answer.lower() == 'y':
                os.remove(self.local_db_path)
                print(f"Deleted existing local DB at {self.local_db_path}")
            else:
                print("Using existing local DB")'''

        # 3) DB 연결
        self.global_conn = sqlite3.connect(self.global_db_path)
        self.global_cur = self.global_conn.cursor()
        




    def check_global_db(self):
        if self.checked_global_db == False:
            # 1) 글로벌 DB 존재 확인
            if not os.path.isfile(self.global_db_path):
                raise FileNotFoundError(f"Global DB not found at {self.global_db_path}")
        self.checked_global_db = True

        
    def check_local_db(self,local_db_path):
        if self.checked_local_db == False:
            # 2) 로컬 DB가 이미 존재할 경우, 삭제 또는 재사용 선택
            if os.path.isfile(local_db_path):
                answer = input(f"Local DB found at {local_db_path}. Delete and recreate? (Y/N): ")
                if answer.lower() == 'y':
                    os.remove(local_db_path)
                    print(f"Deleted existing local DB at {local_db_path}")
                    #로컬 DB에 테이블 생성
                    self._create_tables(local_db_path)
                else:
                    print("Using existing local DB")
                    conn = sqlite3.connect(local_db_path)
                    cur = conn.cursor()
                    cur.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='Node'")
                    node_exists = cur.fetchone()
                    cur.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='Path'")
                    path_exists = cur.fetchone()
                    conn.close()

                    if not node_exists or not path_exists:
                        print(f"Local DB at {local_db_path} is missing required tables. Creating tables.")
                        self._create_tables(os.path.dirname(local_db_path), os.path.basename(local_db_path))
                    else:
                        print("Using existing local DB with tables.")
        self.checked_local_db = True



    def _create_tables(self, local_db_dir_or_name, local_db_name=None):
        """
        Node 테이블의 mission 값을 기반으로 Path.speed와 Node.mission을 일괄 설정합니다.

        • 인자가 하나만 주어지면
          - local_db_dir_or_name을 DB 파일명으로 보고
          - 디렉터리는 기본값 '~/acca/db_file' 사용
        • 인자가 두 개 주어지면
          - 첫번째를 디렉터리, 두번째를 파일명으로 사용
        """
        # 디렉터리/파일명 분기
        if local_db_name is None:
            db_dir  = self.global_db_dir
            db_name = local_db_dir_or_name
            local_db_path = os.path.join(db_dir, db_name)
        else:
            db_dir  = local_db_dir_or_name
            db_name = local_db_name
            local_db_path = os.path.join(db_dir, db_name)
        
        self.check_local_db(local_db_path)
        local_conn = sqlite3.connect(local_db_path)
        local_cur = local_conn.cursor()

        """로컬 DB에 Node와 Path 테이블을 생성합니다."""
        local_cur.execute("""
            CREATE TABLE IF NOT EXISTS Node(
                Start_point CHAR(4),
                End_point   CHAR(4),
                path_id     CHAR(4) PRIMARY KEY,
                mission     CHAR(10)
            );
        """)
        local_cur.execute("""
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
        """)
        local_conn.commit()
        local_conn.close()


    def set_speed_by_mission_in_local_db(self, local_db_dir_or_name, local_db_name=None):
        """
        Node 테이블의 mission 값을 기반으로 Path.speed와 Node.mission을 일괄 설정합니다.

        • 인자가 하나만 주어지면
          - local_db_dir_or_name을 DB 파일명으로 보고
          - 디렉터리는 기본값 '~/acca/db_file' 사용
        • 인자가 두 개 주어지면
          - 첫번째를 디렉터리, 두번째를 파일명으로 사용
        """
        # 디렉터리/파일명 분기
        if local_db_name is None:
            db_dir  = self.global_db_dir
            db_name = local_db_dir_or_name
            local_db_path = os.path.join(db_dir, db_name)
        else:
            db_dir  = local_db_dir_or_name
            db_name = local_db_name
            local_db_path = os.path.join(db_dir, db_name)
        
        local_db_path = os.path.join(db_dir, db_name)

        self.check_local_db(local_db_path)
        local_conn = sqlite3.connect(local_db_path)
        local_cur = local_conn.cursor()

        # Node 테이블에서 path_id, mission(기존에 채워둔 값) 조회
        local_cur.execute("SELECT path_id, mission FROM Node")
        node_rows = local_cur.fetchall()

        for path_id, _ in node_rows:
            try:
                # 1) Enum에서 해당 path_id 멤버 가져오기
                state = State[path_id]          # e.g. State.A1A2
                mission_base = state.value[:-2]  # 'driving_k' → 'driving'
                                                 # e.g. "driving_a" -> "driving"

                # 2) speed_table에서 속도 꺼내기
                assigned_speed = self.speed_table[mission_base]

                # 3) Path.speed 업데이트
                local_cur.execute(
                    "UPDATE Path SET speed = ? WHERE path_id = ?",
                    (assigned_speed, path_id)
                )

                # 4) Node.mission에도 base 미션명 기록
                local_cur.execute(
                    "UPDATE Node SET mission = ? WHERE path_id = ?",
                    (mission_base, path_id)
                )

                print(f"{path_id}: mission='{mission_base}', speed={assigned_speed}")
            except KeyError:
                # Enum에 정의되지 않은 path_id는 건너뛰기
                print(f"Skipped {path_id}: no State enum")

        local_conn.commit()
        local_conn.close()

    def set_speed_by_mission_in_global_db(self):
        """Node 테이블의 mission 값을 기반으로 Path.speed와 Node.mission을 일괄 설정"""


        # Node 테이블에서 path_id, mission(기존에 채워둔 값) 조회
        self.global_cur.execute("SELECT path_id, mission FROM Node")
        node_rows = self.global_cur.fetchall()

        for path_id, _ in node_rows:
            try:
                # 1) Enum에서 해당 path_id 멤버 가져오기
                state = State[path_id]          # e.g. State.A1A2
                mission_base = state.value[:-2] # e.g. "driving_a" -> "driving"

                # 2) speed_table에서 속도 꺼내기
                assigned_speed = self.speed_table[mission_base]

                # 3) Path.speed 업데이트
                self.global_cur.execute(
                    "UPDATE Path SET speed = ? WHERE path_id = ?",
                    (assigned_speed, path_id)
                )

                # 4) Node.mission에도 base 미션명 기록
                self.global_cur.execute(
                    "UPDATE Node SET mission = ? WHERE path_id = ?",
                    (mission_base, path_id)
                )

                print(f"{path_id}: mission='{mission_base}', speed={assigned_speed}")
            except KeyError:
                # Enum에 정의되지 않은 path_id는 건너뛰기
                print(f"Skipped {path_id}: no State enum")

        self.global_conn.commit()


    
    def extract_tables(self,local_db_dir_or_name, local_db_name=None):
        """
        Node 테이블의 mission 값을 기반으로 Path.speed와 Node.mission을 일괄 설정합니다.

        • 인자가 하나만 주어지면
          - local_db_dir_or_name을 DB 파일명으로 보고
          - 디렉터리는 기본값 '~/acca/db_file' 사용
        • 인자가 두 개 주어지면
          - 첫번째를 디렉터리, 두번째를 파일명으로 사용
        """
        # 디렉터리/파일명 분기
        if local_db_name is None:
            db_dir  = self.global_db_dir
            db_name = local_db_dir_or_name
            local_db_path = os.path.join(db_dir, db_name)
        else:
            db_dir  = local_db_dir_or_name
            db_name = local_db_name
            local_db_path = os.path.join(db_dir, db_name)
        
        local_db_path = os.path.join(db_dir, db_name)

        self.check_local_db(local_db_path)
        local_conn = sqlite3.connect(local_db_path)
        local_cur = local_conn.cursor()
        """
        글로벌 DB에서
        1) Node 테이블 전체,
        2) 지정한 self.path_id의 Path 테이블 행들
        을 로컬 DB에 복사합니다.
        """
        # 1) Node 전체 복사
        self.global_cur.execute("SELECT Start_point, End_point, path_id, mission FROM Node")
        node_rows = self.global_cur.fetchall()
        for start_pt, end_pt, pid, mission in node_rows:
            local_cur.execute(
                "INSERT OR IGNORE INTO Node(Start_point, End_point, path_id, mission) VALUES (?, ?, ?, ?)",
                (start_pt, end_pt, pid, mission)
            )

        # 2) 특정 path_id에 해당하는 Path 복사
        self.global_cur.execute(
            "SELECT path_id, idx, x, y, yaw, speed FROM Path WHERE path_id = ? ORDER BY idx",
            (self.path_id,)
        )
        path_rows = self.global_cur.fetchall()
        for pid, idx, x, y, yaw, speed in path_rows:
            local_cur.execute(
                "INSERT OR IGNORE INTO Path(path_id, idx, x, y, yaw, speed) VALUES (?, ?, ?, ?, ?, ?)",
                (pid, idx, x, y, yaw, speed)
            )

        # 최종 커밋
        local_conn.commit()
        local_conn.close()


    def speed_in_node(self):
        """
        (예시) 글로벌 DB의 Path 테이블을
        path_id별로 묶어 평균 speed를 계산해서 출력합니다.
        필요하신 용도에 맞게 수정하세요.
        """
        # path_id별 평균 속도 계산
        self.global_cur.execute(
            "SELECT path_id, AVG(speed) FROM Path GROUP BY path_id"
        )
        for pid, avg_speed in self.global_cur.fetchall():
            print(f"구간 {pid} 의 평균 속도: {avg_speed:.2f}")


        # ────────────────────────────────────────────────────────────────


    
    def _rebuild_node_from_path(self):
        cur = self.global_cur

        # 0) Path-기반 순서 목록 확보
        cur.execute("""
            SELECT path_id, MIN(idx) AS first_idx
            FROM Path
        GROUP BY path_id
        ORDER BY first_idx
        """)
        ordered = [row[0] for row in cur.fetchall()]

        # 1) Node 싹 비우기
        cur.execute("DELETE FROM Node")

        # 2) Python 안에서 새 Node 행 만들기
        for pid in ordered:
            # A13A14 → 13, 14
            k1, k2 = map(int, pid[1:].split('A'))
            start  = f"A{k1}"
            end    = f"A{k2}"

            # Enum → mission 파싱
            try:
                state_val = State[pid].value
                base      = state_val.split('_')[0]
            except KeyError:
                base      = 'driving'          # 병합 규칙: curve→driving

            if base in ('driving', 'curve'):   # MPC 규칙
                mission = 'driving'
            else:
                mission = base

            cur.execute(
                "INSERT INTO Node(Start_point, End_point, path_id, mission) "
                "VALUES (?,?,?,?)",
                (start, end, pid, mission)
            )


    # 1️⃣ Node 자체를 A번호 오름차순으로 정렬
    def _ordered_path_ids(self):
        """Node.path_id를 A번호 기준 오름차순으로 정렬해 반환"""
        self.global_cur.execute("""
            SELECT path_id,
                CAST(
                    substr(path_id, 2,
                            instr(substr(path_id, 2), 'A') - 1)
                    AS INT) AS k        -- A13A14 → 13
            FROM Node
        ORDER BY k
        """)
        return [pid for pid, _ in self.global_cur.fetchall()]
    






    # 2️⃣ 병합 함수 - Node 기준 그룹핑 + mission 압축
    def merge_curve_drive_segments(self):
        cur = self.global_cur
        ordered = self._ordered_path_ids()
        if not ordered: 
            print("⚠️ 병합 대상 없음"); return

        # ── ① 그룹 만들기: driving/curve 연속 묶음
        groups = []
        i = 0
        while i < len(ordered):
            g = [ordered[i]]
            base_i = self._mission_base(ordered[i])
            if base_i in ("driving", "curve"):
                j = i + 1
                while j < len(ordered) and self._mission_base(ordered[j]) in ("driving","curve"):
                    g.append(ordered[j]);  j += 1
                i = j
            else:
                i += 1
            groups.append(g)

        # ── ② 새 path_id (A1A2, A2A3 …)
        new_ids = [f"A{k}A{k+1}" for k in range(1, len(groups)+1)]

        # ── ③ 병합·재명명
        cur.execute("BEGIN")
        for g, new_pid in zip(groups, new_ids):
            master = g[0]

            # Path → TMP_*
            for pid in g:
                cur.execute("UPDATE Path SET path_id=? WHERE path_id=?",
                            (f"TMP_{pid}", pid))

            # 서브 Node 삭제
            for sub in g[1:]:
                cur.execute("DELETE FROM Node WHERE path_id=?", (sub,))

            # mission 결정
            base_list = [self._mission_base(p) for p in g]
            if all(b in ("driving","curve") for b in base_list):
                mission = 'driving'              # 병합 규칙
            else:
                mission = self._mission_base(master)  # pickup/obstacle 등

            # master Node mission 갱신
            cur.execute("UPDATE Node SET mission=? WHERE path_id=?", (mission, master))

            # TMP_* → 최종 new_pid
            for pid in g:
                cur.execute("UPDATE Path SET path_id=? WHERE path_id=?",
                            (new_pid, f"TMP_{pid}"))
            cur.execute("UPDATE Node SET path_id=? WHERE path_id=?", (new_pid, master))

            # Start / End 갱신
            k1, k2 = map(int, new_pid[1:].split('A'))
            cur.execute("UPDATE Node SET Start_point=?, End_point=? WHERE path_id=?",
                        (f"A{k1}", f"A{k2}", new_pid))
            
                # 병합 루프 끝난 뒤, Start/End 업데이트
        # merge_curve_drive_segments() 맨 끝에 (루프 밖):
        cur.execute("""
            UPDATE Node
            SET Start_point = 'A' ||
                CAST(substr(path_id, 2,
                            instr(substr(path_id,2),'A')-1) AS TEXT),
                End_point   = 'A' ||
                CAST(substr(path_id,
                            instr(substr(path_id,2),'A')+1) AS TEXT)
        """)
        # cur.commit()  →  self.global_conn.commit()
        self.global_conn.commit()
        print("✅ Node 기반 병합 완료 – mission 압축 OK")






    


    # 추가 유틸 ──────────────────────────────────────────────
    def _mission_base(self, pid: str) -> str:
        """path_id → 'driving' / 'curve' / …  (enum 없으면 'other')"""
        try:
            return State[pid].value.split('_')[0]
        except KeyError:
            return 'other'
####################################################################################################    이 안에 있는 모든 min_w이랑, max_w를 모두 조정할 것
    def _blend_window(self, dv, min_w=40, max_w=80) -> int:
        """Δv에 따라 전체 창 길이 L 산출(선형)"""
        return int(round(min_w + (max_w - min_w) * min(abs(dv*1.5), 10) / 10))

    # 핵심 함수 ─────────────────────────────────────────────
    def smooth_speed_transitions(self, min_w=90, max_w=120, drive_bias=0.5):
#####################################################################################################        
        """
        Δv로 L 결정 →  (driving↔X) 이면 L*drive_bias 를 driving 쪽,
        나머지를 상대쪽으로 배분.
        drive_bias=0.8  → 80 %를 driving 쪽에 “몰빵”.
        """
        cur = self.global_cur
        cur.execute("SELECT idx, path_id, speed FROM Path ORDER BY idx")
        rows = cur.fetchall()
        if not rows:
            print("⚠️  Path 테이블이 비어 있습니다.");  return

        idxs, pids, speeds = zip(*rows)
        total = len(idxs)
        trans = [i for i in range(1, total) if pids[i] != pids[i-1]]

        for t in trans:
            v1, v2   = speeds[t-1], speeds[t]
            pid1, pid2 = pids[t-1], pids[t]
            m1,  m2  = self._mission_base(pid1), self._mission_base(pid2)
            dv       = abs(v2 - v1)
            L        = self._blend_window(dv, min_w, max_w)

            # ① 기본 대칭값
            before = after = L // 2
            # ② driving ↔ other 이면 비대칭 조정
            if m1 == 'driving' and m2 != 'driving':
                before = int(L * drive_bias)
                after  = L - before
            elif m2 == 'driving' and m1 != 'driving':
                after  = int(L * drive_bias)
                before = L - after

            start = max(0, t - before)
            end   = min(total, t + after)
            if end - start < 2:  continue

            for j in range(start, end):
                τ = (j - start) / (end - start - 1)
                s = 3*τ*τ - 2*τ*τ*τ          # Hermite S-curve
                new_v = v1 + (v2 - v1) * s
                cur.execute("UPDATE Path SET speed=? WHERE idx=?", (float(new_v), idxs[j]))

            print(f"{pid1}->{pid2}  Δv={dv:.1f}  win={before}/{after}")

        self.global_conn.commit()
        print("✅ 비대칭 스무싱 완료")



    
    def plot_speed(self):
        """
        idx-speed 시계열 + 구간별 mission 라벨 (Node.mission 사용)
        """
        cur = self.global_cur
        cur.execute("""
            SELECT p.idx, p.path_id, p.speed, n.mission
            FROM Path p
            JOIN Node n USING(path_id)
        ORDER BY p.idx
        """)
        rows = cur.fetchall()
        if not rows:
            print("⚠️  Path 테이블이 비어 있습니다."); return

        idxs, pids, speeds, missions = zip(*rows)

        # ① path_id 블록 경계 찾기
        blocks, start, cur_pid = [], 0, pids[0]
        for i, pid in enumerate(pids[1:], start=1):
            if pid != cur_pid:
                blocks.append((cur_pid, missions[i-1], start, i))  # [start, i)
                start, cur_pid = i, pid
        blocks.append((cur_pid, missions[-1], start, len(pids)))

        # ② 플롯
        plt.figure()
        plt.plot(idxs, speeds, linewidth=1)
        plt.xlabel("idx");  plt.ylabel("speed");  plt.title("Speed vs. idx")

        # ③ 라벨 (mission) 표시
        y_min, y_max = min(speeds), max(speeds)
        y_off = (y_max - y_min) * 0.05
        for pid, ms, s, e in blocks:
            mid = (s + e) // 2
            plt.text(idxs[mid], speeds[mid] + y_off, ms, ha="center", va="bottom", fontsize=8)

        plt.tight_layout();  plt.show()


    


    def close(self):
        """DB 연결을 종료합니다."""
        self.global_conn.close()
        
        
        
            


def main(): # for local path
    """
    extractor = DBExtractor(local_db_name='BS_final_A4A5.db', path_id='A4A5')
    extractor.extract()
    extractor.close()
    """
##########################################################################################                 

    extractor = DBExtractor(path_id='A4A5') # 만들고자 하는 local path 부분
    
    #extractor.extract()
    #extractor.close()
    
    #원하는 부분을 local path db로 복사
    #extractor.extract_tables('parking_local.db')
##########################################################################################                 
    
    # 설정한 속도에 맞춰 전체 속도 설정하기
    extractor.set_speed_by_mission_in_global_db()
    
    # 전체 구간 속도 3차 보간
    extractor.smooth_speed_transitions()          # min_w=20, max_w=60

    if extractor.controller == "mpc":
        extractor.merge_curve_drive_segments()

    # 전체 구간 속도 plot
    extractor.plot_speed()

    extractor.close()

if __name__ == "__main__":
    
    main()
