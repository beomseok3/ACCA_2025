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
    A11A12 = "driving_e"
    A12A13 = "parking_a"
    A13A14 = "driving_f"


class DBExtractor():
    def __init__(self, path_id,
##########################################################################################                 
                 global_db_dir=os.path.expanduser('~/acca/db_file'),
                 global_db_name='bunsudae_v1.db'):
##########################################################################################                 
        
                # mission 키워드별 속도 정의
        # NEW
        self.speed_table = {
            'driving': 25,
            'curve': 13,
            'pickup': 8,
            'delivery': 8,
            'parking': 8,
            'traffic_light': 8,
            'stop_line': 8,
            'obstacle': 8,
        }
        #OLD
        """ self.speed_table = {
            'driving': 13,
            'curve': 8,
            'pickup': 8,
            'delivery': 8,
            'parking': 8,
            'traffic_light': 8,
            'stop_line': 8,
            'obstacle': 8,
        } """


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
                mission_base = state.value[:-2] # e.g. "driving_a" -> "driving"

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

    


    def smooth_speed_transitions(self, before: int = 30, after: int = 30):
        """
        path_id 전환 지점을 찾은 뒤,
        전(before)·후(after) 포인트씩에 대해
        3차 Hermite 스무스스텝으로 speed를 부드럽게 이어 붙입니다.
        """
        cur = self.global_cur
        cur.execute("SELECT idx, path_id, speed FROM Path ORDER BY idx")
        rows = cur.fetchall()
        if not rows:
            print("⚠️ Path 테이블이 비어 있습니다.")
            return

        idxs, pids, speeds = zip(*rows)
        total = len(idxs)

        # 1) path_id 전환 위치 찾기
        transitions = [i for i in range(1, total) if pids[i] != pids[i-1]]
        print(f"Detected {len(transitions)} transitions → {transitions}")

        # 2) 각 변화점마다 스무스스텝 적용
        for t in transitions:
            v1 = speeds[t-1]  # 이전 구간 속도
            v2 = speeds[t]    # 다음 구간 속도

            start = max(0, t - before)
            end   = min(total, t + after)
            L = end - start    # 전체 윈도우 길이
            if L < 2:
                continue

            for j in range(start, end):
                # 0 ≤ r ≤ L-1
                r = j - t + before
                # 0 ≤ τ ≤ 1
                tau = r / (L - 1)
                # Hermite 스무스스텝
                new_spd = v1 + (v2 - v1) * (3*tau*tau - 2*tau*tau*tau)
                cur.execute(
                    "UPDATE Path SET speed = ? WHERE idx = ?",
                    (float(new_spd), idxs[j])
                )
            print(f"– Smoothed transition at row {t}: {v1}→{v2} over {L} points")

        # 3) 커밋
        self.global_conn.commit()
        print("✅ 모든 스무스 작업 커밋 완료")

    
    def plot_speed(self):
        """
        idx를 x축, speed를 y축으로 하는 시계열 플롯을 그리고,
        각 path_id 구간 블록의 중앙에 해당 mission 이름을 표시합니다.
        """
        # 1) 전체 데이터 조회
        self.global_cur.execute("SELECT idx, path_id, speed FROM Path ORDER BY idx")
        rows = self.global_cur.fetchall()
        if not rows:
            print("⚠️ Path 테이블이 비어 있습니다.")
            return

        idxs, pids, speeds = zip(*rows)

        # 2) 연속된 path_id 블록 탐색
        blocks = []
        start = 0
        cur_pid = pids[0]
        for i, pid in enumerate(pids[1:], start=1):
            if pid != cur_pid:
                blocks.append((cur_pid, start, i))  # [start, i) 구간
                start = i
                cur_pid = pid
        blocks.append((cur_pid, start, len(pids)))  # 마지막 블록

        # 3) 플롯 그리기
        plt.figure()
        plt.plot(idxs, speeds, linewidth=1)
        plt.xlabel('idx')
        plt.ylabel('speed')
        plt.title('Speed vs. idx')

        # 4) 각 블록 중앙에 annotation 추가
        y_min, y_max = min(speeds), max(speeds)
        y_offset = (y_max - y_min) * 0.05  # 전체 범위의 5% 위쪽

        for pid, bstart, bend in blocks:
            # 블록 중앙 인덱스 계산
            mid = (bstart + bend) // 2
            mid_idx = idxs[mid]
            mid_spd = speeds[mid]

            # State enum에서 mission 이름 추출
            try:
                mission_full = State[pid].value    # ex. "driving_a"
                mission_base = mission_full.split('_')[0]  # ex. "driving"
            except KeyError:
                mission_base = pid  # enum에 없으면 그냥 path_id 표시

            # 텍스트 표시
            plt.text(
                mid_idx, 
                mid_spd + y_offset, 
                mission_base, 
                ha='center', 
                va='bottom', 
                fontsize=8, 
                rotation=0
            )

        plt.tight_layout()
        plt.show()


    


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
    extractor.smooth_speed_transitions(before=40, after=40)
    # 전체 구간 속도 plot
    extractor.plot_speed()

    extractor.close()

if __name__ == "__main__":
    
    main()
