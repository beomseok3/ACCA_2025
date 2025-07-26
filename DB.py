import numpy as np
np.float = np.float64
import sqlite3
import os

from std_msgs.msg import Header
from nav_msgs.msg import Path
from tf_transformations import *
from geometry_msgs.msg import PoseStamped



class DB():
    def __init__(self, db_name):
        # 1) DB 디렉토리 보장
        db_dir = "/home/acca/db_file"
        os.makedirs(db_dir, exist_ok=True)

        # 2) DB 파일 경로 설정
        self.db_path = os.path.join(db_dir, db_name)

        # 3) SQLite 연결 & 커서 생성
        self.__conn = sqlite3.connect(self.db_path)
        self.__cur  = self.__conn.cursor()

        # 4) 테이블이 없으면 만들어 줌
        self.makeTable()

        # 5) 기본 ID·인덱스 초기화
        self.id = "A1A2"
        self.i  = 0

    def makeTable(self):
        # Node 테이블 생성
        self.__cur.execute(
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
        self.__cur.execute(
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
        self.__conn.commit()

    
    def find_idx(self, x, y, table):
        self.__cur.execute(f"SELECT idx, x, y FROM {table}")
        rows = self.__cur.fetchall()
        min_err  = float('inf')
        idx = 0
        for row in rows:
            x_val, y_val = row[1], row[2]
            err = (x_val - x)**2 + (y_val - y)**2
            if err < min_err:
                min_err, idx = err, row[0]
        return idx
    
    def get_max_idx(self, table_name: str) -> int:
        
        self.__cur.execute(f"SELECT MAX(idx) FROM {table_name}")
        result = self.__cur.fetchone()[0]
        return result if result is not None else 0

    def read_db_n(self, table, *cols):
        cols_str = ', '.join(cols)
        self.__cur.execute(f"SELECT {cols_str} FROM {table}")
        return self.__cur.fetchall()

    def write_db_Path(self, data):
        for i, (x, y, yaw) in enumerate(data):
            self.__cur.execute(
                """
                INSERT INTO Path (path_id, idx, x, y, yaw)
                VALUES (?, ?, ?, ?, ?)
                ON CONFLICT(idx) DO UPDATE SET
                  path_id=excluded.path_id,
                  x=excluded.x,
                  y=excluded.y,
                  yaw=excluded.yaw
                """,
                (self.id, i, x, y, yaw),
            )
        self.__conn.commit()

    def write_db_Path_con(self, data):
        for x, y, yaw in data:
            self.__cur.execute(
                "INSERT INTO Path (path_id, idx, x, y, yaw) VALUES (?, ?, ?, ?, ?)",
                (self.id, self.i, x, y, yaw),
            )
            self.i += 1
        self.__conn.commit()

    def write_db_Node(self, data):
        for start_pt, end_pt, pid in data:
            self.__cur.execute(
                "INSERT INTO Node (Start_point, End_point, path_id) VALUES (?, ?, ?)",
                (start_pt, end_pt, pid),
            )
        self.__conn.commit()

    def query_from_id_to_path(self, pid):
        self.__cur.execute("SELECT x, y, yaw FROM Path WHERE path_id == ?", (pid,))
        rows = self.__cur.fetchall()
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for x, y, yaw in rows:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, 0.0
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
            path.poses.append(pose)
        return path

    def query_from_id(self, pid):
        self.__cur.execute("SELECT x, y, yaw, speed FROM Path WHERE path_id == ?", (pid,))
        rows = self.__cur.fetchall()
        cx, cy, cyaw, cv = [], [], [], []
        for x, y, yaw, v in rows:
            cx.append(x); cy.append(y); cyaw.append(yaw); cv.append(v)
        return cx, cy, cyaw, cv

    def read_db_from_id_to_mission(self, pid):
        self.__cur.execute("SELECT mission FROM Node WHERE path_id == ?", (pid,))
        return self.__cur.fetchone()

    def deletePath(self, pid):
        self.__cur.execute("DELETE FROM Node WHERE path_id = ?;", (pid,))
        self.__conn.commit()

    def __del__(self):
        if hasattr(self, '__conn') and self.__conn:
            self.__conn.close()
    
    
    def splitPath(self, str_idx, end_idx, id):
        '''Split path_id from str to end
        
        Args:
            str_idx (int): path_id start idx
            end_idx (int): path_id end idx
            id (str): path_id
        '''

        query ="""UPDATE Path
                SET path_id = ?
                WHERE idx >= ? AND idx <= ?
                """
        self.__cur.execute(query,(id, str_idx, end_idx))
        self.__conn.commit()
