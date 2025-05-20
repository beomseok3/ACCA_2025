import sqlite3
import os
import shutil
import numpy as np

from std_msgs.msg import Header
from nav_msgs.msg import Path
from tf_transformations import *
from geometry_msgs.msg import PoseStamped


class DB:
    """
    module for manage DB

    Note:
        self.db_path를 환경에 맞게 수정할것
    """

    def __init__(self, db_name):

        # file path for read and write
        self.db_path = "/home/ps/db_file" + "/" + db_name

        # check whether file exist
        file_already_exist = os.path.isfile(self.db_path)

        # sqlite3
        self.__conn = sqlite3.connect(self.db_path)
        self.__cur = self.__conn.cursor()

        # path attribute
        self.id = "A1A2"

        if file_already_exist:
            print(f"Database Found At {self.db_path}")

            print("backup")

            # copy ~/origin_file.db  to ~/origin_file
            base, ext = os.path.splitext(self.db_path)
            backup_path = f"{base}_backup{ext}"
            shutil.copy(f"{self.db_path}", f"{backup_path}")
        else:
            print(f"Database Not Found At {self.db_path}")

            print("make table")

            # make default table(Node and Path)
            self.makeTable()

    # write method
    def write_db_Path(self, data, id="A1A2"):
        """Write on Path Table

        Args:
            data (itterable object): [(x = float, y = float, yaw = float, speed = float), (, , , ), ... ]
            id: path_id

        Note:
            idx가 같으면 덮어쓰기 된다
        """

        for i, (x, y, yaw, speed) in enumerate(data):
            self.__cur.execute(
                """
                INSERT INTO Path (path_id, idx, x, y, yaw, speed) VALUES (?, ?, ?, ?, ?, ?)
                ON CONFLICT(idx) DO UPDATE SET path_id = excluded.path_id, 
                x=excluded.x, y=excluded.y, yaw=excluded.yaw, speed=excluded.speed
                """,
                (
                    id,
                    i,
                    x,
                    y,
                    yaw,
                    speed,
                ),
            )
        self.__conn.commit()

    def write_db_Path_insert(self, data, id="A1A2", idx=0):
        """Write on Path Table (insert mode)

        Args:
            data (itterable object): [(x = float, y = float, yaw = float, speed = float), (, , , ), ... ]
            idx (int): insert idx
        """

        if self.idx is None:
            self.idx = idx

        for i, (x, y, yaw, speed) in enumerate(data):
            self.__cur.execute(
                "INSERT INTO Path (path_id, idx, x, y, yaw, speed) VALUES (?, ?, ?, ?, ?, ?)",
                (id, self.idx, x, y, yaw, speed),
            )
            self.idx += 1
        self.__conn.commit()

    def write_db_Node(self, data, mission="driving"):
        """Write on Node Table

        Args:
            data (itterable object): (Str_point, End_point, path_id)
        """
        for (
            str,
            end,
            id,
        ) in data:
            self.__cur.execute(
                "INSERT INTO Node (Start_point, End_point, path_id, mission) VALUES (?, ?, ?, ?)",
                (str, end, id, mission),
            )
        self.__conn.commit()

    def deletePath(self, id):
        """Delete Path table associated with given path_id

        Args:
            id (str): path_id

        Note:
            Node Table과 Path Table 모두에서 삭제
        """

        query_Node = "DELETE FROM Node WHERE path_id = ?;"
        self.__cur.execute(query_Node, (id,))
        self.__conn.commit()

    def splitPath(self, str_idx, end_idx, id):
        """Split path_id from str to end

        Args:
            str_idx (int): path_id start idx
            end_idx (int): path_id end idx
            id (str): path_id
        """

        query = """UPDATE Path
                SET path_id = ?
                WHERE idx >= ? AND idx <= ?
                """
        self.__cur.execute(query, (id, str_idx, end_idx))
        self.__conn.commit()

    def find_idx(self, x, y):
        """Find Closest idx from Path Table

        Args:
            x (float): target x
            y (float): target y

        TODO:
            Marker와 Tkinter 기능 추가
        """
        self.__cur.execute(f"SELECT idx, x, y FROM Path")
        rows = self.__cur.fetchall()
        min_err = 10000
        idx = 0
        for row in rows:
            x_value = row[1]
            y_value = row[2]

            x_err = (x_value - x) ** 2
            y_err = (y_value - y) ** 2
            total_err = x_err + y_err

            if total_err < min_err:
                min_err = total_err
                idx = row[0]
        return idx

    # read method
    def query_from_id(self, id):
        """Query from Path Table

        Args:
            id (str): path_id
        """
        self.__cur.execute(
            "SELECT x, y, yaw, speed FROM Path where path_id == ?", (id,)
        )
        rows = self.__cur.fetchall()

        rows = np.array(rows)

        # return cx, cy, cyaw, cv
        # return rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3]
        return rows

    def read_db_mission(self, id):
        """Read mission from Node Table with path_id

        Args:
            id (str): path_id
        """
        self.__cur.execute("SELECT mission FROM Node where path_id == ", (id,))
        rows = self.__cur.fetchone()
        return rows

    # getter method
    def get_cursor(self):
        return self.__cur

    def makeTable(self):
        # Node Table
        self.__cur.execute(
            "CREATE TABLE Node(Start_point CHAR(4), End_point CHAR(4), path_id CHAR(4) PRIMARY KEY, mission CHAR(10));"
        )
        # Path Table
        self.__cur.execute(
            """
            CREATE TABLE Path(
                path_id CHAR(4), 
                idx INTEGER PRIMARY KEY, 
                x REAL, 
                y REAL, 
                yaw REAL, 
                speed REAL,
                FOREIGN KEY(path_id) REFERENCES Node(path_id) 
                ON DELETE CASCADE 
                ON UPDATE CASCADE
            );
            """
        )
