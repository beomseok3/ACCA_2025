import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    qos_profile_sensor_data,
    qos_profile_system_default,
)
import rclpy.time
from tf2_ros import Buffer, TransformListener
import open3d as o3d
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import PoseWithCovarianceStamped as PoseWithCovarianceStamped2


class Grid(object):
    def __init__(self, x, y, idx_x, idx_y):
        self.x = x
        self.y = y
        self.idx_x = idx_x
        self.idx_y = idx_y
        self.data = []

    def __str__(self):
        return "idx_x: {}, idx_y:{}, datas: {}\n".format(
            self.idx_x, self.idx_y, len(self.data)
        )


class TestMap(Node):
    def __init__(self):
        super().__init__("test_map_node")

        self.map_file = "/home/gjs/autoware_map/school_map/pointcloud_map.pcd"
        self.grid_size = 25  # m

        self.map_publisher = self.create_publisher(
            PointCloud2, "/map", qos_profile=qos_profile_system_default
        )
        self.map = self.readPCD()

        self.buffer = Buffer(node=self, cache_time=rclpy.duration.Duration(seconds=0.1))
        self.tf_listener = TransformListener(
            self.buffer, self, qos=qos_profile_system_default
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/odometry/kalman",
            callback=self.odom_callback,
            qos_profile=qos_profile_system_default,
        )
        # self.pose_subscriber = self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     "/initialpose",
        #     callback=self.pose_callback,
        #     qos_profile=qos_profile_system_default,
        # )

        self.x_grid, self.y_grid, self.grids = self.initializeMap()

        self.current_x_idx = None
        self.current_y_idx = None

        self.timer = self.create_timer(5.0, self.timer_callback)



    def timer_callback(self):
        self.publish_pointcloud2(self.map)

    def odom_callback(self, msg):
        if self.buffer.can_transform(
            "map", "odom", rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1)
        ):
            transform_msg = PoseWithCovarianceStamped2(header=msg.header, pose=msg.pose)
            transformed_msg = self.buffer.transform(
                transform_msg, "map", rclpy.duration.Duration(seconds=0.1)
            )

            x = transformed_msg.pose.pose.position.x
            y = transformed_msg.pose.pose.position.y

            idx_x = np.searchsorted(self.x_grid, x) - 1
            idx_y = np.searchsorted(self.y_grid, y) - 1

            if self.current_x_idx != idx_x or self.current_y_idx != idx_y:
                data = self.get_data(x, y)

                if data.shape[1] != 0:
                    self.publish_pointcloud2(data=data)

                    self.current_x_idx = idx_x
                    self.current_y_idx = idx_y

                else:
                    self.get_logger().warn("Cannot Find Grid")

        else:
            self.get_logger().warn("Cannot lookup transform between map and odom")

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        data = self.get_data(x, y)

        if data.shape[1] != 0:
            self.publish_pointcloud2(data=data)

        else:
            self.get_logger().warn("Cannot Find Grid")

    def readPCD(self):
        try:
            map = o3d.io.read_point_cloud(
                self.map_file,
                format="pcd",
            )

            map_np = np.asarray(map.points)

            temp = np.full((map_np.shape[0], 4), 1.0)

            temp[:, :3] = map_np

        except Exception as ex:
            self.get_logger().warn(str(ex))
            rclpy.shutdown()

        self.get_logger().info("Loading PCD : Finished")

        return temp

    def initializeMap(self):
        min_coords = np.min(self.map, axis=0)
        max_coords = np.max(self.map, axis=0)

        x_start = min_coords[0]
        x_end = max_coords[0]
        y_start = min_coords[1]
        y_end = max_coords[1]

        x_grid = np.arange(
            x_start, x_end + float(self.grid_size), float(self.grid_size)
        )
        y_grid = np.arange(
            y_start, y_end + float(self.grid_size), float(self.grid_size)
        )

        self.get_logger().info(
            "Griding : Find {} * {} grids".format(len(x_grid), len(y_grid))
        )
        self.get_logger().info("Calculating : Grid")

        grids = []

        for i, x in enumerate(x_grid):
            for j, y in enumerate(y_grid):
                gird = Grid(x, y, i, j)
                grids.append(gird)

        grids_dic = {(grid.idx_x, grid.idx_y): grid for grid in grids}

        err_cnt = 0

        for point in self.map:
            idx_x = np.searchsorted(x_grid, point[0]) - 1
            idx_y = np.searchsorted(y_grid, point[1]) - 1

            if (idx_x, idx_y) in grids_dic:
                identified_grid = grids_dic[(idx_x, idx_y)]
                identified_grid.data.append(point)

            else:
                err_cnt += 1

        self.get_logger().info("Gridiing : Finished\tExceptions : {}".format(err_cnt))

        return x_grid, y_grid, grids_dic

    def get_data(self, x, y):
        res = None

        idx_x = np.searchsorted(self.x_grid, x) - 1
        idx_y = np.searchsorted(self.y_grid, y) - 1

        idxs = self.get_possible_idx(idx_x, idx_y)

        for idx in idxs:
            if (idx[0], idx[1]) in self.grids:
                res = (
                    np.array(self.grids[(idx[0], idx[1])].data)
                    if res is None
                    else np.vstack((res, np.array(self.grids[(idx[0], idx[1])].data)))
                )

        return res

    def get_possible_idx(self, idx_x, idx_y):
        res = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                res.append([int(idx_x + i), int(idx_y + j)])

        return res

    def publish_pointcloud2(self, data):
        # for _ in range(3):
        self.get_logger().info("Publish New Map")
        self.map_publisher.publish(self.get_pointcloud2(data))

    def get_pointcloud2(self, rawdata):
        header = Header(frame_id="map", stamp=rclpy.time.Time().to_msg())

        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        fields = [
            PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate(["x", "y", "z", "intensity"])  # xyzrgba
        ]

        data = rawdata.astype(dtype).tobytes()

        msg = PointCloud2(
            header=header,
            height=1,
            width=rawdata.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 4),
            row_step=(itemsize * 4 * rawdata.shape[0]),
            data=data,
        )

        return msg


def main(args=None):
    rclpy.init(args=None)

    node = TestMap()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

