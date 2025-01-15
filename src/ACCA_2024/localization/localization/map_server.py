import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import qos_profile_system_default
import open3d as o3d
import numpy as np
import argparse


class MapServer(Node):
    def __init__(self, file, topic, frame, hz):
        super().__init__("map_server_node")

        self.file = file
        self.topic = topic
        self.frame = frame
        self.hz = int(hz)

        # Publisher
        self.map_pub = self.create_publisher(
            PointCloud2, self.topic, qos_profile_system_default
        )

        self.map = self.readPCD()

        if self.hz == 0:
            self.publish_pointcloud2()
        else:
            self.create_timer(
                timer_period_sec=(1.0 / float(self.hz)),
                callback=self.publish_pointcloud2,
            )

    # read entire pcd and return asarray(n, 4)
    def readPCD(self):
        try:
            map = o3d.io.read_point_cloud(
                self.file,
                format="pcd",
            )

            map = map.voxel_down_sample(voxel_size=0.5)
            # 9249639
            # 7467642

            print(map)

            map_np = np.asarray(map.points)

            temp = np.full((map_np.shape[0], 4), 1.0)

            temp[:, :3] = map_np

        except Exception as ex:
            self.get_logger().warn(str(ex))
            rclpy.shutdown()

        self.get_logger().info("Loading PCD : Finished")

        return temp

    def publish_pointcloud2(self):
        self.map_pub.publish(self.get_pointcloud2(self.map))

    # Function to transform asarray to PointCloud2
    def get_pointcloud2(self, rawdata):
        header = Header(frame_id=self.frame, stamp=Time().to_msg())

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


def main():
    parser = argparse.ArgumentParser(
        prog="map server",
        description="read pcd file and publish PointCloud2\nSuch packages are required:\tnumpy==1.24.0\tscipy==1.8.0\topen3d==0.18.0",
        epilog="7cmdehdrb@naver.com",
    )

    parser.add_argument("file", type=str, help="PCD file path. REQUIRED")
    parser.add_argument(
        "-t", "--topic", type=str, required=True, help="Published topic. REQUIRED"
    )
    parser.add_argument(
        "-f",
        "--frame",
        type=str,
        required=False,
        help="Published frame. default to map",
        default="map",
    )
    parser.add_argument(
        "-hz",
        "--hz",
        type=int,
        required=False,
        help="Published hz. default to 0(publish once)",
        default=0,
    )
    args = parser.parse_args()

    file = args.file
    topic = args.topic
    frame = args.frame
    hz = args.hz

    rclpy.init(args=None)
    node = MapServer(file, topic, frame, hz)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
