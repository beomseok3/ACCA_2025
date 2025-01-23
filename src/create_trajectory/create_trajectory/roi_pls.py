#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import struct
from math import isnan
import time

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'
    offset = 0
    for field in sorted(fields, key=lambda f: f.offset):
        while offset < field.offset:
            fmt += 'x'
            offset += 1
        if field_names is None or field.name in field_names:
            if field.datatype == 1:  # INT8
                fmt += 'b'
                offset += 1
            elif field.datatype == 2:  # UINT8
                fmt += 'B'
                offset += 1
            elif field.datatype == 3:  # INT16
                fmt += 'h'
                offset += 2
            elif field.datatype == 4:  # UINT16
                fmt += 'H'
                offset += 2
            elif field.datatype == 5:  # INT32
                fmt += 'i'
                offset += 4
            elif field.datatype == 6:  # UINT32
                fmt += 'I'
                offset += 4
            elif field.datatype == 7:  # FLOAT32
                fmt += 'f'
                offset += 4
            elif field.datatype == 8:  # FLOAT64
                fmt += 'd'
                offset += 8
            else:
                raise Exception('Unknown PointField datatype [%d]' % field.datatype)
        else:
            fmt += 'x' * field.count
            offset += field.count
    return fmt

class PointCloudSaver(Node):

    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',  
            self.pointcloud_callback,
            10)
        self.subscription  
        self.latest_cloud = None
        self.timer = self.create_timer(5.0, self.timer_callback)

    def pointcloud_callback(self, msg):
        self.latest_cloud = msg

    def timer_callback(self):
        if self.latest_cloud is not None:
            self.save_pointcloud(self.latest_cloud)

    def save_pointcloud(self, msg):
        points = np.array(list(self.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)

        filename = f'output_{int(time.time())}.pcd'
        o3d.io.write_point_cloud(filename, cloud)
        self.get_logger().info(f'Saved point cloud to {filename}')

    def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
        assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'

        fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
        width, height, point_step, row_step, data = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data
        unpack_from = struct.Struct(fmt).unpack_from

        if skip_nans:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def main(args=None):
    rclpy.init(args=args)
    pointcloud_saver = PointCloudSaver()
    rclpy.spin(pointcloud_saver)
    pointcloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
