#!/usr/bin/env python3
"""Converts PointCloud2 to depth image for RTABMAP."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import numpy as np
import struct


class PCtoDepthImage(Node):
    def __init__(self):
        super().__init__('pc_to_depth_image')
        self.sub = self.create_subscription(
            PointCloud2, '/atlas/rgbd_camera/points', self.cb, 10)
        self.depth_pub = self.create_publisher(
            Image, '/atlas/rgbd_camera/depth_image', 10)
        self.camera_info = None
        self.info_sub = self.create_subscription(
            CameraInfo, '/atlas/rgbd_camera/camera_info', self.info_cb, 10)
        self.get_logger().info('PC to Depth Image converter started')

    def info_cb(self, msg):
        self.camera_info = msg

    def cb(self, msg):
        if self.camera_info is None:
            return

        width = self.camera_info.width
        height = self.camera_info.height

        if width == 0 or height == 0:
            return

        # Find field offsets
        field_offsets = {}
        for field in msg.fields:
            field_offsets[field.name] = field.offset

        if 'x' not in field_offsets or 'z' not in field_offsets:
            return

        ox = field_offsets['x']
        oy = field_offsets['y']
        oz = field_offsets['z']

        # Check if pointcloud is organized (has same width/height as image)
        if msg.width == width and msg.height == height:
            # Organized pointcloud — extract z values directly
            depth = np.zeros((height, width), dtype=np.float32)
            data = msg.data
            point_step = msg.point_step

            for row in range(height):
                for col in range(width):
                    idx = (row * width + col) * point_step
                    try:
                        z = struct.unpack_from('f', data, idx + oz)[0]
                        if not np.isnan(z) and not np.isinf(z):
                            depth[row, col] = z
                    except:
                        pass
        else:
            # Unorganized — project points into image using camera intrinsics
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            depth = np.zeros((height, width), dtype=np.float32)
            data = msg.data
            point_step = msg.point_step

            for i in range(0, len(data) - point_step + 1, point_step):
                try:
                    x = struct.unpack_from('f', data, i + ox)[0]
                    y = struct.unpack_from('f', data, i + oy)[0]
                    z = struct.unpack_from('f', data, i + oz)[0]
                except:
                    continue

                if np.isnan(z) or z <= 0:
                    continue

                u = int(fx * x / z + cx)
                v = int(fy * y / z + cy)

                if 0 <= u < width and 0 <= v < height:
                    if depth[v, u] == 0 or z < depth[v, u]:
                        depth[v, u] = z

        # Publish depth image
        depth_msg = Image()
        depth_msg.header = msg.header
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.height = height
        depth_msg.width = width
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = False
        depth_msg.step = width * 4
        depth_msg.data = depth.tobytes()

        self.depth_pub.publish(depth_msg)


def main():
    rclpy.init()
    rclpy.spin(PCtoDepthImage())


if __name__ == '__main__':
    main()
