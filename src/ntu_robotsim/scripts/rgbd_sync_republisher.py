#!/usr/bin/env python3
"""
Republishes RGB image, depth image (from pointcloud), and camera_info
with synchronized wall-clock timestamps for RTABMAP.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import numpy as np
import struct


class RGBDSyncRepublisher(Node):
    def __init__(self):
        super().__init__('rgbd_sync_republisher')

        self.latest_rgb = None
        self.latest_info = None

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/atlas/rgbd_camera/image', self.rgb_cb, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/atlas/rgbd_camera/camera_info', self.info_cb, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/atlas/rgbd_camera/points', self.pc_cb, 10)

        # Publishers with synced timestamps
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)

        self.get_logger().info('RGBD Sync Republisher started')

    def rgb_cb(self, msg):
        self.latest_rgb = msg

    def info_cb(self, msg):
        self.latest_info = msg

    def pc_cb(self, msg):
        if self.latest_rgb is None or self.latest_info is None:
            return

        width = self.latest_info.width
        height = self.latest_info.height
        if width == 0 or height == 0:
            return

        now = self.get_clock().now().to_msg()

        # Extract depth from pointcloud
        field_offsets = {}
        for field in msg.fields:
            field_offsets[field.name] = field.offset
        if 'x' not in field_offsets:
            return

        oz = field_offsets['z']
        point_step = msg.point_step
        data = msg.data

        depth = np.zeros((height, width), dtype=np.float32)

        if msg.width == width and msg.height == height:
            for i in range(height * width):
                idx = i * point_step
                if idx + oz + 4 <= len(data):
                    z = struct.unpack_from('f', data, idx + oz)[0]
                    if not np.isnan(z) and not np.isinf(z) and z > 0:
                        row = i // width
                        col = i % width
                        depth[row, col] = z
        else:
            ox = field_offsets['x']
            oy = field_offsets['y']
            fx = self.latest_info.k[0]
            fy = self.latest_info.k[4]
            cx = self.latest_info.k[2]
            cy = self.latest_info.k[5]
            if fx == 0 or fy == 0:
                return
            for i in range(0, len(data) - point_step + 1, point_step):
                x = struct.unpack_from('f', data, i + ox)[0]
                y = struct.unpack_from('f', data, i + oy)[0]
                z = struct.unpack_from('f', data, i + oz)[0]
                if np.isnan(z) or z <= 0:
                    continue
                u = int(fx * x / z + cx)
                v = int(fy * y / z + cy)
                if 0 <= u < width and 0 <= v < height:
                    if depth[v, u] == 0 or z < depth[v, u]:
                        depth[v, u] = z

        # Publish RGB with synced timestamp
        rgb_msg = self.latest_rgb
        rgb_msg.header.stamp = now
        rgb_msg.header.frame_id = 'atlas/realsense'
        self.rgb_pub.publish(rgb_msg)

        # Publish depth with synced timestamp
        depth_msg = Image()
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'atlas/realsense'
        depth_msg.height = height
        depth_msg.width = width
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = False
        depth_msg.step = width * 4
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)

        # Publish camera_info with synced timestamp
        info_msg = self.latest_info
        info_msg.header.stamp = now
        info_msg.header.frame_id = 'atlas/realsense'
        self.info_pub.publish(info_msg)

        self.get_logger().info('Published synced RGBD frame', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    rclpy.spin(RGBDSyncRepublisher())


if __name__ == '__main__':
    main()
