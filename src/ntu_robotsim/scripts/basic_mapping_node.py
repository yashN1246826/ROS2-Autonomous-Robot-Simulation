#!/usr/bin/env python3
"""
Requirement 1: Basic Mapping
Uses ground-truth odometry + RGB-D pointcloud to build occupancy grid.
Ray-casting marks free space, endpoints mark obstacles.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
import numpy as np
import math
import struct


class BasicMappingNode(Node):
    def __init__(self):
        super().__init__('basic_mapping_node')

        self.map_resolution = 0.1
        self.map_width = 200
        self.map_height = 200
        self.map_origin_x = -10.0
        self.map_origin_y = -10.0

        # Use float array for hit/miss counting
        self.hit_count = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        self.miss_count = np.zeros((self.map_height, self.map_width), dtype=np.float32)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.odom_sub = self.create_subscription(
            Odometry, '/atlas/odom_ground_truth', self.odom_cb, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/atlas/rgbd_camera/points', self.pc_cb, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('Basic Mapping Node started (with obstacle detection)')

    def world_to_map(self, wx, wy):
        mx = int((wx - self.map_origin_x) / self.map_resolution)
        my = int((wy - self.map_origin_y) / self.map_resolution)
        return mx, my

    def in_bounds(self, mx, my):
        return 0 <= mx < self.map_width and 0 <= my < self.map_height

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def bresenham(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return cells

    def pc_cb(self, msg):
        point_step = msg.point_step
        data = msg.data

        field_offsets = {}
        for field in msg.fields:
            field_offsets[field.name] = field.offset
        if 'x' not in field_offsets:
            return

        ox = field_offsets['x']
        oy = field_offsets['y']
        oz = field_offsets['z']

        robot_mx, robot_my = self.world_to_map(self.robot_x, self.robot_y)

        # Sample every 20th point for cleaner map
        for i in range(0, len(data) - point_step + 1, point_step * 20):
            try:
                lx = struct.unpack_from('f', data, i + ox)[0]
                ly = struct.unpack_from('f', data, i + oy)[0]
                lz = struct.unpack_from('f', data, i + oz)[0]
            except:
                continue

            if math.isnan(lx) or math.isnan(ly) or math.isnan(lz):
                continue

            # Only use points at wall height (0.05m to 0.8m)
            if lz < 0.05 or lz > 0.8:
                continue

            dist = math.sqrt(lx * lx + ly * ly)
            if dist < 0.3 or dist > 6.0:
                continue

            # Transform to world frame
            angle = math.atan2(ly, lx) + self.robot_yaw
            world_x = self.robot_x + dist * math.cos(angle)
            world_y = self.robot_y + dist * math.sin(angle)

            obs_mx, obs_my = self.world_to_map(world_x, world_y)

            if not self.in_bounds(obs_mx, obs_my):
                continue

            # Ray-cast: free space along ray
            ray_cells = self.bresenham(robot_mx, robot_my, obs_mx, obs_my)
            for cx, cy in ray_cells[:-1]:
                if self.in_bounds(cx, cy):
                    self.miss_count[cy][cx] += 1.0

            # Obstacle at endpoint
            if self.in_bounds(obs_mx, obs_my):
                self.hit_count[obs_my][obs_mx] += 1.0

    def publish_map(self):
        map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        total = self.hit_count + self.miss_count
        observed = total > 0

        # Calculate occupancy probability where observed
        prob = np.zeros_like(total)
        prob[observed] = self.hit_count[observed] / total[observed]

        # Free space: low hit ratio
        free_mask = observed & (prob < 0.3)
        map_data[free_mask] = 0

        # Occupied: high hit ratio with enough observations
        occupied_mask = observed & (prob > 0.6) & (self.hit_count > 2)
        map_data[occupied_mask] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = map_data.flatten().tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BasicMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
