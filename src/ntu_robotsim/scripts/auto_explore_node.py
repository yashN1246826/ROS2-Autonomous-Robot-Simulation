#!/usr/bin/env python3
"""
Extra: Auto-Explore using wall-following algorithm.
Robot autonomously navigates the maze without teleop.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import struct
import math


class AutoExploreNode(Node):
    def __init__(self):
        super().__init__('auto_explore_node')
        self.linear_speed = 0.2
        self.angular_speed = 0.6
        self.wall_dist = 1.2
        self.front_thresh = 0.6

        self.pc_sub = self.create_subscription(
            PointCloud2, '/atlas/rgbd_camera/points', self.pc_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/atlas/cmd_vel', 10)

        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')

        self.state = 'FORWARD'
        self.turn_count = 0

        self.timer = self.create_timer(0.1, self.navigate)
        self.get_logger().info('Auto-Explore started (improved wall-follower)')

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

        front_min = float('inf')
        left_min = float('inf')
        right_min = float('inf')

        for i in range(0, len(data) - point_step + 1, point_step * 4):
            try:
                x = struct.unpack_from('f', data, i + ox)[0]
                y = struct.unpack_from('f', data, i + oy)[0]
                z = struct.unpack_from('f', data, i + oz)[0]
            except:
                continue
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue
            if z < -0.1 or z > 0.5:
                continue
            dist = math.sqrt(x * x + y * y)
            if dist < 0.1:
                continue
            angle = math.atan2(y, x)

            if -0.5 < angle < 0.5:
                front_min = min(front_min, dist)
            elif 0.5 <= angle < 1.5:
                left_min = min(left_min, dist)
            elif -1.5 < angle <= -0.5:
                right_min = min(right_min, dist)

        self.front_dist = front_min
        self.left_dist = left_min
        self.right_dist = right_min

    def navigate(self):
        cmd = Twist()

        if self.front_dist < self.front_thresh:
            # Wall ahead - turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            self.state = 'WALL AHEAD - turning right'
            self.turn_count = 0
        elif self.left_dist < self.wall_dist * 0.5:
            # Too close to left wall - slight right
            cmd.linear.x = self.linear_speed
            cmd.angular.z = -0.2
            self.state = 'Too close left - correcting right'
            self.turn_count = 0
        elif self.left_dist < self.wall_dist:
            # Good - following left wall
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.state = 'Following left wall'
            self.turn_count = 0
        else:
            # No wall on left - turn left to find one, but keep moving forward
            self.turn_count += 1
            if self.turn_count > 30:
                # Been turning too long - just go forward
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
                self.state = 'No wall - driving forward'
            else:
                cmd.linear.x = self.linear_speed * 0.7
                cmd.angular.z = 0.3
                self.state = 'Lost wall - turning left'

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'{self.state} | front={self.front_dist:.2f}m left={self.left_dist:.2f}m right={self.right_dist:.2f}m',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = AutoExploreNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
