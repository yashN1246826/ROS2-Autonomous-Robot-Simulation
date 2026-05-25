#!/usr/bin/env python3
"""
Requirement 6: Enhanced Navigation
Uses odometry feedback to navigate to goal poses autonomously.
Implements obstacle avoidance using LaserScan/PointCloud data.
This demonstrates Nav2-style autonomous navigation.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import math
import struct
import numpy as np


class Nav2GoalNode(Node):
    def __init__(self):
        super().__init__('nav2_goal_node')

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Goal
        self.goal_x = None
        self.goal_y = None
        self.goal_reached = True
        self.goal_tolerance = 0.3

        # Obstacle detection
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        self.front_dist = float('inf')
        self.obstacle_threshold = 0.5

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/atlas/odom_ground_truth', self.odom_cb, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/atlas/rgbd_camera/points', self.pc_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/atlas/cmd_vel', 10)

        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Nav2 Goal Node started')
        self.get_logger().info('Send goals on /goal_pose or use:')
        self.get_logger().info('  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '
                               '"{header: {frame_id: odom}, pose: {position: {x: -1.0, y: -3.0}, orientation: {w: 1.0}}}"')

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_reached = False
        self.get_logger().info(
            f'New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def pc_cb(self, msg):
        """Extract obstacle distances from pointcloud."""
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

        for i in range(0, len(data) - point_step + 1, point_step * 4):  # Skip points for speed
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
            angle = math.atan2(y, x)

            if -0.4 < angle < 0.4:  # Front
                front_min = min(front_min, dist)
            elif 0.4 <= angle < 1.2:  # Left
                left_min = min(left_min, dist)
            elif -1.2 < angle <= -0.4:  # Right
                right_min = min(right_min, dist)

        self.front_dist = front_min
        self.front_clear = front_min > self.obstacle_threshold
        self.left_clear = left_min > self.obstacle_threshold
        self.right_clear = right_min > self.obstacle_threshold

    def control_loop(self):
        cmd = Twist()

        if self.goal_x is None or self.goal_reached:
            self.cmd_pub.publish(cmd)
            return

        # Distance and angle to goal
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - self.robot_yaw

        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2.0 * math.pi

        # Check if goal reached
        if dist < self.goal_tolerance:
            self.goal_reached = True
            self.get_logger().info(
                f'GOAL REACHED! ({self.goal_x:.2f}, {self.goal_y:.2f})')
            self.cmd_pub.publish(cmd)
            return

        # Navigation with obstacle avoidance
        if not self.front_clear:
            # Obstacle ahead — turn away
            if self.left_clear:
                cmd.linear.x = 0.05
                cmd.angular.z = 0.5
                self.get_logger().info(f'Obstacle at {self.front_dist:.2f}m — turning left')
            elif self.right_clear:
                cmd.linear.x = 0.05
                cmd.angular.z = -0.5
                self.get_logger().info(f'Obstacle at {self.front_dist:.2f}m — turning right')
            else:
                cmd.linear.x = -0.1
                cmd.angular.z = 0.8
                self.get_logger().info('Boxed in — backing up')
        elif abs(angle_diff) > 0.3:
            # Turn towards goal
            cmd.linear.x = 0.05
            cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
        else:
            # Drive towards goal
            cmd.linear.x = min(0.2, dist * 0.5)
            cmd.angular.z = angle_diff * 0.5

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'Navigating to ({self.goal_x:.2f},{self.goal_y:.2f}) '
            f'dist={dist:.2f} angle={math.degrees(angle_diff):.0f}° '
            f'front={self.front_dist:.2f}m')


def main():
    rclpy.init()
    node = Nav2GoalNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
