#!/usr/bin/env python3
"""
Extra: Maze Completion Timer
Tracks total distance, elapsed time, and average speed.
Starts when robot first moves, stops on /maze_complete.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import math
import time


class MazeTimerNode(Node):
    def __init__(self):
        super().__init__('maze_timer_node')
        self.start_time = None
        self.total_distance = 0.0
        self.prev_x = None
        self.prev_y = None
        self.is_running = False
        self.is_complete = False

        self.odom_sub = self.create_subscription(
            Odometry, '/atlas/odom_ground_truth', self.odom_cb, 10)
        self.complete_sub = self.create_subscription(
            Bool, '/maze_complete', self.complete_cb, 10)
        self.stats_pub = self.create_publisher(String, '/maze_stats', 10)
        self.timer = self.create_timer(2.0, self.publish_stats)

        self.get_logger().info('Maze Timer ready. Will start when robot moves.')
        self.get_logger().info('To stop: ros2 topic pub --once /maze_complete std_msgs/msg/Bool "{data: true}"')

    def odom_cb(self, msg):
        if self.is_complete:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.prev_x is not None:
            dx = x - self.prev_x
            dy = y - self.prev_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist > 0.001 and not self.is_running:
                self.is_running = True
                self.start_time = time.time()
                self.get_logger().info('Timer STARTED - robot is moving!')

            if self.is_running and dist < 1.0:
                self.total_distance += dist

        self.prev_x = x
        self.prev_y = y

    def complete_cb(self, msg):
        if msg.data and self.is_running:
            self.is_complete = True
            elapsed = time.time() - self.start_time
            avg_speed = self.total_distance / elapsed if elapsed > 0 else 0
            self.get_logger().info(
                f'MAZE COMPLETE! Time: {elapsed:.1f}s, '
                f'Distance: {self.total_distance:.2f}m, '
                f'Avg speed: {avg_speed:.3f} m/s')

    def publish_stats(self):
        if not self.is_running:
            return
        elapsed = time.time() - self.start_time
        avg_speed = self.total_distance / elapsed if elapsed > 0 else 0
        status = 'COMPLETE' if self.is_complete else 'RUNNING'
        stats = (
            f'[{status}] Time: {elapsed:.1f}s | '
            f'Dist: {self.total_distance:.2f}m | '
            f'Speed: {avg_speed:.3f}m/s')
        self.stats_pub.publish(String(data=stats))
        self.get_logger().info(stats)


def main(args=None):
    rclpy.init(args=args)
    node = MazeTimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
