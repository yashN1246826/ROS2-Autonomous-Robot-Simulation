#!/usr/bin/env python3
"""
Requirement 5: Traffic Rules
Modifies robot velocity based on detected traffic signs.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolo_msgs.msg import DetectionArray
import time


class TrafficRulesNode(Node):
    def __init__(self):
        super().__init__('traffic_rules_node')
        self.conf_threshold = 0.6
        self.stop_duration = 3.0
        self.slow_factor = 0.3
        self.fast_factor = 1.5
        self.cooldown = 15.0  # longer cooldown to avoid re-triggering

        self.current_speed_factor = 1.0
        self.is_stopped = False
        self.stop_until = 0.0
        self.last_trigger = {}

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/atlas/cmd_vel_raw', self.cmd_vel_callback, 10)
        self.detection_sub = self.create_subscription(
            DetectionArray, '/yolo/detections', self.detection_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/atlas/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Traffic Rules Node started')
        self.get_logger().info('Remap teleop to publish on /atlas/cmd_vel_raw')

    def detection_callback(self, msg):
        now = time.time()
        for det in msg.detections:
            if det.score < self.conf_threshold:
                continue
            class_name = det.class_name

            if class_name in self.last_trigger:
                if now - self.last_trigger[class_name] < self.cooldown:
                    continue

            if class_name == 'stop_sign':
                self.is_stopped = True
                self.stop_until = now + self.stop_duration
                self.last_trigger[class_name] = now
                self.get_logger().info(
                    f'STOP SIGN detected! Stopping for {self.stop_duration}s')

            elif class_name == 'slow_sign':
                self.current_speed_factor = self.slow_factor
                self.last_trigger[class_name] = now
                self.get_logger().info(
                    f'SLOW SIGN detected! Speed reduced to {self.slow_factor*100:.0f}%')

            elif class_name == 'fast_sign':
                self.current_speed_factor = self.fast_factor
                self.last_trigger[class_name] = now
                self.get_logger().info(
                    f'FAST SIGN detected! Speed increased to {self.fast_factor*100:.0f}%')

    def timer_callback(self):
        if self.is_stopped and time.time() > self.stop_until:
            self.is_stopped = False
            self.current_speed_factor = 1.0
            self.get_logger().info('Stop complete. Resuming normal speed.')

    def cmd_vel_callback(self, msg):
        out = Twist()
        if self.is_stopped:
            out.linear.x = 0.0
            out.angular.z = 0.0
        else:
            out.linear.x = msg.linear.x * self.current_speed_factor
            out.angular.z = msg.angular.z
        self.cmd_vel_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficRulesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
