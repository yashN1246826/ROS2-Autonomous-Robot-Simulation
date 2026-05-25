#!/usr/bin/env python3
"""Re-stamps /scan messages with wall clock time so Nav2/RViz accept them."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.cb, qos)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info('Scan restamper: /scan_raw -> /scan (wall clock, BEST_EFFORT)')

    def cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ScanRestamper())

if __name__ == '__main__':
    main()
