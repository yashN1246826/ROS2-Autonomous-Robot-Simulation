#!/usr/bin/env python3
"""
Extra: Resource Monitor
Tracks CPU, memory usage during the cognitive robotics pipeline.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import os


class ResourceMonitorNode(Node):
    def __init__(self):
        super().__init__('resource_monitor_node')
        self.declare_parameter('interval', 5.0)
        interval = self.get_parameter('interval').value
        self.stats_pub = self.create_publisher(String, '/system_resources', 10)
        self.timer = self.create_timer(interval, self.publish_stats)
        self.get_logger().info(f'Resource Monitor started (every {interval}s)')

    def publish_stats(self):
        cpu = psutil.cpu_percent(interval=0.1)
        mem = psutil.virtual_memory()
        proc = psutil.Process(os.getpid())
        stats = (
            f'CPU: {cpu:.1f}% | '
            f'RAM: {mem.used / 1e9:.1f}/{mem.total / 1e9:.1f}GB ({mem.percent}%) | '
            f'Node RSS: {proc.memory_info().rss / 1e6:.1f}MB')
        self.stats_pub.publish(String(data=stats))
        self.get_logger().info(stats)


def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
