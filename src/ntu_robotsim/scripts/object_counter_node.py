#!/usr/bin/env python3
"""
Requirement 8: Object Counting
Counts objects per class, tracks max simultaneous detections,
and saves results to object_log.log
"""
import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
import os


class ObjectCounterNode(Node):
    def __init__(self):
        super().__init__('object_counter_node')
        self.class_max_counts = {}
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_oz = 0.0
        self.robot_ow = 1.0

        self.log_path = os.path.expanduser('~/coursework_ws/object_log.log')

        self.det_sub = self.create_subscription(
            DetectionArray, '/yolo/detections', self.det_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/atlas/odom_ground_truth', self.odom_cb, 10)
        self.summary_pub = self.create_publisher(String, '/object_count/summary', 10)
        self.count_pubs = {}

        self.summary_timer = self.create_timer(3.0, self.publish_summary)

        self.get_logger().info('Object Counter Node started')
        self.get_logger().info(f'Log file: {self.log_path}')

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_oz = msg.pose.pose.orientation.z
        self.robot_ow = msg.pose.pose.orientation.w

    def det_cb(self, msg):
        frame_counts = {}
        for det in msg.detections:
            cls = det.class_name
            frame_counts[cls] = frame_counts.get(cls, 0) + 1

        for cls, count in frame_counts.items():
            if cls not in self.class_max_counts or count > self.class_max_counts[cls]:
                self.class_max_counts[cls] = count
                self.get_logger().info(f'NEW MAX: {count} x {cls} detected in single frame!')

                if cls not in self.count_pubs:
                    self.count_pubs[cls] = self.create_publisher(
                        Int32, f'/object_count/{cls}', 10)

            if cls in self.count_pubs:
                count_msg = Int32()
                count_msg.data = self.class_max_counts[cls]
                self.count_pubs[cls].publish(count_msg)

    def publish_summary(self):
        if not self.class_max_counts:
            return
        total = sum(self.class_max_counts.values())
        lines = ['=== OBJECT COUNT SUMMARY ===']
        for cls, count in self.class_max_counts.items():
            lines.append(f'  {cls}: {count} detected')
        lines.append(f'--- Total objects: {total} ---')
        summary = '\n'.join(lines)
        self.summary_pub.publish(String(data=summary))
        self.get_logger().info(summary)

    def save_log(self):
        """Save object counts to log file"""
        try:
            with open(self.log_path, 'w') as f:
                for cls, count in self.class_max_counts.items():
                    line = (
                        f'Number of {cls}s detected: {count} ; '
                        f'Robot odometry:Position(x={self.robot_x:.2f}, y={self.robot_y:.2f}, z=0.0), '
                        f'Orientation(x=0.0, y=0.0, z={self.robot_oz:.4f}, w={self.robot_ow:.4f})'
                    )
                    f.write(line + '\n')
            self.get_logger().info(f'Object log saved to {self.log_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save log: {e}')

    def destroy_node(self):
        self.save_log()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectCounterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_log()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
