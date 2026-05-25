#!/usr/bin/env python3
"""
Requirement 4: Goal Position Detection
Subscribes to YOLO detections + odometry, estimates goal positions
in the map frame, and publishes them as PoseStamped goals.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from yolo_msgs.msg import DetectionArray
import math

class GoalDetectionNode(Node):
    def __init__(self):
        super().__init__('goal_detection_node')
        self.declare_parameter('camera_fov_h', 1.2113)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('goal_class', 'stop_sign')
        self.declare_parameter('estimated_depth', 1.5)
        self.camera_fov_h = self.get_parameter('camera_fov_h').value
        self.image_width = self.get_parameter('image_width').value
        self.goal_class = self.get_parameter('goal_class').value
        self.estimated_depth = self.get_parameter('estimated_depth').value
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.detected_goals = {}
        self.odom_sub = self.create_subscription(
            Odometry, '/atlas/odom_ground_truth', self.odom_callback, 10)
        self.detection_sub = self.create_subscription(
            DetectionArray, '/yolo/detections', self.detection_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/detected_goal', 10)
        self.all_goals_pub = self.create_publisher(PoseStamped, '/all_detected_goals', 10)
        self.get_logger().info('Goal Detection Node started')
        self.get_logger().info(f'Looking for goal class: {self.goal_class}')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def detection_callback(self, msg):
        for det in msg.detections:
            class_name = det.class_name
            confidence = det.score
            if confidence < 0.6:
                continue
            bbox_center_x = det.bbox.center.position.x
            angle_offset = ((bbox_center_x / self.image_width) - 0.5) * self.camera_fov_h
            bearing = self.robot_yaw + angle_offset
            depth = self.estimated_depth
            goal_x = self.robot_x + depth * math.cos(bearing)
            goal_y = self.robot_y + depth * math.sin(bearing)
            key = class_name
            if key in self.detected_goals:
                prev_x, prev_y = self.detected_goals[key]
                dist = math.sqrt((goal_x - prev_x)**2 + (goal_y - prev_y)**2)
                if dist < 0.5:
                    continue
            self.detected_goals[key] = (goal_x, goal_y)
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'odom'
            goal_msg.pose.position.x = goal_x
            goal_msg.pose.position.y = goal_y
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation.w = 1.0
            self.all_goals_pub.publish(goal_msg)
            self.get_logger().info(
                f'[GOAL] {class_name} (conf={confidence:.2f}) at map ({goal_x:.2f}, {goal_y:.2f})')
            if class_name == self.goal_class:
                self.goal_pub.publish(goal_msg)
                self.get_logger().info(f'*** TARGET GOAL DETECTED: {class_name} ***')

def main(args=None):
    rclpy.init(args=args)
    node = GoalDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
