#!/usr/bin/env python3
"""
Requirement 7: Landmark Database
+ Extra: Detection Logger
Stores detected landmarks with map coordinates, class, confidence, count.
Publishes summary and saves to CSV.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from yolo_msgs.msg import DetectionArray
from std_msgs.msg import String
import math
import csv
import os
from datetime import datetime


class LandmarkDatabaseNode(Node):
    def __init__(self):
        super().__init__('landmark_database_node')
        self.declare_parameter('merge_distance', 0.8)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('csv_path', os.path.expanduser(
            '~/coursework_ws/landmark_log.csv'))
        self.declare_parameter('camera_fov_h', 1.047)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('estimated_depth', 1.5)

        self.merge_dist = self.get_parameter('merge_distance').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.csv_path = self.get_parameter('csv_path').value
        self.fov_h = self.get_parameter('camera_fov_h').value
        self.img_w = self.get_parameter('image_width').value
        self.est_depth = self.get_parameter('estimated_depth').value

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.landmarks = []

        self._init_csv()

        self.odom_sub = self.create_subscription(
            Odometry, '/atlas/odom_ground_truth', self.odom_cb, 10)
        self.det_sub = self.create_subscription(
            DetectionArray, '/yolo/detections', self.det_cb, 10)
        self.summary_pub = self.create_publisher(String, '/landmark_summary', 10)
        self.timer = self.create_timer(5.0, self.publish_summary)

        self.get_logger().info(f'Landmark Database started. CSV: {self.csv_path}')

    def _init_csv(self):
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'class', 'confidence', 'map_x', 'map_y',
                'robot_x', 'robot_y', 'robot_yaw', 'is_new'
            ])

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def det_cb(self, msg):
        for det in msg.detections:
            if det.score < self.conf_threshold:
                continue

            bbox_cx = det.bbox.center.position.x
            angle_off = ((bbox_cx / self.img_w) - 0.5) * self.fov_h
            bearing = self.robot_yaw + angle_off
            lx = self.robot_x + self.est_depth * math.cos(bearing)
            ly = self.robot_y + self.est_depth * math.sin(bearing)

            is_new = True
            for lm in self.landmarks:
                if lm['class'] == det.class_name:
                    dist = math.sqrt((lx - lm['x'])**2 + (ly - lm['y'])**2)
                    if dist < self.merge_dist:
                        lm['count'] += 1
                        lm['last_conf'] = det.score
                        lm['last_seen'] = datetime.now().isoformat()
                        n = lm['count']
                        lm['x'] = lm['x'] + (lx - lm['x']) / n
                        lm['y'] = lm['y'] + (ly - lm['y']) / n
                        is_new = False
                        break

            if is_new:
                landmark = {
                    'class': det.class_name,
                    'x': lx, 'y': ly,
                    'count': 1,
                    'first_seen': datetime.now().isoformat(),
                    'last_seen': datetime.now().isoformat(),
                    'last_conf': det.score,
                }
                self.landmarks.append(landmark)
                self.get_logger().info(
                    f'NEW LANDMARK: {det.class_name} at ({lx:.2f}, {ly:.2f})')

            self._log_csv(det, lx, ly, is_new)

    def _log_csv(self, det, lx, ly, is_new):
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().isoformat(),
                det.class_name, f'{det.score:.3f}',
                f'{lx:.3f}', f'{ly:.3f}',
                f'{self.robot_x:.3f}', f'{self.robot_y:.3f}',
                f'{self.robot_yaw:.3f}',
                'NEW' if is_new else 'UPDATE'
            ])

    def publish_summary(self):
        if not self.landmarks:
            return
        lines = ['=== LANDMARK DATABASE ===']
        class_counts = {}
        for lm in self.landmarks:
            c = lm['class']
            class_counts[c] = class_counts.get(c, 0) + 1
            lines.append(
                f"  {c} @ ({lm['x']:.2f},{lm['y']:.2f}) "
                f"seen {lm['count']}x conf={lm['last_conf']:.2f}")
        lines.append(f'--- Total: {len(self.landmarks)} landmarks ---')
        for c, n in class_counts.items():
            lines.append(f'  {c}: {n} unique')
        summary = '\n'.join(lines)
        self.summary_pub.publish(String(data=summary))
        self.get_logger().info(summary)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkDatabaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
