#!/usr/bin/env python3
"""Converts PointCloud2 to LaserScan with wall-clock timestamps."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import struct
import math
import numpy as np

class PCtoScanNode(Node):
    def __init__(self):
        super().__init__('pc_to_scan_node')
        self.sub = self.create_subscription(
            PointCloud2, '/atlas/rgbd_camera/points', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info('PC-to-Scan node started (wall clock timestamps)')

    def cb(self, msg):
        # Extract x,y,z points
        point_step = msg.point_step
        data = msg.data
        
        # Find field offsets
        field_offsets = {}
        for field in msg.fields:
            field_offsets[field.name] = field.offset
        
        if 'x' not in field_offsets:
            return
            
        ox = field_offsets['x']
        oy = field_offsets['y']
        oz = field_offsets['z']
        
        # LaserScan parameters
        angle_min = -1.5708  # -90 degrees
        angle_max = 1.5708   # +90 degrees
        num_readings = 180
        angle_increment = (angle_max - angle_min) / num_readings
        ranges = [float('inf')] * num_readings
        
        for i in range(0, len(data) - point_step + 1, point_step):
            try:
                x = struct.unpack_from('f', data, i + ox)[0]
                y = struct.unpack_from('f', data, i + oy)[0]
                z = struct.unpack_from('f', data, i + oz)[0]
            except:
                continue
                
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue
            
            # Filter by height (relative to sensor)
            if z < -0.3 or z > 1.0:
                continue
                
            dist = math.sqrt(x*x + y*y)
            if dist < 0.15 or dist > 10.0:
                continue
                
            angle = math.atan2(y, x)
            if angle < angle_min or angle > angle_max:
                continue
                
            idx = int((angle - angle_min) / angle_increment)
            if 0 <= idx < num_readings:
                if dist < ranges[idx]:
                    ranges[idx] = dist
        
        # Build LaserScan with WALL CLOCK timestamp
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_footprint'
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 10.0
        scan.ranges = ranges
        
        self.pub.publish(scan)

def main():
    rclpy.init()
    rclpy.spin(PCtoScanNode())

if __name__ == '__main__':
    main()
