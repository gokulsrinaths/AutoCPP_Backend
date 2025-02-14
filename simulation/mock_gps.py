#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

class MockGPS(Node):
    def __init__(self):
        super().__init__('mock_gps')
        
        # Publishers
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/gps/fix',
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/gps/pose',
            10
        )
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 1.0),
                ('start_latitude', 34.0589),
                ('start_longitude', -117.8194),
                ('simulate_movement', True)
            ]
        )
        
        # Initialize position
        self.current_lat = self.get_parameter('start_latitude').value
        self.current_lon = self.get_parameter('start_longitude').value
        self.heading = 0.0  # radians
        
        # Create timer for GPS updates
        self.create_timer(
            1.0/self.get_parameter('update_rate').value,
            self.publish_gps
        )
        
    def publish_gps(self):
        # Simulate movement if enabled
        if self.get_parameter('simulate_movement').value:
            self.simulate_movement()
        
        # Create and publish NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'
        gps_msg.latitude = self.current_lat
        gps_msg.longitude = self.current_lon
        gps_msg.altitude = 0.0
        
        self.gps_pub.publish(gps_msg)
        
        # Create and publish PoseStamped message
        pose_msg = self.create_pose_message()
        self.pose_pub.publish(pose_msg)
        
    def simulate_movement(self):
        # Simulate simple circular movement
        radius = 0.0001  # roughly 10 meters
        angular_speed = 0.1  # radians per second
        
        self.heading += angular_speed
        
        # Update position
        self.current_lat += radius * math.cos(self.heading)
        self.current_lon += radius * math.sin(self.heading)
        
    def create_pose_message(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Convert GPS to local coordinates (simplified)
        origin_lat = self.get_parameter('start_latitude').value
        origin_lon = self.get_parameter('start_longitude').value
        
        # Rough conversion (not accurate for long distances)
        meters_per_degree = 111319.9
        x = (self.current_lon - origin_lon) * meters_per_degree * math.cos(math.radians(origin_lat))
        y = (self.current_lat - origin_lat) * meters_per_degree
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        
        # Set orientation based on heading
        pose_msg.pose.orientation.z = math.sin(self.heading/2)
        pose_msg.pose.orientation.w = math.cos(self.heading/2)
        
        return pose_msg

def main(args=None):
    rclpy.init(args=args)
    node = MockGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 