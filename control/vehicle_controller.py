#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import numpy as np
import math
from tf2_ros import TransformListener, Buffer

class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )
        
        self.emergency_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_callback,
            10
        )
        
        # TF listener for pose tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Control state
        self.current_path = None
        self.current_pose = None
        self.emergency_stop = False
        self.current_waypoint_index = 0
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_linear_speed', 0.5),      # m/s
                ('max_angular_speed', 1.0),     # rad/s
                ('goal_tolerance', 0.1),        # meters
                ('lookahead_distance', 1.0),    # meters
                ('control_frequency', 10.0)     # Hz
            ]
        )
        
        # Create control timer
        self.create_timer(
            1.0/self.get_parameter('control_frequency').value,
            self.control_loop
        )
        
    def path_callback(self, msg):
        self.current_path = msg
        self.current_waypoint_index = 0
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def emergency_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.stop_vehicle()
            
    def control_loop(self):
        if self.emergency_stop:
            self.stop_vehicle()
            return
            
        if not self.current_path or not self.current_pose:
            return
            
        # Get next waypoint
        target_pose = self.get_target_waypoint()
        if not target_pose:
            return
            
        # Calculate control commands
        cmd_vel = self.compute_velocity_commands(target_pose)
        
        # Publish commands
        self.cmd_vel_pub.publish(cmd_vel)
        
    def get_target_waypoint(self):
        if self.current_waypoint_index >= len(self.current_path.poses):
            return None
            
        # Get lookahead distance
        lookahead = self.get_parameter('lookahead_distance').value
        
        # Find the furthest waypoint within lookahead distance
        current_pos = self.current_pose.pose.position
        for i in range(self.current_waypoint_index, len(self.current_path.poses)):
            waypoint = self.current_path.poses[i]
            distance = self.compute_distance(current_pos, waypoint.pose.position)
            
            if distance > lookahead:
                break
                
            self.current_waypoint_index = i
            
        return self.current_path.poses[self.current_waypoint_index]
        
    def compute_velocity_commands(self, target_pose):
        cmd_vel = Twist()
        
        # Get current position and target
        current_pos = self.current_pose.pose.position
        target_pos = target_pose.pose.position
        
        # Calculate distance and angle to target
        distance = self.compute_distance(current_pos, target_pos)
        angle = self.compute_angle_to_target(target_pos)
        
        # Set linear velocity based on distance
        max_linear = self.get_parameter('max_linear_speed').value
        cmd_vel.linear.x = min(max_linear, distance)
        
        # Set angular velocity based on angle
        max_angular = self.get_parameter('max_angular_speed').value
        cmd_vel.angular.z = max(-max_angular, min(max_angular, angle))
        
        return cmd_vel
        
    def compute_distance(self, point1, point2):
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        return math.sqrt(dx*dx + dy*dy)
        
    def compute_angle_to_target(self, target_pos):
        # Get current position and orientation
        current_pos = self.current_pose.pose.position
        current_quat = self.current_pose.pose.orientation
        
        # Calculate current yaw
        current_yaw = 2 * math.atan2(current_quat.z, current_quat.w)
        
        # Calculate target angle
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        target_yaw = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_yaw - current_yaw
        
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        return angle_diff
        
    def stop_vehicle(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 