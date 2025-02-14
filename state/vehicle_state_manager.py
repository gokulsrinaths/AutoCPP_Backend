#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import json
from enum import Enum
from datetime import datetime

class VehicleState(Enum):
    IDLE = "IDLE"
    MOVING_TO_PICKUP = "MOVING_TO_PICKUP"
    WAITING_FOR_PASSENGER = "WAITING_FOR_PASSENGER"
    MOVING_TO_DROPOFF = "MOVING_TO_DROPOFF"
    EMERGENCY = "EMERGENCY"
    CHARGING = "CHARGING"
    MAINTENANCE = "MAINTENANCE"

class VehicleStateManager(Node):
    def __init__(self):
        super().__init__('vehicle_state_manager')
        
        # Publishers
        self.state_pub = self.create_publisher(
            String,
            '/vehicle/state',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/vehicle/status',
            10
        )
        
        # Subscribers
        self.emergency_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        # State variables
        self.current_state = VehicleState.IDLE
        self.emergency_active = False
        self.current_pose = None
        self.current_path = None
        self.current_booking = None
        self.battery_level = 100.0  # Simulated battery level
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_battery_level', 20.0),
                ('status_update_rate', 1.0),  # Hz
                ('pickup_radius', 5.0),       # meters
                ('dropoff_radius', 5.0)       # meters
            ]
        )
        
        # Create status update timer
        self.create_timer(
            1.0/self.get_parameter('status_update_rate').value,
            self.publish_status
        )
        
    def set_state(self, new_state: VehicleState):
        if new_state != self.current_state:
            self.current_state = new_state
            self.publish_state()
            
    def emergency_callback(self, msg):
        self.emergency_active = msg.data
        if self.emergency_active:
            self.set_state(VehicleState.EMERGENCY)
            
    def pose_callback(self, msg):
        self.current_pose = msg
        self.check_state_transition()
        
    def path_callback(self, msg):
        self.current_path = msg
        
    def start_new_booking(self, booking_data):
        self.current_booking = booking_data
        self.set_state(VehicleState.MOVING_TO_PICKUP)
        
    def check_state_transition(self):
        if self.emergency_active:
            return
            
        if self.get_battery_level() < self.get_parameter('min_battery_level').value:
            self.set_state(VehicleState.CHARGING)
            return
            
        if self.current_state == VehicleState.MOVING_TO_PICKUP:
            if self.is_at_pickup():
                self.set_state(VehicleState.WAITING_FOR_PASSENGER)
                
        elif self.current_state == VehicleState.MOVING_TO_DROPOFF:
            if self.is_at_dropoff():
                self.set_state(VehicleState.IDLE)
                self.current_booking = None
                
    def is_at_pickup(self):
        if not self.current_booking or not self.current_pose:
            return False
            
        pickup_pos = self.current_booking['pickup_location']
        current_pos = self.current_pose.pose.position
        
        distance = self.compute_distance(
            current_pos,
            pickup_pos
        )
        
        return distance < self.get_parameter('pickup_radius').value
        
    def is_at_dropoff(self):
        if not self.current_booking or not self.current_pose:
            return False
            
        dropoff_pos = self.current_booking['dropoff_location']
        current_pos = self.current_pose.pose.position
        
        distance = self.compute_distance(
            current_pos,
            dropoff_pos
        )
        
        return distance < self.get_parameter('dropoff_radius').value
        
    def compute_distance(self, point1, point2):
        dx = point2['x'] - point1.x
        dy = point2['y'] - point1.y
        return (dx*dx + dy*dy)**0.5
        
    def get_battery_level(self):
        # TODO: Implement actual battery level reading
        # Simulated battery drain
        self.battery_level -= 0.01
        return max(0.0, self.battery_level)
        
    def publish_state(self):
        msg = String()
        msg.data = self.current_state.value
        self.state_pub.publish(msg)
        
    def publish_status(self):
        status = {
            'state': self.current_state.value,
            'battery_level': self.get_battery_level(),
            'emergency_active': self.emergency_active,
            'timestamp': datetime.now().isoformat(),
            'booking': self.current_booking.copy() if self.current_booking else None,
            'position': {
                'x': self.current_pose.pose.position.x,
                'y': self.current_pose.pose.position.y
            } if self.current_pose else None
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        
    def start_passenger_ride(self):
        """Called when passenger is picked up"""
        if self.current_state == VehicleState.WAITING_FOR_PASSENGER:
            self.set_state(VehicleState.MOVING_TO_DROPOFF)
            
    def cancel_current_booking(self):
        """Cancel current booking and return to idle"""
        self.current_booking = None
        self.set_state(VehicleState.IDLE)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleStateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 