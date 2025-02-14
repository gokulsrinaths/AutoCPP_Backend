#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import serial
import pynmea2
import math
import pyproj

class GPSInterface(Node):
    def __init__(self):
        super().__init__('gps_interface')
        
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
                ('port', '/dev/ttyUSB0'),
                ('baud_rate', 9600),
                ('update_rate', 10.0),  # Hz
                ('origin_latitude', 0.0),
                ('origin_longitude', 0.0)
            ]
        )
        
        # Initialize serial connection
        try:
            self.serial_port = serial.Serial(
                self.get_parameter('port').value,
                self.get_parameter('baud_rate').value,
                timeout=1.0
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open GPS serial port: {str(e)}')
            return
            
        # Create timer for GPS reading
        self.create_timer(
            1.0/self.get_parameter('update_rate').value,
            self.read_gps
        )
        
        # Initialize projection
        self.projector = pyproj.Proj(proj='utm', zone=10, ellps='WGS84')  # Update zone based on your location
        
    def read_gps(self):
        try:
            if self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('ascii', errors='replace')
                if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                    msg = pynmea2.parse(line)
                    if msg.status == 'A':  # Valid fix
                        self.publish_gps_data(msg)
                        
        except serial.SerialException as e:
            self.get_logger().error(f'GPS read error: {str(e)}')
            
    def publish_gps_data(self, nmea_msg):
        # Create NavSatFix message
        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = 'gps'
        
        fix_msg.latitude = nmea_msg.latitude
        fix_msg.longitude = nmea_msg.longitude
        fix_msg.altitude = 0.0  # Set if available from GPS
        
        # Publish raw GPS data
        self.gps_pub.publish(fix_msg)
        
        # Convert to local coordinates and publish pose
        pose_msg = self.gps_to_pose(nmea_msg)
        self.pose_pub.publish(pose_msg)
        
    def gps_to_pose(self, nmea_msg):
        # Convert GPS coordinates to UTM
        easting, northing = self.projector(
            nmea_msg.longitude,
            nmea_msg.latitude
        )
        
        # Convert to local coordinates
        origin_easting, origin_northing = self.projector(
            self.get_parameter('origin_longitude').value,
            self.get_parameter('origin_latitude').value
        )
        
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Set position relative to origin
        pose_msg.pose.position.x = easting - origin_easting
        pose_msg.pose.position.y = northing - origin_northing
        pose_msg.pose.position.z = 0.0
        
        # Set orientation (if you have heading information)
        if hasattr(nmea_msg, 'true_course') and nmea_msg.true_course:
            heading = math.radians(nmea_msg.true_course)
            pose_msg.pose.orientation.z = math.sin(heading/2)
            pose_msg.pose.orientation.w = math.cos(heading/2)
        else:
            pose_msg.pose.orientation.w = 1.0
            
        return pose_msg

def main(args=None):
    rclpy.init(args=args)
    node = GPSInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 