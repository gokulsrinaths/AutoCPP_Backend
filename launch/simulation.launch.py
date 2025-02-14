from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('AutoCPP_Backend')
    
    # Config files
    gps_config = os.path.join(pkg_dir, 'config', 'gps_config.yaml')
    camera_config = os.path.join(pkg_dir, 'config', 'camera_config.yaml')
    
    nodes = [
        # Mock GPS publisher
        Node(
            package='AutoCPP_Backend',
            executable='mock_gps',
            name='mock_gps',
            parameters=[{
                'update_rate': 1.0,  # Hz
                'start_latitude': 34.0589,  # Example coordinates
                'start_longitude': -117.8194,
                'simulate_movement': True
            }]
        ),
        
        # Mock camera publisher
        Node(
            package='AutoCPP_Backend',
            executable='mock_camera',
            name='mock_camera',
            parameters=[{
                'fps': 30,
                'image_width': 640,
                'image_height': 480,
                'simulate_objects': True
            }]
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'simulation.rviz')]
        ),
        
        # rqt_graph for viewing node connections
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph'
        ),
        
        # Mock booking client
        Node(
            package='AutoCPP_Backend',
            executable='mock_booking_client',
            name='mock_booking_client',
            parameters=[{
                'simulate_bookings': True,
                'booking_interval': 30.0  # seconds
            }]
        )
    ]
    
    return LaunchDescription(nodes) 