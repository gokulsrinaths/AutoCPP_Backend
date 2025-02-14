from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import launch.conditions

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('AutoCPP_Backend')
    
    # Config file paths
    gps_config = os.path.join(pkg_dir, 'config', 'gps_config.yaml')
    camera_config = os.path.join(pkg_dir, 'config', 'camera_config.yaml')
    navigation_config = os.path.join(pkg_dir, 'config', 'navigation_config.yaml')
    vehicle_config = os.path.join(pkg_dir, 'config', 'vehicle_config.yaml')
    
    # Launch arguments
    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation instead of real hardware'
    )
    
    # Nodes to launch
    nodes = [
        # Perception nodes
        Node(
            package='AutoCPP_Backend',
            executable='object_detector',
            name='object_detector',
            parameters=[camera_config],
            remappings=[('/camera/image', '/camera/color/image_raw')]
        ),
        
        # Navigation and Control nodes
        Node(
            package='AutoCPP_Backend',
            executable='path_planner',
            name='path_planner',
            parameters=[navigation_config]
        ),
        
        Node(
            package='AutoCPP_Backend',
            executable='vehicle_controller',
            name='vehicle_controller',
            parameters=[vehicle_config]
        ),
        
        # State Management
        Node(
            package='AutoCPP_Backend',
            executable='vehicle_state_manager',
            name='vehicle_state_manager'
        ),
        
        # Hardware Interface nodes
        Node(
            package='AutoCPP_Backend',
            executable='gps_interface',
            name='gps_interface',
            parameters=[gps_config]
        ),
        
        # API Bridge
        Node(
            package='AutoCPP_Backend',
            executable='api_bridge',
            name='api_bridge'
        ),
        
        # Visualization (if not in simulation)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=launch.conditions.UnlessCondition(use_sim)
        )
    ]
    
    return LaunchDescription([
        declare_use_sim_arg,
        *nodes
    ]) 