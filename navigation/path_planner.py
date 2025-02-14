#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray
import numpy as np
from queue import PriorityQueue

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.obstacles_sub = self.create_subscription(
            MarkerArray,
            '/detected_obstacles',
            self.obstacles_callback,
            10
        )
        
        # Initialize map and obstacles
        self.map = None
        self.obstacles = []
        self.current_path = None
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('planning_resolution', 0.5),  # meters
                ('obstacle_inflation', 1.0),   # meters
                ('max_planning_time', 5.0)     # seconds
            ]
        )
        
    def map_callback(self, msg):
        self.map = msg
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.origin = msg.info.origin
        
    def obstacles_callback(self, msg):
        self.obstacles = msg.markers
        # Replan if path is blocked
        if self.current_path and self.is_path_blocked():
            self.replan_path()
            
    def goal_callback(self, msg):
        if not self.map:
            self.get_logger().warn('No map received yet')
            return
            
        start_pose = self.get_current_pose()
        goal_pose = msg
        
        # Plan path
        path = self.plan_path(start_pose, goal_pose)
        if path:
            self.current_path = path
            self.path_pub.publish(path)
        else:
            self.get_logger().warn('No path found')
            
    def plan_path(self, start, goal):
        # Convert poses to grid coordinates
        start_grid = self.world_to_grid(start.pose.position)
        goal_grid = self.world_to_grid(goal.pose.position)
        
        # A* path planning
        path = self.a_star(start_grid, goal_grid)
        if not path:
            return None
            
        # Convert path to ROS message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for point in path:
            world_point = self.grid_to_world(point)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = world_point
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        return path_msg
        
    def a_star(self, start, goal):
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while not frontier.empty():
            current = frontier.get()[1]
            
            if current == goal:
                break
                
            for next_pos in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal, next_pos)
                    frontier.put((priority, next_pos))
                    came_from[next_pos] = current
                    
        # Reconstruct path
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        
        return path
        
    def get_neighbors(self, pos):
        neighbors = []
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]:
            next_pos = (pos[0] + dx, pos[1] + dy)
            if self.is_valid_position(next_pos):
                neighbors.append(next_pos)
        return neighbors
        
    def is_valid_position(self, pos):
        if not (0 <= pos[0] < self.map_width and 0 <= pos[1] < self.map_height):
            return False
            
        # Check if position is obstacle-free
        index = pos[1] * self.map_width + pos[0]
        return self.map.data[index] < 50  # Less than 50 means free space
        
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
    def world_to_grid(self, point):
        x = int((point.x - self.origin.position.x) / self.map_resolution)
        y = int((point.y - self.origin.position.y) / self.map_resolution)
        return (x, y)
        
    def grid_to_world(self, point):
        world_point = Point()
        world_point.x = point[0] * self.map_resolution + self.origin.position.x
        world_point.y = point[1] * self.map_resolution + self.origin.position.y
        world_point.z = 0.0
        return world_point
        
    def is_path_blocked(self):
        if not self.current_path or not self.obstacles:
            return False
            
        # Check if any obstacles intersect with path
        for pose in self.current_path.poses:
            for obstacle in self.obstacles:
                if self.check_collision(pose.pose.position, obstacle):
                    return True
        return False
        
    def check_collision(self, point, obstacle):
        # Simple distance-based collision check
        dx = point.x - obstacle.pose.position.x
        dy = point.y - obstacle.pose.position.y
        distance = np.sqrt(dx*dx + dy*dy)
        
        # Add obstacle inflation
        inflation = self.get_parameter('obstacle_inflation').value
        collision_threshold = max(
            obstacle.scale.x,
            obstacle.scale.y
        ) / 2.0 + inflation
        
        return distance < collision_threshold
        
    def get_current_pose(self):
        # TODO: Implement getting current pose from tf or other source
        return PoseStamped()

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 