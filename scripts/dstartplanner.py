#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.srv import GetPlan
import numpy as np
from typing import List, Tuple, Dict, Set
from enum import Enum
import math
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import threading
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from heapq import heappush, heappop

class State(Enum):
    NEW = 0
    OPEN = 1
    CLOSED = 2

class DStarPlannerWithController(Node):
    def __init__(self):
        super().__init__('dstar_planner_controller')
        
        # ROS2 parameters for path planning
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_topic', '/global_costmap/costmap'),
                ('path_topic', '/planned_path'),
                ('odom_topic', '/odom'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('transform_tolerance', 0.1),
                ('min_lookahead_dist', 0.3),
                ('max_lookahead_dist', 0.9),
                ('lookahead_time', 1.5),
                ('rotate_to_heading_angular_vel', 1.8),
                ('max_angular_vel', 1.0),
                ('min_angular_vel', 0.0),
                ('max_linear_vel', 0.5),
                ('min_linear_vel', 0.0),
                ('use_velocity_scaled_lookahead_dist', True),
                ('regulated_linear_velocity_scaling_min_radius', 0.9),
                ('regulated_linear_velocity_scaling_gain', 2.0),
                ('use_regulated_linear_velocity_scaling', True),
                ('use_cost_regulated_linear_velocity_scaling', False),
                ('robot_radius', 0.3),
                ('transform_tolerance', 0.1),
                ('goal_tolerance', 0.25)
            ]
        )

        # Subscribe to costmap updates
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self.costmap_callback,
            10
        )
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10
        )
        
        # Publisher for the planned path
        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter('path_topic').value,
            10
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10
        )

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # D* specific attributes
        self.costmap = None
        self.resolution = 0.05
        self.origin = None
        self.k_m = 0
        self.states: Dict[Tuple[int, int], dict] = {}
        self.goal = None
        self.start = None
        
        # Controller specific attributes
        self.current_pose = None
        self.current_velocity = None
        self.path = None
        self.goal_reached = False
        self.control_thread = None
        self.running = True

        # Start control loop
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

        self.get_logger().info('D* Planner with Regulated Pure Pursuit Controller initialized')

    def regulate_linear_velocity(self, linear_vel: float, curvature: float) -> float:
        """Regulate linear velocity based on curvature"""
        if not self.get_parameter('use_regulated_linear_velocity_scaling').value:
            return linear_vel

        scaling_factor = 1.0
        min_radius = self.get_parameter('regulated_linear_velocity_scaling_min_radius').value
        scaling_gain = self.get_parameter('regulated_linear_velocity_scaling_gain').value

        if abs(curvature) > 0.001:
            radius = 1.0 / abs(curvature)
            if radius < min_radius:
                scaling_factor = radius / min_radius
            else:
                scaling_factor = 1.0 - scaling_gain * (min_radius / radius)
                scaling_factor = max(scaling_factor, 0.1)

        return linear_vel * scaling_factor

    def get_lookahead_point(self, path: Path, current_pose: PoseStamped) -> Tuple[Point, float]:
        """Get lookahead point and distance along the path"""
        min_lookahead = self.get_parameter('min_lookahead_dist').value
        max_lookahead = self.get_parameter('max_lookahead_dist').value
        
        if len(path.poses) < 2:
            return None, 0.0

        # Calculate adaptive lookahead distance based on velocity
        if self.get_parameter('use_velocity_scaled_lookahead_dist').value and self.current_velocity is not None:
            lookahead_dist = self.current_velocity.linear.x * self.get_parameter('lookahead_time').value
            lookahead_dist = max(min_lookahead, min(max_lookahead, lookahead_dist))
        else:
            lookahead_dist = min_lookahead

        # Find the closest point on path
        closest_point_idx = 0
        min_dist = float('inf')
        
        for i, pose in enumerate(path.poses):
            dist = math.sqrt(
                (pose.pose.position.x - current_pose.pose.position.x) ** 2 +
                (pose.pose.position.y - current_pose.pose.position.y) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                closest_point_idx = i

        # Look for furthest point within lookahead distance
        lookahead_point = None
        for i in range(closest_point_idx, len(path.poses)):
            dist = math.sqrt(
                (path.poses[i].pose.position.x - current_pose.pose.position.x) ** 2 +
                (path.poses[i].pose.position.y - current_pose.pose.position.y) ** 2
            )
            if dist >= lookahead_dist:
                lookahead_point = path.poses[i].pose.position
                break

        if lookahead_point is None and len(path.poses) > 0:
            lookahead_point = path.poses[-1].pose.position
            
        return lookahead_point, lookahead_dist

    def calculate_velocity_commands(self, lookahead_point: Point, current_pose: PoseStamped) -> Twist:
        """Calculate velocity commands using regulated pure pursuit"""
        cmd_vel = Twist()
        
        if lookahead_point is None:
            return cmd_vel

        # Transform lookahead point to robot frame
        dx = lookahead_point.x - current_pose.pose.position.x
        dy = lookahead_point.y - current_pose.pose.position.y
        
        # Get robot's heading
        q = current_pose.pose.orientation
        heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Calculate angle to goal point
        goal_heading = math.atan2(dy, dx)
        angle_to_goal = goal_heading - heading
        
        # Normalize angle
        while angle_to_goal > math.pi:
            angle_to_goal -= 2.0 * math.pi
        while angle_to_goal < -math.pi:
            angle_to_goal += 2.0 * math.pi

        # Calculate curvature
        lateral_error = math.sqrt(dx * dx + dy * dy) * math.sin(angle_to_goal)
        lookahead_dist = math.sqrt(dx * dx + dy * dy)
        curvature = 2.0 * lateral_error / (lookahead_dist * lookahead_dist)

        # Calculate angular velocity
        angular_vel = curvature * self.get_parameter('max_linear_vel').value
        angular_vel = max(min(angular_vel, self.get_parameter('max_angular_vel').value),
                        -self.get_parameter('max_angular_vel').value)

        # Calculate linear velocity
        linear_vel = self.get_parameter('max_linear_vel').value
        linear_vel = self.regulate_linear_velocity(linear_vel, curvature)
        
        # Set velocity commands
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        
        return cmd_vel

    def control_loop(self):
        """Main control loop"""
        rate = self.create_rate(20)  # 20Hz control loop
        
        while rclpy.ok() and self.running:
            if self.path is not None and self.current_pose is not None and not self.goal_reached:
                # Get lookahead point
                lookahead_point, _ = self.get_lookahead_point(self.path, self.current_pose)
                
                if lookahead_point is not None:
                    # Calculate velocity commands
                    cmd_vel = self.calculate_velocity_commands(lookahead_point, self.current_pose)
                    
                    # Check if goal reached
                    dist_to_goal = math.sqrt(
                        (self.path.poses[-1].pose.position.x - self.current_pose.pose.position.x) ** 2 +
                        (self.path.poses[-1].pose.position.y - self.current_pose.pose.position.y) ** 2
                    )
                    
                    if dist_to_goal < self.get_parameter('goal_tolerance').value:
                        self.goal_reached = True
                        cmd_vel = Twist()  # Stop the robot
                    
                    # Publish velocity commands
                    self.cmd_vel_pub.publish(cmd_vel)
            
            rate.sleep()

    def odom_callback(self, msg: Odometry):
        """Handle odometry updates"""
        self.current_velocity = msg.twist.twist
        
        # Convert odometry to PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose



    def costmap_callback(self, msg: OccupancyGrid):
        """Handle updates to the costmap"""
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        # Convert costmap to numpy array
        width = msg.info.width
        height = msg.info.height
        self.costmap = np.array(msg.data).reshape((height, width))
        
        # If we have a current path, check if we need to replan
        if self.start is not None and self.goal is not None:
            self.update_path()

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x - self.origin[0]) / self.resolution)
        grid_y = int((y - self.origin[1]) / self.resolution)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        world_x = grid_x * self.resolution + self.origin[0]
        world_y = grid_y * self.resolution + self.origin[1]
        return (world_x, world_y)

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells"""
        x, y = pos
        neighbors = []
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]:
            new_x, new_y = x + dx, y + dy
            if (0 <= new_x < self.costmap.shape[1] and 
                0 <= new_y < self.costmap.shape[0] and
                self.costmap[new_y, new_x] < 100):  # Not completely occupied
                neighbors.append((new_x, new_y))
        return neighbors

    def h_cost(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Heuristic cost estimate between two points"""
        return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def cost(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """True cost between neighboring cells"""
        base_cost = math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
        obstacle_cost = self.costmap[b[1], b[0]] / 100.0  # Normalize cost
        return base_cost * (1 + obstacle_cost)

    def initialize_state(self, pos: Tuple[int, int]):
        """Initialize or reset the state of a cell"""
        self.states[pos] = {
            'g': float('inf'),
            'rhs': float('inf'),
            'state': State.NEW
        }

    def calculate_key(self, pos: Tuple[int, int]) -> Tuple[float, float]:
        """Calculate priority key for a state"""
        if pos not in self.states:
            self.initialize_state(pos)
        
        g = self.states[pos]['g']
        rhs = self.states[pos]['rhs']
        h = self.h_cost(pos, self.goal)
        
        k1 = min(g, rhs) + h + self.k_m
        k2 = min(g, rhs)
        
        return (k1, k2)

    def update_vertex(self, u: Tuple[int, int], open_list: List):
        """Update vertex u and its cost"""
        if u != self.goal:
            # Update rhs
            min_cost = float('inf')
            for successor in self.get_neighbors(u):
                new_cost = self.states[successor]['g'] + self.cost(u, successor)
                min_cost = min(min_cost, new_cost)
            self.states[u]['rhs'] = min_cost

        # Remove u from open list if it exists
        self.states[u]['state'] = State.NEW

        # Add back to open list if inconsistent
        if self.states[u]['g'] != self.states[u]['rhs']:
            heappush(open_list, (self.calculate_key(u), u))
            self.states[u]['state'] = State.OPEN

    def compute_shortest_path(self):
        """Compute or update the shortest path using D*"""
        open_list = []
        
        # Initialize if necessary
        if self.goal not in self.states:
            self.initialize_state(self.goal)
            self.states[self.goal]['rhs'] = 0
            heappush(open_list, (self.calculate_key(self.goal), self.goal))
            self.states[self.goal]['state'] = State.OPEN

        while (len(open_list) > 0 and 
               (open_list[0][0] < self.calculate_key(self.start) or 
                self.states[self.start]['rhs'] != self.states[self.start]['g'])):
            k_old, u = heappop(open_list)
            
            if k_old < self.calculate_key(u):
                heappush(open_list, (self.calculate_key(u), u))
            elif self.states[u]['g'] > self.states[u]['rhs']:
                self.states[u]['g'] = self.states[u]['rhs']
                self.states[u]['state'] = State.CLOSED
                for predecessor in self.get_neighbors(u):
                    self.update_vertex(predecessor, open_list)
            else:
                self.states[u]['g'] = float('inf')
                self.update_vertex(u, open_list)
                for predecessor in self.get_neighbors(u):
                    self.update_vertex(predecessor, open_list)

    def extract_path(self) -> List[Tuple[float, float]]:
        """Extract the path from start to goal"""
        if self.states[self.start]['g'] == float('inf'):
            return []  # No path exists
            
        path = [self.start]
        current = self.start
        
        while current != self.goal:
            min_cost = float('inf')
            next_pos = None
            
            for neighbor in self.get_neighbors(current):
                cost = self.cost(current, neighbor) + self.states[neighbor]['g']
                if cost < min_cost:
                    min_cost = cost
                    next_pos = neighbor
                    
            if next_pos is None:
                return []  # No path exists
                
            path.append(next_pos)
            current = next_pos
            
        return path

    def create_path_msg(self, path: List[Tuple[int, int]]) -> Path:
        """Convert path to ROS2 Path message"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for grid_pos in path:
            world_pos = self.grid_to_world(*grid_pos)
            pose = PoseStamped()
            pose.pose.position.x = world_pos[0]
            pose.pose.position.y = world_pos[1]
            pose.pose.position.z = 0.0
            pose.header = path_msg.header
            path_msg.poses.append(pose)
            
        return path_msg

   

    def update_path(self):
        """Update path when costmap changes"""
        if self.start is None or self.goal is None:
            return
            
        # Increment k_m to maintain consistency
        self.k_m += self.h_cost(self.start, self.goal)
        
        # Recompute path
        self.compute_shortest_path()
        path = self.extract_path()
        
        # Publish updated path
        path_msg = self.create_path_msg(path)
        self.path_pub.publish(path_msg)

 

    def plan_path_callback(self, request, response):
        """Service callback for path planning requests"""
        if self.costmap is None:
            self.get_logger().warn('No costmap received yet')
            return response
            
        # Reset goal reached flag
        self.goal_reached = False
# Convert start and goal to grid coordinates
        self.start = self.world_to_grid(
            request.start.pose.position.x,
            request.start.pose.position.y
        )
        self.goal = self.world_to_grid(
            request.goal.pose.position.x,
            request.goal.pose.position.y
        )
        
        # Initialize states if necessary
        if self.start not in self.states:
            self.initialize_state(self.start)
        if self.goal not in self.states:
            self.initialize_state(self.goal)
            self.states[self.goal]['rhs'] = 0
            
        # Compute path
        self.compute_shortest_path()
        path = self.extract_path()
        
        # Convert to ROS message and publish
        path_msg = self.create_path_msg(path)
        self.path_pub.publish(path_msg)
        response.plan = path_msg

        self.path = response.plan
        
        return response
    
    def destroy_node(self):
        """Cleanup when node is shut down"""
        self.running = False
        if self.control_thread is not None:
            self.control_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DStarPlannerWithController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()