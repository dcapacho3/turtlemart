#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Subscription to laser for obstacle detection
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        
        # Subscription to cmd_vel_out to monitor movement commands
        self.cmd_vel_out_subscription = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_vel_out_callback,
            10)
        
        # Publisher for avoidance commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_avoidance',
            10)

        self.last_cmd_vel_out_msg = Twist()
        self.last_cmd_vel_avoidance_msg = Twist()

        # Variable to control the publishing state
        self.publishing_enabled = False
        self.last_publishing_enabled = False

        # Variable to check if avoidance should be activated
        self.should_activate_avoidance = False

        # Flag to indicate if an avoidance routine is in progress
        self.avoidance_in_progress = False

        # Avoidance state machine
        self.STATES = {
            'IDLE': 0,
            'STOPPING': 1,
            'TURNING': 2,
            'MOVING': 3
        }
        self.current_state = self.STATES['IDLE']

        # Configuration parameters
        self.max_obstacle_distance = 0.25  # Maximum distance to consider obstacles
        self.min_obstacle_distance = 0.15  # Minimum distance to consider obstacles
        self.turn_angular_speed = 0.5  # Speed for turning
        self.forward_speed = 0.2  # Speed for forward movement
        
        # Angle thresholds for finding clear path (45 degrees in radians)
        self.angle_window = math.pi/4

    def cmd_vel_out_callback(self, msg):
        if (msg.linear.x != self.last_cmd_vel_avoidance_msg.linear.x or
            msg.linear.y != self.last_cmd_vel_avoidance_msg.linear.y or
            msg.angular.z != self.last_cmd_vel_avoidance_msg.angular.z):
            
            self.should_activate_avoidance = bool(msg.linear.x != 0.0 or 
                                                msg.linear.y != 0.0 or 
                                                msg.angular.z != 0.0)
        else:
            self.should_activate_avoidance = False

        self.last_cmd_vel_out_msg = msg

    def find_best_direction(self, msg):
        """Find the best direction to move based on laser scan data"""
        # Initialize variables for finding the largest gap
        largest_gap_angle = 0.0
        largest_gap_size = 0.0
        current_gap_start = None
        current_gap_size = 0
        
        # Process the scan data to find gaps
        for i, range_val in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Check if this point is clear of obstacles
            is_clear = range_val > self.max_obstacle_distance
            
            if is_clear:
                # If this is the start of a new gap
                if current_gap_start is None:
                    current_gap_start = angle
                current_gap_size += msg.angle_increment
            else:
                # If we were tracking a gap, check if it's the largest
                if current_gap_start is not None:
                    if current_gap_size > largest_gap_size:
                        largest_gap_size = current_gap_size
                        largest_gap_angle = current_gap_start + (current_gap_size / 2)
                    current_gap_start = None
                    current_gap_size = 0

        # Check the final gap if we ended while tracking one
        if current_gap_start is not None and current_gap_size > largest_gap_size:
            largest_gap_size = current_gap_size
            largest_gap_angle = current_gap_start + (current_gap_size / 2)

        return largest_gap_angle, largest_gap_size

    def is_path_clear(self, msg):
        """Check if the path ahead is clear of obstacles"""
        # Check a window of readings in front of the robot
        for range_val in msg.ranges:
            if self.min_obstacle_distance < range_val < self.max_obstacle_distance:
                return False
        return True

    def laser_callback(self, msg):
        if not self.should_activate_avoidance and not self.avoidance_in_progress:
            self.publishing_enabled = False
            self.current_state = self.STATES['IDLE']
            return

        # Check for obstacles within safety distance
        obstacle_detected = not self.is_path_clear(msg)

        # State machine
        if self.current_state == self.STATES['IDLE']:
            if obstacle_detected:
                self.publishing_enabled = True
                self.avoidance_in_progress = True
                self.current_state = self.STATES['STOPPING']
                self.publish_cmd_vel(0.0, 0.0, 0.0)  # Stop the robot
                self.get_logger().info('Obstacle detected - Stopping')

        elif self.current_state == self.STATES['STOPPING']:
            # Find best direction immediately using LIDAR data
            best_angle, gap_size = self.find_best_direction(msg)
            
            if gap_size > self.angle_window:  # If we found a suitable gap
                # Start turning towards the gap
                self.current_state = self.STATES['TURNING']
                turn_direction = 1 if best_angle > 0 else -1
                self.publish_cmd_vel(0.0, 0.0, self.turn_angular_speed * turn_direction)
                self.get_logger().info(f'Turning towards clear path at angle: {math.degrees(best_angle)}')
            else:
                # No suitable gap found, stay stopped
                self.publish_cmd_vel(0.0, 0.0, 0.0)
                self.get_logger().info('No clear path found - waiting')

        elif self.current_state == self.STATES['TURNING']:
            # Check if we've found a clear path ahead
            if self.is_path_clear(msg):
                self.current_state = self.STATES['MOVING']
                self.get_logger().info('Path clear - starting forward movement')
            else:
                # Continue turning
                best_angle, _ = self.find_best_direction(msg)
                turn_direction = 1 if best_angle > 0 else -1
                self.publish_cmd_vel(0.0, 0.0, self.turn_angular_speed * turn_direction)

        elif self.current_state == self.STATES['MOVING']:
            if obstacle_detected:
                # If new obstacle detected, restart avoidance
                self.current_state = self.STATES['STOPPING']
                self.publish_cmd_vel(0.0, 0.0, 0.0)
                self.get_logger().info('New obstacle detected - stopping')
            else:
                # Move forward if path is clear
                self.publish_cmd_vel(self.forward_speed, 0.0, 0.0)
                
                # Check if all directions are clear beyond max_obstacle_distance
                if all(r > self.max_obstacle_distance for r in msg.ranges):
                    self.publishing_enabled = False
                    self.avoidance_in_progress = False
                    self.current_state = self.STATES['IDLE']
                    self.get_logger().info('All clear - returning to normal operation')

    def publish_cmd_vel(self, linear_x, linear_y, angular_z):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.linear.y = linear_y
        cmd_vel_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.last_cmd_vel_avoidance_msg = cmd_vel_msg
        self.get_logger().info(f'Published cmd_vel: linear.x={linear_x}, angular.z={angular_z}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()