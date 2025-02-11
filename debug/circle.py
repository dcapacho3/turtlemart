#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
import time
import math

class CircularMotionController(Node):
    def __init__(self):
        super().__init__('circular_motion_controller')
        
        # Create publisher
        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', qos)
        
        # Robot limits
        self.MAX_LINEAR_SPEED = 0.22    # m/s
        self.MAX_ANGULAR_SPEED = 2.84   # rad/s
        
        # Calculate maximum possible radius at full speed
        # For a circle: radius = linear_speed / angular_speed
        self.max_possible_radius = self.MAX_LINEAR_SPEED / self.MAX_ANGULAR_SPEED
        
        # Movement parameters (optimized for larger circle)
        self.desired_radius = 0.25      # Desired radius in meters (increased for larger circle)
        
        # Calculate optimal speeds based on desired radius
        if self.desired_radius <= self.max_possible_radius:
            # We can achieve the exact radius
            self.linear_speed = self.MAX_LINEAR_SPEED
            self.angular_speed = self.linear_speed / self.desired_radius
            self.actual_radius = self.desired_radius
        else:
            # We'll use maximum linear speed and adjust angular speed
            # This will result in the largest possible circle while maintaining smooth motion
            self.linear_speed = self.MAX_LINEAR_SPEED
            self.angular_speed = self.linear_speed / self.desired_radius
            if self.angular_speed > self.MAX_ANGULAR_SPEED:
                # If angular speed would exceed limit, we adjust both speeds proportionally
                self.angular_speed = self.MAX_ANGULAR_SPEED
                self.linear_speed = self.angular_speed * self.desired_radius
                if self.linear_speed > self.MAX_LINEAR_SPEED:
                    self.linear_speed = self.MAX_LINEAR_SPEED
                    self.angular_speed = self.linear_speed / self.desired_radius
            self.actual_radius = self.linear_speed / self.angular_speed

        self.total_laps = 5             # Number of times to repeat the circular pattern
        
        # Initialize twist message
        self.twist = Twist()
        
        # Log configuration
        self.get_logger().info('Circular Motion Configuration:')
        self.get_logger().info(f'Desired Radius: {self.desired_radius:.3f} m')
        self.get_logger().info(f'Actual Radius: {self.actual_radius:.3f} m')
        self.get_logger().info(f'Linear Speed: {self.linear_speed:.3f} m/s')
        self.get_logger().info(f'Angular Speed: {self.angular_speed:.3f} rad/s')
        self.get_logger().info(f'Circle Diameter: {(self.actual_radius * 2):.3f} m')
        self.get_logger().info(f'Number of laps: {self.total_laps}')
        
        # Start the circular motion
        self.execute_circular_motion()

    def stop_robot(self):
        """Stop all robot movement"""
        self.twist = Twist()
        self.publisher.publish(self.twist)
        time.sleep(0.5)  # Brief pause between movements

    def execute_circular_motion(self):
        """Execute the circular motion pattern for the specified number of laps"""
        try:
            # Calculate time needed for one complete lap
            # Time = (2 * π * radius) / linear_speed
            lap_duration = (2 * math.pi * self.actual_radius) / self.linear_speed
            
            for lap in range(self.total_laps):
                self.get_logger().info(f'Starting lap {lap + 1} of {self.total_laps}')
                
                # Set constant linear and angular velocity for circular motion
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = self.angular_speed  # Positive for counter-clockwise motion
                
                # Publish the twist message
                self.publisher.publish(self.twist)
                
                # Wait for one complete lap
                time.sleep(lap_duration)
                
                self.get_logger().info(f'Lap {lap + 1} completed!')

            self.get_logger().info('All laps completed successfully!')

        except KeyboardInterrupt:
            self.get_logger().info('Motion interrupted by user')
        finally:
            self.stop_robot()
            self.get_logger().info('Motion stopped')

def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the node
    circular_motion_controller = CircularMotionController()
    
    try:
        rclpy.spin(circular_motion_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        circular_motion_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
