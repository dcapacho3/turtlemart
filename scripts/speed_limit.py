#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistLimiter(Node):
    def __init__(self):
        super().__init__('twist_limiter')
        
        # Define maximum linear and angular speeds as variables
        self.MAX_LINEAR_SPEED = 0.15
        self.MAX_ANGULAR_SPEED = 1.82

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_in',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_out', 10)

    def listener_callback(self, msg):
        limited_msg = Twist()
        
        # Limit linear velocity using MAX_LINEAR_SPEED
        limited_msg.linear.x = max(min(msg.linear.x, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        limited_msg.linear.y = max(min(msg.linear.y, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        limited_msg.linear.z = max(min(msg.linear.z, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        
        # Limit angular velocity using MAX_ANGULAR_SPEED
        limited_msg.angular.x = max(min(msg.angular.x, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        limited_msg.angular.y = max(min(msg.angular.y, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        limited_msg.angular.z = max(min(msg.angular.z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        
        self.publisher.publish(limited_msg)

def main(args=None):
    rclpy.init(args=args)
    twist_limiter = TwistLimiter()
    rclpy.spin(twist_limiter)
    twist_limiter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
