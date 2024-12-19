#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_pub_node')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
       
    def publish(self, x, y, theta):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0  # Assuming the robot is on a flat plane
        
        # Set orientation (quaternion from yaw angle)
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Log the published position
        self.get_logger().info(f'Publishing Initial Pose: X={x}, Y={y}, Theta={theta}')

        # Publish the message
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    publisher = InitialPosePublisher()

    # Example values (replace with dynamic values as needed)
    x = 1.0
    y = -3.0
    theta = 1.58  # In radians (e.g., pi/2 for 90 degrees)

    publisher.publish(x, y, theta)
    
    rclpy.spin_once(publisher)  # Only publish once and stop
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

