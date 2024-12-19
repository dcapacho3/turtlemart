#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class OdometryResetter(Node):
    def __init__(self):
        super().__init__('odometry_resetter')
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.reset_odometry()

    def reset_odometry(self):
        odom = Odometry()
        odom.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='odom')
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        self.publisher.publish(odom)
        self.get_logger().info('Odometry has been reset.')

def main(args=None):
    rclpy.init(args=args)
    node = OdometryResetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
