#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class teleop_node(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.timer_ = self.create_timer(0.1, self.publish_message)

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.msg = Twist()
        self.msg.linear.x = self.linear_x
        self.msg.linear.y = self.linear_y
        self.msg.angular.z = self.angular_z
        self.keyboard_input()

    def publish_message(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = self.linear_y
        msg.angular.z = self.angular_z
        self.publisher_.publish(msg)
        self.get_logger().info('Linear X: {:.2f}, Linear Y: {:.2f}, Angular Z: {:.2f}'.format(self.linear_x, self.linear_y, self.angular_z))

    def keyboard_input(self):
        while True:
            self.msg.linear.x = self.linear_x
            self.msg.linear.y = self.linear_y
            self.msg.angular.z = self.angular_z

            key = input("Enter a command (w/a/s/d/q/e): ")
            if key == 'w':
                self.linear_x = 0.5
                self.linear_y = 0.0
                self.angular_z = 0.0
            elif key == 'a':
                self.linear_x = 0.0
                self.linear_y = 0.5
                self.angular_z = 0.0
            elif key == 's':
                self.linear_x = -0.5
                self.linear_y = 0.0
                self.angular_z = 0.0
            elif key == 'd':
                self.linear_x = 0.0
                self.linear_y = -0.5
                self.angular_z = 0.0
            elif key == 'q':
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = 0.5
            elif key == 'e':
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = -0.5
            else:
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = 0.0

            self.publisher_.publish(self.msg)
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = teleop_node()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
