#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_gamepad', 10)
        
        # Speed control variables
        self.max_linear_speed = 0.22  # Maximum forward/backward speed
        self.max_angular_speed = 2.84 # Maximum rotation speed
        
        self.timeout_duration = 0.1
        self.debounce_duration = 0.2
        self.last_received_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)
        
        # Button B control variables
        self.button_b_pressed = False
        self.counter = 0
        self.last_button_b_state = 0
        self.last_button_b_change_time = self.get_clock().now()
        self.mode_normal = False
        self.mode_special = True
        
        # Publishing control
        self.publish_enabled = False
        self.is_joy_on_publisher = self.create_publisher(String, 'is_joy_on', 10)
        self.is_joy_on_msg = String()

    def joy_callback(self, msg):
        current_time = self.get_clock().now()
        
        # Button B handling for enabling/disabling publishing
        current_button_b_state = msg.buttons[1]
        time_since_last_change = (current_time - self.last_button_b_change_time).nanoseconds / 1e9
        
        if current_button_b_state != self.last_button_b_state and time_since_last_change > self.debounce_duration:
            self.last_button_b_change_time = current_time
            self.last_button_b_state = current_button_b_state

            if current_button_b_state == 1:
                if not self.button_b_pressed:
                    self.get_logger().info('Button B pressed')
                    self.button_b_pressed = True
                    self.counter += 1
                    if self.counter % 2 == 0:
                        self.get_logger().info('normal')
                        self.mode_normal = True
                        self.mode_special = False
                        self.publish_enabled = True
                    else:
                        self.get_logger().info('stopped')
                        self.mode_normal = False
                        self.mode_special = True
                        self.publish_enabled = False
            else:
                if self.button_b_pressed:
                    self.get_logger().info('Button B released')
                    self.button_b_pressed = False

        # Create Twist message
        twist = Twist()
        
        if self.mode_normal:
            self.is_joy_on_msg.data = 'yes'
            
            # Left stick for forward/backward movement (vertical axis)
            # Using axes[1] for vertical movement of left stick
            twist.linear.x = self.max_linear_speed * msg.axes[1]
            
            # Right stick for rotation (horizontal axis)
            # Using axes[3] for horizontal movement of right stick
            twist.angular.z = self.max_angular_speed * msg.axes[3]
            
            # Y movement is eliminated for differential drive
            twist.linear.y = 0.0

        elif self.mode_special:
            # In special mode, publish empty Twist to stop movement
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.publish_enabled = False
            self.is_joy_on_msg.data = 'no'

        self.is_joy_on_publisher.publish(self.is_joy_on_msg)
        
        # Publish Twist message only if enabled
        if self.publish_enabled:
            self.publisher.publish(twist)

        self.last_received_time = self.get_clock().now()

    def check_timeout(self):
        current_time = self.get_clock().now()
        time_since_last_msg = (current_time - self.last_received_time).nanoseconds / 1e9

        if time_since_last_msg > self.timeout_duration:
            if self.mode_normal:
                twist = Twist()
                if self.publish_enabled:
                    self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()