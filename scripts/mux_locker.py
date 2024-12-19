#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import ast

class TwistMuxController(Node):
    def __init__(self):
        super().__init__('twist_mux_controller')

        # Suscripciones y publicaciones
        self.subscription = self.create_subscription(
            String,
            'external_sensor_data',
            self.listener_callback,
            10)
        self.lock_all_publisher = self.create_publisher(Bool, 'lock_all', 10)
        self.lock_navigation_publisher = self.create_publisher(Bool, 'lock_navigation', 10)
        self.cmd_vel_block_all_publisher = self.create_publisher(Twist, 'cmd_vel_block_all', 10)
        self.cmd_vel_block_navigation_publisher = self.create_publisher(Twist, 'cmd_vel_block_navigation', 10)

        # Estados de los flags de bloqueo
        self.lock_all_active = False
        self.lock_navigation_active = False

        # Timer para publicar continuamente los mensajes de Twist
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def listener_callback(self, msg):
        # Parsear los datos recibidos
        data = ast.literal_eval(msg.data)

        if len(data) != 2:
            self.get_logger().warn('Received data does not contain exactly 2 numbers')
            return

        first_number, second_number = data

        # Condiciones de bloqueo
        self.lock_all_active = first_number > 300
        self.lock_navigation_active = second_number < 700 or second_number > 2000
        self.lock_navigation_active = False

        # Publicar el estado de los flags
        self.publish_lock_all(self.lock_all_active)
        self.publish_lock_navigation(self.lock_navigation_active)

    def publish_lock_all(self, lock):
        msg = Bool()
        msg.data = lock
        self.lock_all_publisher.publish(msg)
        self.get_logger().info(f'Published lock_all: {lock}')

    def publish_lock_navigation(self, lock):
        msg = Bool()
        msg.data = lock
        self.lock_navigation_publisher.publish(msg)
       # self.get_logger().info(f'Published lock_navigation: {lock}')

    def publish_velocity(self):
        # Definir el twist con base en los flags de bloqueo
        twist = Twist()
        if self.lock_all_active:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.y = 0.005  # Publicar mientras esté en bloqueo total
            self.cmd_vel_block_all_publisher.publish(twist)

        elif self.lock_navigation_active:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.y = 0.004  # Publicar mientras esté en bloqueo de navegación
            self.cmd_vel_block_navigation_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    twist_mux_controller = TwistMuxController()
    rclpy.spin(twist_mux_controller)
    twist_mux_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

