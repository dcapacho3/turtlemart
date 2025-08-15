#!/usr/bin/env python3
# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Publicador de posición inicial para el robot SARA
# Implementa un nodo ROS2 que publica un mensaje de posición inicial
# en el tópico '/initialpose'. Esto permite establecer la pose de partida
# del robot SARA en el mapa, facilitando la localización inicial en AMCL
# y otros algoritmos de navegación que requieren una estimación de pose inicial.

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class InitialPosePublisher(Node):
   def __init__(self):
       # Inicialización del nodo ROS
       # Configura el publicador para el tópico de pose inicial
       super().__init__('initial_pose_pub_node')
       self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

   def publish(self, x, y, theta):
       # Método para publicar la pose inicial
       # Crea y configura el mensaje con los valores de posición y orientación
       msg = PoseWithCovarianceStamped()
       msg.header.frame_id = 'map'
       msg.header.stamp = self.get_clock().now().to_msg()
       
       # Set position
       # Define las coordenadas cartesianas del robot en el mapa
       msg.pose.pose.position.x = x
       msg.pose.pose.position.y = y
       msg.pose.pose.position.z = 0.0 # Assuming the robot is on a flat plane
       
       # Set orientation (quaternion from yaw angle)
       # Convierte el ángulo de orientación (theta) a cuaternión
       msg.pose.pose.orientation.z = math.sin(theta / 2.0)
       msg.pose.pose.orientation.w = math.cos(theta / 2.0)
       
       # Log the published position
       # Registra en el log la información de la pose publicada
       self.get_logger().info(f'Publishing Initial Pose: X={x}, Y={y}, Theta={theta}')
       
       # Publish the message
       # Envía el mensaje al tópico para ser utilizado por el sistema de navegación
       self.publisher_.publish(msg)

def main(args=None):
   # Función principal del programa
   # Inicializa el nodo, publica la pose inicial y termina
   rclpy.init(args=args)
   publisher = InitialPosePublisher()
   

   # Valores predefinidos de posición inicial para el robot SARA
   x = 0.5
   y = -3.0
   theta = 1.58 # In radians (e.g., pi/2 for 90 degrees)
   
   # Publicar la pose inicial y procesar un ciclo de callbacks
   publisher.publish(x, y, theta)
   rclpy.spin_once(publisher) # Only publish once and stop
   
   # Limpieza de recursos
   publisher.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()