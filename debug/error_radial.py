#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from debug_navigator import BasicNavigator, NavigationResult
import math
import matplotlib.pyplot as plt
import numpy as np
from tf_transformations import quaternion_from_euler

class PrecisionNavigator(Node):
    def __init__(self):
        super().__init__('precision_navigator')
        
        # Inicializar el navegador básico
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        # Suscriptor para la odometría
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Variables para almacenar datos
        self.current_pose = None
        self.arrival_points = []
        
        # Punto objetivo (puedes modificarlo según necesites)
        self.goal_point = {'x': 0.8, 'y': 0.3}  # Ejemplo de punto objetivo
        
    def odom_callback(self, msg):
        """Callback para actualizar la posición actual del robot"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

    def calculate_distance(self, point1, point2):
        """Calcula la distancia euclidiana entre dos puntos"""
        return math.sqrt((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)

    def calculate_goal_orientation(self, current_x, current_y, goal_x, goal_y):
        """Calcula la orientación hacia el objetivo"""
        dx = goal_x - current_x
        dy = goal_y - current_y
        return math.atan2(dy, dx)

    def create_goal_pose(self):
        """Crea el mensaje PoseStamped para el objetivo"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal_point['x']
        goal_pose.pose.position.y = self.goal_point['y']
        
        # Calcular orientación hacia el objetivo
        if self.current_pose:
            yaw = self.calculate_goal_orientation(
                self.current_pose['x'], 
                self.current_pose['y'],
                self.goal_point['x'],
                self.goal_point['y']
            )
            q = quaternion_from_euler(0, 0, yaw)
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]
        else:
            goal_pose.pose.orientation.w = 1.0
        
        return goal_pose

    def plot_results(self):
        """Genera la gráfica de resultados"""
        plt.figure(figsize=(10, 10))
        
        # Plotear punto objetivo
        plt.plot(self.goal_point['x'], self.goal_point['y'], 'r*', markersize=15, label='Objetivo')
        
        # Plotear puntos de llegada
        x_points = [p['x'] for p in self.arrival_points]
        y_points = [p['y'] for p in self.arrival_points]
        plt.plot(x_points, y_points, 'bo', label='Puntos de llegada')
        
        # Calcular error radial promedio
        distances = [self.calculate_distance(p, self.goal_point) for p in self.arrival_points]
        avg_error = sum(distances) / len(distances)
        
        # Dibujar círculo de error radial
        circle = plt.Circle((self.goal_point['x'], self.goal_point['y']), avg_error, 
                          fill=False, linestyle='--', color='gray', label=f'Error radial promedio: {avg_error:.3f}m')
        plt.gca().add_artist(circle)
        
        # Añadir estadísticas al título
        max_error = max(distances)
        min_error = min(distances)
        std_error = np.std(distances)
        plt.title(f'Análisis de Precisión de Navegación\n' + 
                 f'Error Promedio: {avg_error:.3f}m, Desv. Est: {std_error:.3f}m\n' +
                 f'Error Mín: {min_error:.3f}m, Error Máx: {max_error:.3f}m')
        
        plt.xlabel('X (metros)')
        plt.ylabel('Y (metros)')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')
        plt.show()

    def run_precision_test(self):
        """Ejecuta la prueba de precisión"""
        print("\nPunto objetivo establecido en: ")
        print(f"X: {self.goal_point['x']}, Y: {self.goal_point['y']}")
        
        while True:
            # Esperar confirmación inicial
            print("\nPresiona 'c' + Enter para comenzar una iteración o 'z' + Enter para terminar")
            command = input().strip().lower()
            
            if command == 'z':
                if len(self.arrival_points) > 0:
                    print("\nGenerando gráfica de resultados...")
                    self.plot_results()
                return
            elif command != 'c':
                print("Comando no válido. Use 'c' para continuar o 'z' para terminar")
                continue
            
            print("\nTeleoperando robot... Presiona 'c' + Enter cuando esté en posición")
            while True:
                if input().strip().lower() == 'c':
                    break
            
            print("\nIniciando navegación autónoma...")
            
            # Crear y enviar goal
            goal_pose = self.create_goal_pose()
            self.navigator.goToPose(goal_pose)
            
            # Esperar a que termine la navegación
            while not self.navigator.isNavComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # Verificar resultado
            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                print("\nNavegación exitosa")
                if self.current_pose:
                    self.arrival_points.append(self.current_pose.copy())
                    error = self.calculate_distance(self.current_pose, self.goal_point)
                    print(f"Error de llegada: {error:.3f} metros")
            else:
                print("\nLa navegación no fue exitosa")

def main(args=None):
    rclpy.init(args=args)
    navigator = PrecisionNavigator()
    
    try:
        navigator.run_precision_test()
    except KeyboardInterrupt:
        print("\nPrueba interrumpida por el usuario")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()