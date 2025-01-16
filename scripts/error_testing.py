#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult
import math
import matplotlib.pyplot as plt
import numpy as np
from tf_transformations import quaternion_from_euler

class PrecisionNavigator(Node):
    def __init__(self):
        super().__init__('precision_navigator')
        
        # Radio del robot en metros
        self.robot_radius = 0.105  # 10.5 cm
        
        # Inicializar el navegador básico
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        # Suscriptor para la odometría
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Variables para almacenar datos
        self.current_pose = None
        self.arrival_points = []  # Lista de puntos de llegada
        self.iterations_data = []  # Lista para almacenar datos de cada iteración
        
        # Punto objetivo (puedes modificarlo según necesites)
        self.goal_point = {'x': 1.85, 'y': 0.57}  # Ejemplo de punto objetivo
        
    def odom_callback(self, msg):
        """Callback para actualizar la posición actual del robot"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

    def calculate_distance(self, point1, point2):
        """Calcula la distancia euclidiana entre dos puntos considerando el radio del robot"""
        raw_distance = math.sqrt((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)
        # Restamos el radio del robot para obtener la distancia real desde el borde del robot
        adjusted_distance = max(0, raw_distance - self.robot_radius)
        return adjusted_distance

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
        
        # Plotear puntos de llegada con números de iteración
        for i, data in enumerate(self.iterations_data, 1):
            plt.plot(data['pose']['x'], data['pose']['y'], 'bo')
            plt.annotate(f'#{i}', 
                        (data['pose']['x'], data['pose']['y']),
                        xytext=(5, 5), textcoords='offset points')
        
        # Calcular error radial promedio
        distances = [data['error'] for data in self.iterations_data]
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
                 f'Total de iteraciones: {len(self.iterations_data)}\n' +
                 f'Error Promedio: {avg_error:.3f}m, Desv. Est: {std_error:.3f}m\n' +
                 f'Error Mín: {min_error:.3f}m, Error Máx: {max_error:.3f}m')
        
        plt.xlabel('X (metros)')
        plt.ylabel('Y (metros)')
        plt.grid(True)
        plt.legend()
        # Ajustar los límites de la gráfica basados en el punto objetivo y el error radial
        margin = max_error * 1.5  # Margen adicional más allá del punto más lejano
        
        plt.xlim([
            min(self.goal_point['x'] - margin, min(data['pose']['x'] for data in self.iterations_data)),
            max(self.goal_point['x'] + margin, max(data['pose']['x'] for data in self.iterations_data))
        ])
        plt.ylim([
            min(self.goal_point['y'] - margin, min(data['pose']['y'] for data in self.iterations_data)),
            max(self.goal_point['y'] + margin, max(data['pose']['y'] for data in self.iterations_data))
        ])
        
        plt.axis('equal')
        plt.show()

    def print_iteration_summary(self):
        """Imprime un resumen de todas las iteraciones"""
        print("\nResumen de iteraciones:")
        print("----------------------")
        for i, data in enumerate(self.iterations_data, 1):
            print(f"Iteración #{i}:")
            print(f"  Posición: (x: {data['pose']['x']:.3f}, y: {data['pose']['y']:.3f})")
            print(f"  Error: {data['error']:.3f} metros")
        print("----------------------")

    def run_precision_test(self):
        """Ejecuta la prueba de precisión"""
        print("\nPunto objetivo establecido en: ")
        print(f"X: {self.goal_point['x']}, Y: {self.goal_point['y']}")
        
        iteration = 1
        while True:
            # Esperar confirmación inicial
            print(f"\n[Iteración #{iteration}]")
            print("Presiona 'c' + Enter para comenzar una iteración o 'z' + Enter para terminar")
            command = input().strip().lower()
            
            if command == 'z':
                if len(self.iterations_data) > 0:
                    print("\nResumen final:")
                    self.print_iteration_summary()
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
                    error = self.calculate_distance(self.current_pose, self.goal_point)
                    
                    # Guardar datos de la iteración
                    iteration_data = {
                        'iteration': iteration,
                        'pose': self.current_pose.copy(),
                        'error': error
                    }
                    self.iterations_data.append(iteration_data)
                    
                    print(f"Error de llegada: {error:.3f} metros")
                    iteration += 1
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