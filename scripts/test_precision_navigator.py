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
import csv
from datetime import datetime

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
        plt.figure(figsize=(12, 8))
        
        # Plotear punto objetivo
        plt.plot(self.goal_point['x'], self.goal_point['y'], 'r*', markersize=15, label='Objetivo')
        
        # Plotear puntos de llegada con números de iteración y el círculo del robot
        x_coords = [data['pose']['x'] for data in self.iterations_data]
        y_coords = [data['pose']['y'] for data in self.iterations_data]
        plt.scatter(x_coords, y_coords, c='blue', marker='o', s=100)
        
        # Añadir círculos que representan el robot en cada punto
        for x, y in zip(x_coords, y_coords):
            robot_circle = plt.Circle((x, y), self.robot_radius, 
                                    fill=False, color='lightblue', 
                                    linestyle=':', alpha=0.5)
            plt.gca().add_artist(robot_circle)
        
        # Añadir círculo del robot en el objetivo para referencia
        target_robot_circle = plt.Circle((self.goal_point['x'], self.goal_point['y']), 
                                    self.robot_radius, 
                                    fill=False, 
                                    color='lightcoral', 
                                    linestyle=':', 
                                    alpha=0.5,
                                    label='Radio del robot')
        plt.gca().add_artist(target_robot_circle)
        
        # Añadir números de iteración
        for i, (x, y) in enumerate(zip(x_coords, y_coords), 1):
            plt.annotate(f'#{i}', (x, y), xytext=(5, 5), textcoords='offset points')
        
        # Calcular estadísticas de error (considerando que ya incluyen el ajuste del radio)
        distances = [data['error'] for data in self.iterations_data]
        avg_error = sum(distances) / len(distances)
        max_error = max(distances)
        min_error = min(distances)
        std_error = np.std(distances)
        
        # Dibujar círculo de error radial promedio (ya incluye el ajuste del radio)
        error_circle = plt.Circle((self.goal_point['x'], self.goal_point['y']), 
                                avg_error + self.robot_radius,  # Sumamos el radio aquí
                                fill=False, 
                                linestyle='--', 
                                color='gray', 
                                label=f'Error radial promedio: {avg_error:.3f}m')
        plt.gca().add_artist(error_circle)
        
        # Configurar título y etiquetas
        plt.title(f'Análisis de Precisión de Navegación\n' + 
                f'Total de iteraciones: {len(self.iterations_data)}\n' +
                f'Error Promedio: {avg_error:.3f}m, Desv. Est: {std_error:.3f}m\n' +
                f'Error Mín: {min_error:.3f}m, Error Máx: {max_error:.3f}m\n' +
                f'Radio del robot: {self.robot_radius:.3f}m')
        
        plt.xlabel('X (metros)')
        plt.ylabel('Y (metros)')
        plt.grid(True)
        plt.legend()
        
        # Ajustar los límites de la gráfica
        margin = max(max_error * 1.2, 0.3) + self.robot_radius  # Añadimos el radio al margen
        plt.xlim([
            min(min(x_coords + [self.goal_point['x']]) - margin,
                self.goal_point['x'] - margin),
            max(max(x_coords + [self.goal_point['x']]) + margin,
                self.goal_point['x'] + margin)
        ])
        plt.ylim([
            min(min(y_coords + [self.goal_point['y']]) - margin,
                self.goal_point['y'] - margin),
            max(max(y_coords + [self.goal_point['y']]) + margin,
                self.goal_point['y'] + margin)
        ])
        
        plt.axis('equal')
        plt.tight_layout()
        plt.show()

    def save_to_csv(self):
        """Guarda los datos de las iteraciones en un archivo CSV"""
        # Crear nombre de archivo con timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'precision_test_results_{timestamp}.csv'
        
        # Calcular estadísticas
        distances = [data['error'] for data in self.iterations_data]
        avg_error = sum(distances) / len(distances)
        max_error = max(distances)
        min_error = min(distances)
        std_error = np.std(distances)
        
        try:
            with open(filename, 'w', newline='') as csvfile:
                # Escribir metadatos
                writer = csv.writer(csvfile)
                writer.writerow(['Metadata'])
                writer.writerow(['Fecha y hora', datetime.now().strftime("%Y-%m-%d %H:%M:%S")])
                writer.writerow(['Punto objetivo X', self.goal_point['x']])
                writer.writerow(['Punto objetivo Y', self.goal_point['y']])
                writer.writerow(['Error promedio (m)', f'{avg_error:.3f}'])
                writer.writerow(['Desviación estándar (m)', f'{std_error:.3f}'])
                writer.writerow(['Error mínimo (m)', f'{min_error:.3f}'])
                writer.writerow(['Error máximo (m)', f'{max_error:.3f}'])
                writer.writerow([])  # Línea en blanco para separar

                # Escribir datos de iteraciones
                writer.writerow(['Datos de iteraciones'])
                writer.writerow(['Iteración', 'Posición X', 'Posición Y', 'Error (m)'])
                
                for data in self.iterations_data:
                    writer.writerow([
                        data['iteration'],
                        f"{data['pose']['x']:.3f}",
                        f"{data['pose']['y']:.3f}",
                        f"{data['error']:.3f}"
                    ])
            
            print(f"\nDatos exportados exitosamente a: {filename}")
            return filename
        except Exception as e:
            print(f"\nError al exportar datos: {str(e)}")
            return None

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
                    
                    # Guardar datos en CSV y obtener nombre del archivo
                    csv_filename = self.save_to_csv()
                    
                    # Imprimir resumen global
                    distances = [data['error'] for data in self.iterations_data]
                    avg_error = sum(distances) / len(distances)
                    max_error = max(distances)
                    min_error = min(distances)
                    std_error = np.std(distances)
                    
                    print("\nResumen Global:")
                    print("==============")
                    print(f"Total de iteraciones: {len(self.iterations_data)}")
                    print(f"Error promedio: {avg_error:.3f} metros")
                    print(f"Desviación estándar: {std_error:.3f} metros")
                    print(f"Error mínimo: {min_error:.3f} metros")
                    print(f"Error máximo: {max_error:.3f} metros")
                    
                    if csv_filename:
                        print(f"Datos guardados en: {csv_filename}")
                    
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