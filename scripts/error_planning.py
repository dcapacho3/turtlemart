#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from robot_navigator import BasicNavigator, NavigationResult
import math
import matplotlib.pyplot as plt
import numpy as np
from tf_transformations import quaternion_from_euler

class PathPrecisionNavigator(Node):
    def __init__(self):
        super().__init__('path_precision_navigator')
        
        # Radio del robot en metros
        self.robot_radius = 0.105  # 10.5 cm
        
        # Inicializar el navegador básico
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        # Suscriptores
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            10
        )
        
        self.path_subscriber = self.create_subscription(
            Path,
            'plan',
            self.path_callback,
            10
        )
        
        # Variables para almacenar datos
        self.current_pose = None
        self.current_iteration_plan = None  # Plan de la iteración actual
        self.waiting_for_plan = False  # Flag para controlar la captura del plan
        self.points = {
            'A': {'x': 0.15, 'y': 1.6},
            'B': {'x': 2.9, 'y': 0.15}
            #'A': {'x': 0.5, 'y': -2.6},
            #'B': {'x': -0.5, 'y': 1.0}
        }
        self.iterations_data = []  # Lista para almacenar datos de cada iteración
        self.current_trajectory = []  # Lista para almacenar la trayectoria actual
        
    def odom_callback(self, msg):
        """Callback para actualizar la posición actual del robot"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }
        if self.current_trajectory is not None:
            self.current_trajectory.append(self.current_pose.copy())

    def path_callback(self, msg):
        """Callback para guardar la ruta planeada solo cuando se está esperando un nuevo plan"""
        if self.waiting_for_plan and self.current_iteration_plan is None:
            self.current_iteration_plan = [(pose.pose.position.x, pose.pose.position.y) 
                                         for pose in msg.poses]
            self.waiting_for_plan = False
            print("Plan inicial capturado para esta iteración")

    def calculate_metrics(self, planned_path, actual_trajectory):
        """Calcula métricas de error entre la ruta planeada y la real"""
        if not planned_path or not actual_trajectory:
            return None

        # Convertir listas a arrays de numpy para cálculos más eficientes
        planned = np.array(planned_path)
        actual = np.array([(p['x'], p['y']) for p in actual_trajectory])

        # Error punto a punto más cercano
        errors = []
        for actual_point in actual:
            # Calcular distancias a todos los puntos planeados
            distances = np.sqrt(np.sum((planned - actual_point)**2, axis=1))
            errors.append(np.min(distances))

        metrics = {
            'mean_error': np.mean(errors),
            'max_error': np.max(errors),
            'min_error': np.min(errors),
            'rmse': np.sqrt(np.mean(np.array(errors)**2)),
            'std_error': np.std(errors)
        }
        
        return metrics

    def plot_results(self):
        """Genera la gráfica de resultados"""
        plt.figure(figsize=(15, 10))
        
        # Plotear puntos A y B
        plt.plot(self.points['A']['x'], self.points['A']['y'], 'g*', 
                markersize=15, label='Punto A')
        plt.plot(self.points['B']['x'], self.points['B']['y'], 'r*', 
                markersize=15, label='Punto B')
        
        # Encontrar límites máximos y mínimos de todas las trayectorias
        x_coords = []
        y_coords = []
        
        # Plotear cada iteración con un color diferente
        colors = plt.cm.rainbow(np.linspace(0, 1, len(self.iterations_data)))
        
        for i, (data, color) in enumerate(zip(self.iterations_data, colors)):
            if data['planned_path']:
                planned = np.array(data['planned_path'])
                plt.plot(planned[:, 0], planned[:, 1], '--', color=color, 
                        alpha=0.5, label=f'Plan #{i+1}: {data["target"]}')
                x_coords.extend(planned[:, 0])
                y_coords.extend(planned[:, 1])
            
            if data['actual_trajectory']:
                actual = np.array([(p['x'], p['y']) for p in data['actual_trajectory']])
                plt.plot(actual[:, 0], actual[:, 1], '-', color=color, 
                        alpha=1.0, label=f'Real #{i+1}: {data["target"]}')
                x_coords.extend(actual[:, 0])
                y_coords.extend(actual[:, 1])
        
        # Añadir anotaciones de métricas
        bbox_props = dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.8)
        for i, data in enumerate(self.iterations_data):
            if data['metrics']:
                metrics = data['metrics']
                plt.annotate(
                    f'Iter #{i+1} → {data["target"]}\n' +
                    f'Error medio: {metrics["mean_error"]:.3f}m\n' +
                    f'RMSE: {metrics["rmse"]:.3f}m',
                    xy=(0.02, 0.98 - i*0.1),
                    xycoords='axes fraction',
                    bbox=bbox_props,
                    fontsize=8
                )
        
        plt.title('Análisis de Precisión de Navegación')
        plt.xlabel('X (metros)')
        plt.ylabel('Y (metros)')
        plt.grid(True)
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.axis('equal')
        
        # Ajustar los límites de la gráfica
        if x_coords and y_coords:
            x_range = max(x_coords) - min(x_coords)
            y_range = max(y_coords) - min(y_coords)
            margin_x = x_range * 0.25
            margin_y = y_range * 0.25
            
            plt.xlim([min(x_coords) - margin_x, max(x_coords) + margin_x])
            plt.ylim([min(y_coords) - margin_y, max(y_coords) + margin_y])
        
        plt.tight_layout()
        plt.show()

    def initialize_position(self):
        """Inicializa la posición del robot en el punto A o B según sea necesario"""
        if not self.current_pose:
            print("Esperando datos de odometría...")
            while not self.current_pose:
                rclpy.spin_once(self, timeout_sec=0.1)
        
        # Calcular distancias a los puntos A y B
        dist_to_a = math.sqrt((self.current_pose['x'] - self.points['A']['x'])**2 + 
                            (self.current_pose['y'] - self.points['A']['y'])**2)
        dist_to_b = math.sqrt((self.current_pose['x'] - self.points['B']['x'])**2 + 
                            (self.current_pose['y'] - self.points['B']['y'])**2)
        
        # Determinar punto más cercano
        closest_point = 'A' if dist_to_a < dist_to_b else 'B'
        print(f"\nPosición actual - X: {self.current_pose['x']:.3f}, Y: {self.current_pose['y']:.3f}")
        print(f"Distancia a A: {dist_to_a:.3f}m, Distancia a B: {dist_to_b:.3f}m")
        print(f"Punto más cercano: {closest_point}")
        
        # Si está suficientemente cerca de algún punto, no necesita inicialización
        if min(dist_to_a, dist_to_b) < 0.1:
            print(f"Robot ya está cerca del punto {closest_point}")
            return closest_point
        
        # Si no, navegar al punto más cercano
        print(f"\nInicializando posición... Navegando al punto {closest_point}")
        
        # Preparar para capturar el plan inicial
        self.waiting_for_plan = True
        self.current_iteration_plan = None
        self.current_trajectory = []
        
        goal_pose = self.create_goal_pose(self.points[closest_point])
        self.navigator.goToPose(goal_pose)
        
        while not self.navigator.isNavComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print(f"Inicialización exitosa - Robot en punto {closest_point}")
            return closest_point
        else:
            print("Error en la inicialización")
            return None

    def create_goal_pose(self, target_point):
        """Crea el mensaje PoseStamped para el objetivo"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = target_point['x']
        goal_pose.pose.position.y = target_point['y']
        
        if self.current_pose:
            yaw = math.atan2(
                target_point['y'] - self.current_pose['y'],
                target_point['x'] - self.current_pose['x']
            )
            q = quaternion_from_euler(0, 0, yaw)
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]
        else:
            goal_pose.pose.orientation.w = 1.0
        
        return goal_pose

    def print_summary_statistics(self):
        """Imprime un resumen estadístico de todas las iteraciones"""
        if not self.iterations_data:
            print("No hay datos para generar estadísticas")
            return

        print("\n=== RESUMEN ESTADÍSTICO GENERAL ===")
        print("===================================")
        
        # Separar métricas por destino
        metrics_to_a = []
        metrics_to_b = []
        all_metrics = []
        
        for data in self.iterations_data:
            if data['metrics']:
                all_metrics.append(data['metrics'])
                if data['target'] == 'A':
                    metrics_to_a.append(data['metrics'])
                else:
                    metrics_to_b.append(data['metrics'])
        
        def calculate_stats(metrics_list, name):
            if not metrics_list:
                return
                
            mean_errors = [m['mean_error'] for m in metrics_list]
            rmse_values = [m['rmse'] for m in metrics_list]
            max_errors = [m['max_error'] for m in metrics_list]
            min_errors = [m['min_error'] for m in metrics_list]
            
            print(f"\n{name}:")
            print(f"  • Total de iteraciones: {len(metrics_list)}")
            print(f"  • Error promedio: {np.mean(mean_errors):.3f} ± {np.std(mean_errors):.3f} m")
            print(f"  • RMSE promedio: {np.mean(rmse_values):.3f} ± {np.std(rmse_values):.3f} m")
            print(f"  • Error máximo: {max(max_errors):.3f} m")
            print(f"  • Error mínimo: {min(min_errors):.3f} m")
        
        # Imprimir estadísticas globales y por destino
        calculate_stats(all_metrics, "ESTADÍSTICAS GLOBALES")
        calculate_stats(metrics_to_a, "Navegación hacia punto A")
        calculate_stats(metrics_to_b, "Navegación hacia punto B")
        
        print("\n===================================\n")

    def run_precision_test(self):
        """Ejecuta la prueba de precisión"""
        print("\nPuntos de navegación establecidos:")
        print(f"Punto A: X: {self.points['A']['x']}, Y: {self.points['A']['y']}")
        print(f"Punto B: X: {self.points['B']['x']}, Y: {self.points['B']['y']}")
        
        # Inicializar posición
        initial_point = self.initialize_position()
        if not initial_point:
            print("No se pudo inicializar la posición del robot")
            return
        
        iteration = 1
        current_target = 'B' if initial_point == 'A' else 'A'
        
        while True:
            print(f"\n[Iteración #{iteration}]")
            print("Presiona 'c' + Enter para comenzar una iteración o 'z' + Enter para terminar")
            command = input().strip().lower()
            
            if command == 'z':
                if self.iterations_data:
                    print("\nGenerando resumen estadístico...")
                    self.print_summary_statistics()
                    print("\nGenerando gráfica de resultados...")
                    self.plot_results()
                return
            elif command != 'c':
                print("Comando no válido. Use 'c' para continuar o 'z' para terminar")
                continue
            
            # Reiniciar datos para nueva iteración
            self.current_trajectory = []
            self.current_iteration_plan = None
            self.waiting_for_plan = True
            
            print(f"\nNavegando al punto {current_target}...")
            goal_pose = self.create_goal_pose(self.points[current_target])
            self.navigator.goToPose(goal_pose)
            
            # Esperar a que se reciba el plan inicial
            timeout = 0
            while self.current_iteration_plan is None and timeout < 50:
                rclpy.spin_once(self, timeout_sec=0.1)
                timeout += 1
            
            if self.current_iteration_plan is None:
                print("No se recibió el plan de navegación")
                continue
            
            # Esperar a que termine la navegación
            while not self.navigator.isNavComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            
            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                print("Navegación exitosa")
                
                # Calcular y guardar métricas
                metrics = self.calculate_metrics(
                    self.current_iteration_plan, 
                    self.current_trajectory
                )
                
                iteration_data = {
                    'iteration': iteration,
                    'target': current_target,
                    'planned_path': self.current_iteration_plan,
                    'actual_trajectory': self.current_trajectory,
                    'metrics': metrics
                }
                self.iterations_data.append(iteration_data)
                
                if metrics:
                    print(f"Error medio: {metrics['mean_error']:.3f} metros")
                    print(f"RMSE: {metrics['rmse']:.3f} metros")
                
                current_target = 'A' if current_target == 'B' else 'B'
                iteration += 1
            else:
                print("La navegación no fue exitosa")
                print(f"Resultado: {result}")

def main(args=None):
    rclpy.init(args=args)
    navigator = PathPrecisionNavigator()
    
    try:
        navigator.run_precision_test()
    except KeyboardInterrupt:
        print("\nPrueba interrumpida por el usuario")
    finally:
        navigator.navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()