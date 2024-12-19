#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import numpy as np
from datetime import datetime
import sys
from scipy.spatial.distance import cdist
import os
import ament_index_python.packages

class Nav2MetricsNode(Node):
    def __init__(self):
        super().__init__('nav2_metrics_node')
        
        # Control de estado del nodo
        self._is_running = True
        self._shutdown_requested = False
        
        # Configuración
        self.COVARIANCE_WARNING_THRESHOLD = 0.5
        self.CRITICAL_COVARIANCE_THRESHOLD = 1.0
        self.PATH_TRACKING_SAMPLING_RATE = 5.0  # Hz
        
        # Variables para tiempo y velocidad
        self.total_navigation_time = 0.0  # Tiempo total acumulado
        self.current_navigation_start_time = None
        self.navigation_times = []  # Lista para almacenar tiempos de navegación
        self.velocity_samples = []  # Lista para muestras de velocidad
        self.last_position_time = None
        self.last_position = None
        self.velocity_sampling_period = 0.1  # 100ms para muestreo de velocidad
        
        # Callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Suscriptores
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10,
            callback_group=self.callback_group
        )

        self.feedback_sub = self.create_subscription(
            NavigateToPose_FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.feedback_callback,
            10,
            callback_group=self.callback_group
        )

        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Variables de estado
        self.current_pose = None
        self.current_pose_covariance = None
        self.goal_pose = None
        self.initial_pose = None
        self.navigation_results = []
        self.previous_status = None

        # Variables para tracking de trayectoria
        self.planned_path = None
        self.executed_path = []
        self.path_tracking_errors = []
        self.navigation_start_time = None
        self.current_navigation_id = 0

        # Variables para eficiencia de ruta
        self.planned_distance = 0.0
        self.executed_distance = 0.0
        self.last_executed_point = None
        self.initial_planned_distance = None
        self.total_replanned_distance = 0.0
        
        # Timer para muestreo de trayectoria
        self.create_timer(
            1.0/self.PATH_TRACKING_SAMPLING_RATE,
            self.path_tracking_callback,
            callback_group=self.callback_group
        )
        
        # Archivo de resultados
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.txt_filename = f'nav2_metrics_{timestamp}.txt'
        self.csv_filename = f'nav2_path_metrics_{timestamp}.csv'
        
        self.init_files()
        self.print_header()


    def init_files(self):
        """Inicializa archivos de registro en la carpeta debug_output"""
        try:
            # Ruta fija para los archivos de debug
            debug_output_dir = '/home/david/tesis/superdev_ws/src/turtlemart/debug_output'
            
            # Crear carpeta debug_output si no existe
            os.makedirs(debug_output_dir, exist_ok=True)
            
            # Generar nombres de archivo con timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.txt_filename = os.path.join(debug_output_dir, f'nav2_metrics_{timestamp}.txt')
            self.csv_filename = os.path.join(debug_output_dir, f'nav2_path_metrics_{timestamp}.csv')
            
            """Inicializa archivos de registro"""
            # Archivo TXT para resumen
            with open(self.txt_filename, 'w') as file:
                file.write("=== Registro de Navegación Nav2 (usando AMCL) ===\n\n")
                file.write(f"Fecha y hora de inicio: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            # Archivo CSV con columnas adicionales
            with open(self.csv_filename, 'w') as file:
                file.write("navigation_id,timestamp,planned_x,planned_y,executed_x,executed_y,"
                        "path_error,accumulated_distance,instantaneous_velocity,elapsed_time\n")
            
            # Registrar la ubicación de los archivos
            self.get_logger().info(f"Archivos de debug guardados en: {debug_output_dir}")
        
        except Exception as e:
            self.get_logger().error(f"Error al crear archivos de depuración: {e}")
            
    def print_header(self):
        """Imprime encabezado inicial"""
        print('\n=== Nav2 Metrics Node iniciado ===')
        print('- Usando AMCL para localización')
        print('- Monitoreando navegación y error de trayectoria')
        print('- Presiona Ctrl+C para finalizar y ver resultados\n')

    def calculate_path_length(self, path):
        """Calcula la longitud total de un camino"""
        if not path or len(path) < 2:
            return 0.0
            
        total_distance = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            total_distance += math.sqrt(dx*dx + dy*dy)
            
        return total_distance

    def plan_callback(self, msg):
        """Callback para el plan global"""
        if not self._is_running or self.current_navigation_id <= 0:
            return
        
        if msg.poses:
            self.planned_path = [(pose.pose.position.x, pose.pose.position.y) 
                               for pose in msg.poses]
            current_plan_distance = self.calculate_path_length(self.planned_path)
            
            if self.initial_planned_distance is None:
                self.initial_planned_distance = current_plan_distance
                self.write_and_print(f"\nPrimer plan recibido (Nav {self.current_navigation_id})")
                self.write_and_print(f"  Puntos: {len(self.planned_path)}")
                self.write_and_print(f"  Distancia planeada inicial: {current_plan_distance:.3f} metros")
            else:
                self.write_and_print(f"Plan actualizado (Nav {self.current_navigation_id}): {len(self.planned_path)} puntos, {current_plan_distance:.3f} metros")

    def path_tracking_callback(self):
        """Timer callback para muestreo de trayectoria y velocidad"""
        if not self._is_running or not self.current_pose:
            return

        current_time = self.get_node_time()
        current_point = (self.current_pose.position.x, self.current_pose.position.y)
        
        # Calcular velocidad instantánea
        if self.last_position and self.last_position_time:
            dt = current_time - self.last_position_time
            if dt >= self.velocity_sampling_period:  # Muestrear cada 100ms
                dx = current_point[0] - self.last_position[0]
                dy = current_point[1] - self.last_position[1]
                distance = math.sqrt(dx*dx + dy*dy)
                velocity = distance / dt
                self.velocity_samples.append(velocity)
                self.last_position = current_point
                self.last_position_time = current_time
        else:
            self.last_position = current_point
            self.last_position_time = current_time
        
        # Actualizar distancia ejecutada
        if self.last_executed_point:
            dx = current_point[0] - self.last_executed_point[0]
            dy = current_point[1] - self.last_executed_point[1]
            distance = math.sqrt(dx*dx + dy*dy)
            if distance > 0.0001:  # Umbral de 0.1mm
                self.executed_distance += distance
        
        self.last_executed_point = current_point
        self.executed_path.append(current_point)
        
        # Calcular error si hay un plan
        if self.planned_path:
            error = self.calculate_path_error(current_point)
            if error is not None:
                self.path_tracking_errors.append(error)
                
                # Registrar en CSV
                timestamp = self.get_node_time()
                planned_point = self.get_closest_planned_point(current_point)
                self.log_path_data(timestamp, planned_point, current_point, error)

    def calculate_path_error(self, point):
        """Calcula el error perpendicular al camino planeado"""
        if not self.planned_path or len(self.planned_path) < 2:
            return None

        p = np.array(point)
        min_distance = float('inf')

        for i in range(len(self.planned_path) - 1):
            a = np.array(self.planned_path[i])
            b = np.array(self.planned_path[i + 1])
            segment = b - a
            segment_length = np.linalg.norm(segment)

            if segment_length == 0:
                continue

            segment_normalized = segment / segment_length
            ap = p - a
            projection_length = np.dot(ap, segment_normalized)

            if 0 <= projection_length <= segment_length:
                projected_point = a + projection_length * segment_normalized
                distance = np.linalg.norm(p - projected_point)
            else:
                distance = min(np.linalg.norm(p - a), np.linalg.norm(p - b))

            min_distance = min(min_distance, distance)

        return min_distance

    def get_closest_planned_point(self, current_point):
        """Obtiene el punto más cercano en la ruta planeada"""
        if not self.planned_path:
            return None
            
        point_array = np.array([current_point])
        path_array = np.array(self.planned_path)
        distances = cdist(point_array, path_array).flatten()
        closest_idx = np.argmin(distances)
        
        return self.planned_path[closest_idx]

    def get_node_time(self):
        """Obtiene el tiempo actual del nodo"""
        return self.get_clock().now().nanoseconds / 1e9

    def log_path_data(self, timestamp, planned_point, executed_point, error):
        """Registra datos de trayectoria en CSV"""
        if planned_point is None:
            return
            
        current_velocity = 0.0
        if self.velocity_samples:
            current_velocity = self.velocity_samples[-1]
        elapsed_time = timestamp - self.current_navigation_start_time if self.current_navigation_start_time else 0.0
        
        with open(self.csv_filename, 'a') as file:
            file.write(f"{self.current_navigation_id},{timestamp:.3f},"
                      f"{planned_point[0]:.3f},{planned_point[1]:.3f},"
                      f"{executed_point[0]:.3f},{executed_point[1]:.3f},"
                      f"{error:.3f},{self.executed_distance:.3f},"
                      f"{current_velocity:.3f},{elapsed_time:.3f}\n")

    def status_callback(self, msg):
        """Callback para status de navegación"""
        if not self._is_running or not msg.status_list:
            return
            
        status = msg.status_list[-1].status
        if status != self.previous_status:
            status_messages = {
                1: "ACEPTADO",
                2: "EJECUTANDO",
                3: "CANCELADO",
                4: "COMPLETADO",
                5: "FALLIDO"
            }
            status_text = status_messages.get(status, f"DESCONOCIDO ({status})")
            
            if status == 2:  # EXECUTING
                self.write_and_print(f"\nEstado de navegación: {status_text}")
                self.start_new_navigation()
            else:
                self.write_and_print(f"\nEstado de navegación {self.current_navigation_id}: {status_text}")
                if status in [3, 4, 5]:  # CANCELED, SUCCEEDED, FAILED
                    self.complete_navigation()
            
            self.previous_status = status

    def start_new_navigation(self):
        """Inicia nueva navegación"""
        self.current_navigation_id += 1
        self.initial_pose = self.current_pose
        self.navigation_start_time = self.get_node_time()
        self.current_navigation_start_time = self.navigation_start_time
        self.executed_path = []
        self.path_tracking_errors = []
        self.planned_distance = 0.0
        self.executed_distance = 0.0
        self.last_executed_point = None
        self.velocity_samples = []
        self.initial_planned_distance = None
        self.write_and_print("Iniciando nueva navegación...")

    def complete_navigation(self):
        """Completa navegación actual"""
        if self.current_navigation_id > 0:
            navigation_time = self.get_node_time() - self.current_navigation_start_time
            self.navigation_times.append(navigation_time)
            self.total_navigation_time += navigation_time
            
            self.record_navigation_result()
            self.process_path_tracking_results()
            self.process_velocity_results()

    def process_velocity_results(self):
        """Procesa y registra resultados de velocidad"""
        if not self.velocity_samples:
            return

        velocities = np.array(self.velocity_samples)
        mean_velocity = np.mean(velocities)
        max_velocity = np.max(velocities)
        min_velocity = np.min(velocities)
        std_velocity = np.std(velocities)

        results = [
            f"\n=== Métricas de Velocidad (Navegación {self.current_navigation_id}) ===",
            f"Tiempo de navegación: {self.navigation_times[-1]:.2f} segundos",
            f"Velocidad media: {mean_velocity:.3f} m/s",
            f"Velocidad máxima: {max_velocity:.3f} m/s",
            f"Velocidad mínima: {min_velocity:.3f} m/s",
            f"Desviación estándar: {std_velocity:.3f} m/s",
            f"Muestras de velocidad: {len(self.velocity_samples)}"
        ]

        for line in results:
            self.write_and_print(line)

    def process_path_tracking_results(self):
        """Procesa resultados de tracking de trayectoria"""
        if not self.path_tracking_errors:
            return

        errors_array = np.array(self.path_tracking_errors)
        mean_error = np.mean(errors_array)
        std_error = np.std(errors_array)
        max_error = np.max(errors_array)
        
        # Calcular eficiencia de ruta
        path_efficiency = (self.initial_planned_distance / self.executed_distance * 100) if self.executed_distance > 0 and self.initial_planned_distance is not None else 0
        
        results = [
            f"\n=== Métricas de Trayectoria (Navegación {self.current_navigation_id}) ===",
            f"Error medio de trayectoria: {mean_error:.3f} metros",
            f"Desviación estándar: {std_error:.3f} metros",
            f"Error máximo: {max_error:.3f} metros",
            f"Puntos muestreados: {len(self.path_tracking_errors)}",
            f"\nEficiencia de Ruta:",
            f"  Distancia planeada inicial: {self.initial_planned_distance:.3f} metros",
            f"  Distancia ejecutada: {self.executed_distance:.3f} metros",
            f"  Eficiencia: {path_efficiency:.2f}% (100% = óptimo)\n"
        ]
        
        for line in results:
            self.write_and_print(line)

    def write_and_print(self, message, print_to_console=True):
        """Escribe en archivo y opcionalmente imprime en consola"""
        with open(self.txt_filename, 'a') as file:
            file.write(message + '\n')
        if print_to_console:
            print(message)

    def shutdown(self):
        """Manejo limpio del cierre del nodo"""
        if not self._shutdown_requested:
            self._shutdown_requested = True
            self._is_running = False
            self.write_and_print("\nProcesando resultados finales...")
            self.process_final_results()
            self.write_and_print("\nCierre completado.")

    def pose_callback(self, msg):
        """Callback para actualización de pose"""
        if not self._is_running:
            return
        self.current_pose = msg.pose.pose
        self.current_pose_covariance = msg.pose.covariance

    def feedback_callback(self, msg):
        """Callback para feedback de navegación"""
        if not self._is_running:
            return
        self.goal_pose = msg.feedback.current_pose.pose

    def record_navigation_result(self):
        """Registra resultado de navegación"""
        if not self.current_pose or not self.goal_pose:
            return

        dx = self.current_pose.position.x - self.goal_pose.position.x
        dy = self.current_pose.position.y - self.goal_pose.position.y
        radial_error = math.sqrt(dx*dx + dy*dy)

        # Calcular eficiencia y velocidad media
        efficiency = (self.initial_planned_distance / self.executed_distance * 100) if self.executed_distance > 0 and self.initial_planned_distance is not None else 0
        mean_velocity = np.mean(self.velocity_samples) if self.velocity_samples else 0
        navigation_time = self.navigation_times[-1] if self.navigation_times else 0

        result = {
            'id': len(self.navigation_results) + 1,
            'goal_x': self.goal_pose.position.x,
            'goal_y': self.goal_pose.position.y,
            'final_x': self.current_pose.position.x,
            'final_y': self.current_pose.position.y,
            'radial_error': radial_error,
            'planned_distance': self.initial_planned_distance,
            'executed_distance': self.executed_distance,
            'efficiency': efficiency,
            'path_tracking_errors': self.path_tracking_errors.copy(),
            'mean_tracking_error': np.mean(self.path_tracking_errors) if self.path_tracking_errors else 0,
            'std_tracking_error': np.std(self.path_tracking_errors) if self.path_tracking_errors else 0,
            'max_tracking_error': np.max(self.path_tracking_errors) if self.path_tracking_errors else 0,
            'navigation_time': navigation_time,
            'mean_velocity': mean_velocity,
            'max_velocity': np.max(self.velocity_samples) if self.velocity_samples else 0,
            'min_velocity': np.min(self.velocity_samples) if self.velocity_samples else 0,
            'velocity_std': np.std(self.velocity_samples) if self.velocity_samples else 0
        }

        self.navigation_results.append(result)
        
        # Mostrar resultado básico de la navegación
        self.write_and_print(f"\n=== Navegación {result['id']} Completada ===")
        self.write_and_print(f"Goal: [{result['goal_x']:.3f}, {result['goal_y']:.3f}]")
        self.write_and_print(f"Posición final: [{result['final_x']:.3f}, {result['final_y']:.3f}]")
        self.write_and_print(f"Error radial: {radial_error:.3f} metros")
        self.write_and_print(f"Tiempo de navegación: {navigation_time:.2f} segundos")
        self.write_and_print(f"Velocidad media: {mean_velocity:.3f} m/s")

    def process_final_results(self):
        """Procesamiento final expandido con métricas de tiempo y velocidad"""
        if not self.navigation_results:
            self.write_and_print("\nNo se registraron navegaciones.")
            return

        # Procesar errores radiales
        radial_errors = [result['radial_error'] for result in self.navigation_results]
        path_efficiencies = [result['efficiency'] for result in self.navigation_results]

        # Extraer métricas de trayectoria
        mean_tracking_errors = []
        max_tracking_errors = []
        std_tracking_errors = []
        mse_tracking_errors = []
        rmse_tracking_errors = []
        all_tracking_errors = []
        
        for result in self.navigation_results:
            if 'path_tracking_errors' in result and result['path_tracking_errors']:
                errors = np.array(result['path_tracking_errors'])
                mean_tracking_errors.append(np.mean(errors))
                max_tracking_errors.append(np.max(errors))
                std_tracking_errors.append(np.std(errors))
                all_tracking_errors.extend(errors)
                
                mse = np.mean(errors ** 2)
                rmse = np.sqrt(mse)
                mse_tracking_errors.append(mse)
                rmse_tracking_errors.append(rmse)

        # Procesar métricas de tiempo y velocidad
        navigation_times = [result['navigation_time'] for result in self.navigation_results]
        mean_velocities = [result['mean_velocity'] for result in self.navigation_results]
        max_velocities = [result['max_velocity'] for result in self.navigation_results]
        min_velocities = [result['min_velocity'] for result in self.navigation_results]
        velocity_stds = [result['velocity_std'] for result in self.navigation_results]

        results = [
            "\n============================",
            "=== RESULTADOS FINALES ===",
            "============================\n",
            f"Total de navegaciones: {len(self.navigation_results)}"
        ]

        if all_tracking_errors:
            all_errors = np.array(all_tracking_errors)
            global_mse = np.mean(all_errors ** 2)
            global_rmse = np.sqrt(global_mse)
            
            results.extend([
                "\nEstadísticas Globales de Error de Tracking:",
                f"  Error Medio Global: {np.mean(all_errors):.3f} metros",
                f"  MSE Global: {global_mse:.3f} metros²",
                f"  RMSE Global: {global_rmse:.3f} metros",
                f"  Desviación Estándar Global: {np.std(all_errors):.3f} metros",
                f"  Error Máximo Global: {np.max(all_errors):.3f} metros",
                f"  Error Mínimo Global: {np.min(all_errors):.3f} metros",
                f"  Total de muestras: {len(all_errors)}"
            ])

        if mean_tracking_errors:
            results.extend([
                "\nEstadísticas por Navegación:",
                f"  Error Medio Promedio: {np.mean(mean_tracking_errors):.3f} metros",
                f"  MSE Promedio: {np.mean(mse_tracking_errors):.3f} metros²",
                f"  RMSE Promedio: {np.mean(rmse_tracking_errors):.3f} metros",
                f"  RMSE Máximo: {np.max(rmse_tracking_errors):.3f} metros",
                f"  RMSE Mínimo: {np.min(rmse_tracking_errors):.3f} metros",
                f"  Error Máximo Global: {np.max(max_tracking_errors):.3f} metros",
                f"  Desviación Estándar Promedio: {np.mean(std_tracking_errors):.3f} metros"
            ])

        results.extend([
            "\nErrores Radiales:",
            f"  Promedio: {np.mean(radial_errors):.3f} metros",
            f"  Desviación estándar: {np.std(radial_errors):.3f} metros",
            f"  Máximo: {np.max(radial_errors):.3f} metros",
            f"  Mínimo: {np.min(radial_errors):.3f} metros"
        ])

        results.extend([
            "\nEficiencia de Ruta:",
            f"  Promedio: {np.mean(path_efficiencies):.2f}%",
            f"  Desviación estándar: {np.std(path_efficiencies):.2f}%",
            f"  Mejor: {np.min(path_efficiencies):.2f}%",
            f"  Peor: {np.max(path_efficiencies):.2f}%"
        ])

        results.extend([
            "\nEstadísticas de Tiempo:",
            f"  Tiempo total de navegación: {self.total_navigation_time:.2f} segundos",
            f"  Tiempo medio por navegación: {np.mean(navigation_times):.2f} segundos",
            f"  Tiempo máximo: {np.max(navigation_times):.2f} segundos",
            f"  Tiempo mínimo: {np.min(navigation_times):.2f} segundos"
        ])

        results.extend([
            "\nEstadísticas de Velocidad:",
            f"  Velocidad media global: {np.mean(mean_velocities):.3f} m/s",
            f"  Velocidad máxima alcanzada: {np.max(max_velocities):.3f} m/s",
            f"  Velocidad mínima registrada: {np.min(min_velocities):.3f} m/s",
            f"  Desviación estándar media: {np.mean(velocity_stds):.3f} m/s"
        ])

        results.append(f"\nDatos detallados guardados en: {self.csv_filename}")
        
        for line in results:
            self.write_and_print(line)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2MetricsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCierre solicitado por el usuario...")
    finally:
        node.shutdown()
        rclpy.try_shutdown()
        
    print(f"\nResultados guardados en:")
    print(f"- Resumen: {node.txt_filename}")
    print(f"- Datos de trayectoria: {node.csv_filename}")

if __name__ == '__main__':
    main()