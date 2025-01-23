#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from itertools import permutations
import math
from tf_transformations import euler_from_quaternion
import yaml
from PIL import Image
import os
from ament_index_python.packages import get_package_share_directory
from scipy.signal import savgol_filter

class RouteAnalysisNode(Node):
    def __init__(self):
        super().__init__('route_analysis_node')
        
        # Configuración básica
        self.db_path = 'src/turtlemart/database/products.db'
        
        #self.fixed_cash_location = {'name': 'cashier', 'x': -1.0, 'y': -2.0} #sim
        self.fixed_cash_location = {'name': 'cashier', 'x': 0.1, 'y': 2.0}
        
        # Variables para seguimiento
        self.current_path = []
        self.smoothed_path = []
        self.is_navigating = False
        self.start_pose = None
        self.all_possible_routes = []
        self.initial_pose_received = False
        self.amcl_initialized = False
        
        # Cargar mapa
        self.load_map()
        
        # Suscriptores
        self.create_subscription(
            Odometry, 
            'odometry/filtered', 
            self.odom_callback, 
            10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            10
        )
        self.create_subscription(
            String, 
            '/navigation_status', 
            self.status_callback, 
            10
        )
        
        # La visualización se iniciará después de recibir la posición inicial
        self.fig = None
        self.ax = None
        self.update_timer = None
        
        self.get_logger().info('Esperando inicialización de AMCL...')

    def load_map(self):
        """Cargar el mapa desde el archivo YAML"""
        bringup_dir = get_package_share_directory('turtlemart')
       # map_yaml_path = os.path.join(bringup_dir, 'maps/supermarket_map.yaml')
        map_yaml_path = os.path.join(bringup_dir, 'maps/labrobfinal_mask.yaml')
        
        with open(map_yaml_path, 'r') as f:
            yaml_content = yaml.safe_load(f)

        self.map_resolution = yaml_content['resolution']
        self.map_origin = yaml_content['origin']
        map_image_path = os.path.join(os.path.dirname(map_yaml_path), yaml_content['image'])
        
        # Cargar y procesar la imagen del mapa
        map_image = Image.open(map_image_path)
        self.map_array = np.array(map_image)
        if len(self.map_array.shape) > 2:  # Si es RGB, convertir a escala de grises
            self.map_array = np.mean(self.map_array, axis=2)
        
        # Invertir el mapa para que coincida con las coordenadas ROS
        self.map_array = np.flipud(self.map_array)

    def world_to_pixel(self, x, y):
        """Convertir coordenadas del mundo a píxeles del mapa"""
        pixel_x = int((x - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((y - self.map_origin[1]) / self.map_resolution)
        return pixel_x, pixel_y

    def initialize_visualization(self):
        """Inicializar la visualización una vez que tengamos la posición inicial"""
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title('Análisis de Rutas en Tiempo Real')
        
        # Mostrar el mapa como fondo
        self.ax.imshow(self.map_array, cmap='gray', origin='lower', 
                      extent=[self.map_origin[0], 
                             self.map_origin[0] + self.map_array.shape[1] * self.map_resolution,
                             self.map_origin[1], 
                             self.map_origin[1] + self.map_array.shape[0] * self.map_resolution])
        
        self.update_timer = self.create_timer(1.0, self.update_plot)

    def smooth_path(self, path, window=5):
        """Suavizar la ruta usando Savitzky-Golay filter"""
        if len(path) < window:
            return path
            
        x_coords = [p['x'] for p in path]
        y_coords = [p['y'] for p in path]
        
        # Usar Savitzky-Golay filter para suavizar
        if len(x_coords) > window:
            x_smooth = savgol_filter(x_coords, window, 3)
            y_smooth = savgol_filter(y_coords, window, 3)
        else:
            x_smooth = x_coords
            y_smooth = y_coords
            
        return [{'x': x, 'y': y} for x, y in zip(x_smooth, y_smooth)]

    def get_product_locations(self):
        """Obtener ubicaciones de productos desde la base de datos"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('SELECT name, x, y FROM selected_products')
        locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
        conn.close()
        locations.append(self.fixed_cash_location)
        return locations

    def calculate_distance(self, point1, point2):
        """Calcular distancia euclidiana entre dos puntos"""
        return math.sqrt((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)

    def calculate_route_distance(self, route):
        """Calcular distancia total de una ruta"""
        return sum(self.calculate_distance(route[i], route[i + 1]) for i in range(len(route) - 1))

    def analyze_all_possible_routes(self):
        """Analizar todas las rutas posibles y sus distancias"""
        if not self.initial_pose_received:
            return

        locations = self.get_product_locations()
        intermediate_points = [loc for loc in locations if loc['name'] != 'cashier']
        
        print("\nAnalizando todas las rutas posibles desde posición inicial:")
        print(f"x: {self.start_pose['x']:.2f}, y: {self.start_pose['y']:.2f}")
        print("=====================================")
        
        # Generar todas las permutaciones posibles
        all_routes = []
        for perm in permutations(intermediate_points):
            route = [self.start_pose] + list(perm) + [self.fixed_cash_location]
            distance = self.calculate_route_distance(route)
            all_routes.append({
                'route': route,
                'distance': distance,
                'waypoints': ['start'] + [point['name'] for point in perm] + ['cashier']
            })

        # Ordenar rutas por distancia
        self.all_possible_routes = sorted(all_routes, key=lambda x: x['distance'])
        
        # Mostrar análisis de rutas
        print(f"\nTotal de rutas posibles: {len(self.all_possible_routes)}")
        print("\nMejores 5 rutas:")
        print("---------------")
        for i, route in enumerate(self.all_possible_routes[:5], 1):
            print(f"\nRuta #{i}")
            print(f"Secuencia: {' -> '.join(route['waypoints'])}")
            print(f"Distancia total: {route['distance']:.2f} metros")
        
        print(f"\nRuta más corta: {self.all_possible_routes[0]['distance']:.2f} metros")
        print(f"Ruta más larga: {self.all_possible_routes[-1]['distance']:.2f} metros")

    def update_plot(self):
        """Actualizar la visualización en tiempo real"""
        if not self.initial_pose_received or not self.fig:
            return
                
        self.ax.clear()
        
        # Mostrar el mapa como fondo
        self.ax.imshow(self.map_array, cmap='gray', origin='lower',
                    extent=[self.map_origin[0], 
                            self.map_origin[0] + self.map_array.shape[1] * self.map_resolution,
                            self.map_origin[1], 
                            self.map_origin[1] + self.map_array.shape[0] * self.map_resolution])
        
        # Plotear puntos de interés
        locations = self.get_product_locations()
        for loc in locations:
            self.ax.scatter(loc['x'], loc['y'], c='red', s=100, alpha=0.7)
            self.ax.annotate(loc['name'], (loc['x'], loc['y']), 
                        xytext=(5, 5), textcoords='offset points')

        # Plotear posición inicial
        self.ax.scatter(self.start_pose['x'], self.start_pose['y'], 
                    c='green', s=100, label='Posición inicial')

        # Plotear las 3 mejores rutas
        if self.all_possible_routes:
            colors = ['g-', 'b--', 'r:']
            labels = ['Mejor ruta', 'Segunda mejor', 'Tercera mejor']
            
            for i, (route_data, color, label) in enumerate(zip(self.all_possible_routes[:3], colors, labels)):
                route = route_data['route']
                x_coords = [point['x'] for point in route]
                y_coords = [point['y'] for point in route]
                self.ax.plot(x_coords, y_coords, color, 
                        label=f'{label} ({route_data["distance"]:.2f}m)', alpha=0.7)

        # Plotear ruta actual suavizada
        if self.current_path:
            # Calcular la distancia entre la posición inicial y el primer punto de la ruta
            initial_gap = self.calculate_distance(self.start_pose, self.current_path[0])
            
            # Crear el path completo incluyendo el punto inicial
            complete_path = [self.start_pose] + self.current_path
            
            # Suavizar la ruta completa
            smoothed_path = self.smooth_path(complete_path)
            x_coords = [point['x'] for point in smoothed_path]
            y_coords = [point['y'] for point in smoothed_path]
            
            # Calcular la distancia total (incluyendo la compensación inicial)
            path_distance = self.calculate_route_distance(self.current_path)
            total_distance = path_distance + initial_gap
            
            # Plotear la ruta principal con la distancia total correcta
            self.ax.plot(x_coords, y_coords, 'm-', 
                    label=f'Ruta actual ({total_distance:.2f}m)', linewidth=2)

            # Plotear posición actual del robot
            if self.current_path:
                current_pos = self.current_path[-1]
                self.ax.scatter(current_pos['x'], current_pos['y'], 
                            c='blue', s=150, marker='*', label='Robot')

        self.ax.set_title('Comparación de Rutas en Tiempo Real')
        self.ax.set_xlabel('X (metros)')
        self.ax.set_ylabel('Y (metros)')
        self.ax.legend()
        self.ax.grid(True)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()  

    def amcl_callback(self, msg):
        """Callback para verificar que AMCL está publicando"""
        if not self.amcl_initialized:
            self.amcl_initialized = True
            self.get_logger().info('AMCL inicializado correctamente')

    def status_callback(self, msg):
        """Callback para mensajes de estado de navegación"""
        status_parts = msg.data.split('|')
        status = status_parts[0]
        
        if status == "NAVIGATING" and not self.is_navigating:
            self.is_navigating = True
            print("\nIniciando nueva navegación...")
            
        elif status in ["COMPLETED", "CANCELED", "FAILED"]:
            self.is_navigating = False
            if self.current_path and self.all_possible_routes:
                # Calcular la distancia entre la posición inicial y el primer punto de la ruta
                initial_gap = self.calculate_distance(self.start_pose, self.current_path[0])
                
                # Calcular la distancia total incluyendo la compensación inicial
                final_distance = self.calculate_route_distance(self.current_path)
                if initial_gap > 0.1:
                    final_distance += initial_gap
                    
                optimal_distance = self.all_possible_routes[0]['distance']
                efficiency = (optimal_distance / final_distance) * 100 if final_distance > 0 else 0
                
                print("\n\nResultados finales:")
                print("===================")
                print(f"Estado final: {status}")
                print(f"Distancia recorrida: {final_distance:.2f} metros")
                if initial_gap > 0.1:
                    print(f"Compensación inicial incluida: {initial_gap:.2f} metros")
                print(f"Distancia óptima: {optimal_distance:.2f} metros")
                print(f"Diferencia: {final_distance - optimal_distance:.2f} metros")
                print(f"Eficiencia: {efficiency:.2f}%")

    def odom_callback(self, msg):
        """Callback para datos de odometría"""
        # Solo procesar odometría si AMCL está inicializado
        if not self.amcl_initialized:
            return

        point = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

        # Ignorar valores extremadamente grandes
        if abs(point['x']) >= 100000 or abs(point['y']) >= 100000:
            self.get_logger().warning('Ignorando posición inválida (valor extremo): ' + 
                                    f'x={point["x"]:.2f}, y={point["y"]:.2f}')
            return

        # Solo filtrar valores iniciales hasta obtener una posición válida
        if not self.initial_pose_received:
            # Primera posición válida después de que AMCL está inicializado
            self.start_pose = point.copy()
            self.initial_pose_received = True
            print(f"\nPosición inicial válida recibida: x={point['x']:.2f}, y={point['y']:.2f}")
            
            # Inicializar visualización y análisis después de recibir posición inicial válida
            self.initialize_visualization()
            self.analyze_all_possible_routes()
            return

        if self.is_navigating:
            self.current_path.append(point)
            
            # Mostrar información en tiempo real
            if len(self.current_path) > 1:
                # Calcular la distancia actual de la ruta
                current_distance = self.calculate_route_distance(self.current_path)
                
                # Calcular la distancia entre la posición inicial y el primer punto de la ruta actual
                initial_gap = self.calculate_distance(self.start_pose, self.current_path[0])
                
                # Solo añadir la distancia inicial si hay una discrepancia significativa (más de 0.1 metros)
                if initial_gap > 0.1:
                    current_distance += initial_gap
                    
                if self.all_possible_routes:
                    optimal_distance = self.all_possible_routes[0]['distance']
                    print(f"\rDistancia actual (con compensación inicial): {current_distance:.2f}m | "
                          f"Óptima: {optimal_distance:.2f}m | "
                          f"Diferencia: {current_distance - optimal_distance:.2f}m | "
                          f"Compensación inicial: {initial_gap:.2f}m", end='')

def main(args=None):
    rclpy.init(args=args)
    node = RouteAnalysisNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCerrando el nodo de análisis de rutas...")
    finally:
        # Limpiar recursos antes de salir
        if node.update_timer:
            node.update_timer.cancel()
        if node.fig:
            plt.close(node.fig)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()