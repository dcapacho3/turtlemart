#!/usr/bin/env python3
import sqlite3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult
import math
import numpy as np
import yaml
from PIL import Image
import os
import matplotlib.pyplot as plt
from itertools import permutations
from ament_index_python.packages import get_package_share_directory

class AutonomousNavigator:
    def __init__(self):
        self.node = rclpy.create_node('navigator_node')
        self.odom_subscriber = OdomSubscriber(self.node)
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.map_array, self.resolution, self.origin = self.load_map()
        print("Map loaded successfully.")

    def load_map(self):
        bringup_dir = get_package_share_directory('turtlemart')
        map_yaml_path = os.path.join(bringup_dir, 'maps/supermarket_map.yaml')
        with open(map_yaml_path, 'r') as f:
            yaml_content = yaml.safe_load(f)
        
        resolution = yaml_content['resolution']
        origin = yaml_content['origin']
        map_image_path = yaml_content['image']
        
        if not os.path.isabs(map_image_path):
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)
        
        print(f'Loading map image from: {map_image_path}')
        map_image = Image.open(map_image_path)
        map_array = np.array(map_image)
        
        return map_array, resolution, origin

    def get_product_locations(self):
        db_dir = os.path.join('src/turtlemart/database/products.db')
        conn = sqlite3.connect(db_dir)
        cursor = conn.cursor()
        cursor.execute('SELECT name, x, y FROM selected_products')
        locations = [{'name': row[0], 'x': row[1], 'y': row[2]} for row in cursor.fetchall()]
        conn.close()
        return locations

    def calculate_distance(self, p1, p2):
        return math.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)

    def calculate_distance_matrix(self, locations):
        locs = [(loc['x'], loc['y']) for loc in locations]
        return np.array([[self.calculate_distance({'x': loc1[0], 'y': loc1[1]}, {'x': loc2[0], 'y': loc2[1]}) for loc2 in locs] for loc1 in locs])

    def tsp_bruteforce(self, distance_matrix):
        n = len(distance_matrix)
        min_path = None
        min_distance = float('inf')
        for perm in permutations(range(n)):
            dist = sum(distance_matrix[perm[i], perm[i + 1]] for i in range(n - 1))
            if dist < min_distance:
                min_distance = dist
                min_path = perm
        return min_path

    def sort_waypoints_by_tsp(self, locations):
        distance_matrix = self.calculate_distance_matrix(locations)
        tsp_order = self.tsp_bruteforce(distance_matrix)
        return [locations[i] for i in tsp_order]

    def visualize_obstacles(self, locations, tsp_path, start_pose):
        plt.ion()
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map_array, cmap='gray', origin='lower')

        for loc in locations:
            x_index = int((loc['x'] - self.origin[0]) / self.resolution)
            y_index = int((loc['y'] - self.origin[1]) / self.resolution)
            y_index = self.map_array.shape[0] - y_index - 1
            plt.scatter(x_index, y_index, color='blue', marker='o', s=50, label=loc['name'])
        
        tsp_coords = [(start_pose['x'], start_pose['y'])] + [(locations[i]['x'], locations[i]['y']) for i in tsp_path]
        tsp_coords_index = [(int((x - self.origin[0]) / self.resolution), self.map_array.shape[0] - int((y - self.origin[1]) / self.resolution) - 1) for x, y in tsp_coords]
        
        plt.plot(*zip(*tsp_coords_index), color='magenta', marker='o', markersize=5, linestyle='-', linewidth=2, label='TSP Path')
        plt.legend(loc='upper right', bbox_to_anchor=(1.2, 1))
        plt.title('Ubicaciones de Productos y Trayectoria TSP')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

    def calculate_completion_percentage(self, start_pose, current_pose, goal_pose):
        total_distance = self.calculate_distance(start_pose, goal_pose)
        remaining_distance = self.calculate_distance(current_pose, goal_pose)
        completion_percentage = ((total_distance - remaining_distance) / total_distance) * 100
        return max(0, min(completion_percentage, 100))

    def request_user_confirmation(self, message):
        while True:
            user_input = input(message)
            if user_input.lower() == 'c':
                break

    def navigate(self):
        plt.show()
        print("Waiting for initial position from odometry...")
        while self.odom_subscriber.current_pose is None:
            rclpy.spin_once(self.node)
        start_pose = self.odom_subscriber.current_pose
        print(f"Initial position: {start_pose}")

        locations = self.get_product_locations()
        if not locations:
            print("No selected products found.")
            return

        sorted_locations = self.sort_waypoints_by_tsp(locations)
        tsp_path = [sorted_locations.index(loc) for loc in sorted_locations]
        self.visualize_obstacles(sorted_locations, tsp_path, start_pose)
        
        # Solicitar confirmación del usuario antes de iniciar la navegación
        self.request_user_confirmation(f"Presione 'c' para comenzar la navegación ({len(sorted_locations)} waypoints)...")

        goal_poses = []
        pose_names = {}

        for loc in sorted_locations:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = loc['x']
            goal_pose.pose.position.y = loc['y']
            goal_pose.pose.orientation.w = 1.0
            goal_poses.append(goal_pose)
            pose_names[len(goal_poses) - 1] = loc['name']

        current_waypoint = 0
        total_waypoints = len(goal_poses)

        while current_waypoint < total_waypoints:
            goal_pose = goal_poses[current_waypoint]
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isNavComplete():
                rclpy.spin_once(self.node, timeout_sec=1.0)
                feedback = self.navigator.getFeedback()
                if feedback:
                    current_pose = self.odom_subscriber.current_pose
                    if current_pose:
                        completion_percentage = self.calculate_completion_percentage(start_pose, current_pose, {'x': goal_pose.pose.position.x, 'y': goal_pose.pose.position.y})
                        print(f'Navegando hacia "{pose_names[current_waypoint]}" {current_waypoint + 1}/{total_waypoints} ({completion_percentage:.1f}% completado)')
                plt.pause(0.001)

            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                print(f'Waypoint "{pose_names[current_waypoint]}" alcanzado exitosamente.')
            elif result == NavigationResult.CANCELED:
                print(f'La navegación al waypoint "{pose_names[current_waypoint]}" fue cancelada.')
                break
            elif result == NavigationResult.FAILED:
                print(f'Error al navegar al waypoint "{pose_names[current_waypoint]}".')
                break

            # Solicitar confirmación del usuario para continuar al siguiente waypoint
            self.request_user_confirmation(f"Presione 'c' para continuar al siguiente waypoint ({current_waypoint + 1}/{total_waypoints})...")

            current_waypoint += 1

        print('Navegación completada.')
        
        return current_waypoint

class OdomSubscriber:
    def __init__(self, node):
        self.node = node
        self.subscription = node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_pose = None

    def odom_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }

def main():
    rclpy.init()
    navigator = AutonomousNavigator()
    navigator.navigate()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

