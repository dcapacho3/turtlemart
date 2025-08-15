#!/usr/bin/env python3
"""
Nodo de navegación automática para tutorial SARA
Navega continuamente entre 4 puntos predefinidos: 1->2->3->4->1...
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class AutoNavigationTutorial(Node):
    def __init__(self):
        super().__init__('auto_navigation_tutorial')
        
        # Cliente de acción para Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Puntos de navegación (ajustados para el mapa del supermercado)
        self.waypoints = [
            {'name': 'Punto 1', 'x': 0.86, 'y': -1.13, 'yaw': 0.0},
            {'name': 'Punto 2', 'x': 0.86, 'y': 2.08, 'yaw': 1.57},
            {'name': 'Punto 3', 'x': -0.78, 'y': 2.08, 'yaw': 3.14},
            {'name': 'Punto 4', 'x': -0.78, 'y': -1.13, 'yaw': -1.57},
        ]
        
        self.current_waypoint = 0
        self.navigation_active = True
        
        # Timer para verificar estado
        self.timer = self.create_timer(2.0, self.check_and_navigate)
        
        self.get_logger().info('Navegación automática iniciada - Tutorial SARA')
        self.get_logger().info(f'Navegando a {len(self.waypoints)} puntos continuamente')

    def create_goal(self, waypoint):
        """Crear goal de navegación para un waypoint"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Posición
        goal_msg.pose.pose.position.x = waypoint['x']
        goal_msg.pose.pose.position.y = waypoint['y']
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientación (convertir yaw a quaternion simple)
        import math
        yaw = waypoint['yaw']
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        
        return goal_msg

    def check_and_navigate(self):
        """Verificar estado y navegar al siguiente punto"""
        if not self.navigation_active:
            return
            
        # Verificar si hay una navegación en curso
        if hasattr(self, '_current_goal_handle') and self._current_goal_handle:
            return
            
        # Navegar al siguiente waypoint
        waypoint = self.waypoints[self.current_waypoint]
        self.get_logger().info(f'Navegando a {waypoint["name"]} ({waypoint["x"]}, {waypoint["y"]})')
        
        # Esperar a que el servidor de acción esté disponible
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Servidor de navegación no disponible')
            return
        
        # Enviar goal
        goal_msg = self.create_goal(waypoint)
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback cuando se acepta el goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado')
            return
            
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Callback cuando se completa la navegación"""
        result = future.result().result
        self._current_goal_handle = None
        
        waypoint = self.waypoints[self.current_waypoint]
        self.get_logger().info(f'Llegué a {waypoint["name"]}!')
        
        # Avanzar al siguiente waypoint
        self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
        
        # Pequeña pausa antes del siguiente punto
        time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    auto_nav = AutoNavigationTutorial()
    
    try:
        rclpy.spin(auto_nav)
    except KeyboardInterrupt:
        pass
    
    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
