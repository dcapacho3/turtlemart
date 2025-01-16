#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

class PlanVisualizer(Node):
    def __init__(self):
        super().__init__('plan_visualizer')
        
        # Suscribirse al topic del plan
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            1  # Solo necesitamos el primer mensaje
        )
        self.got_plan = False

    def plan_callback(self, msg):
        if not self.got_plan:  # Solo procesar el primer plan recibido
            # Extraer coordenadas x, y
            path_x = [pose.pose.position.x for pose in msg.poses]
            path_y = [pose.pose.position.y for pose in msg.poses]
            
            # Crear la visualización
            plt.figure(figsize=(10, 8))
            plt.plot(path_x, path_y, 'b-', linewidth=2, label='Plan')
            plt.plot(path_x[0], path_y[0], 'ro', markersize=10, label='Inicio')
            plt.plot(path_x[-1], path_y[-1], 'g*', markersize=15, label='Objetivo')
            
            # Configurar la gráfica
            plt.xlabel('X (metros)')
            plt.ylabel('Y (metros)')
            plt.title('Plan de Navegación')
            plt.grid(True)
            plt.legend()
            plt.axis('equal')
            
            # Mostrar la gráfica
            plt.show()
            
            # Marcar que ya recibimos el plan
            self.got_plan = True
            
            # Cerrar el nodo
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    visualizer = PlanVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')

if __name__ == '__main__':
    main()