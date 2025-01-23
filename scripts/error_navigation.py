#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        
        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        # Buffer para almacenar las últimas N rutas
        self.buffer_size = 50
        self.path_buffer = deque(maxlen=self.buffer_size)
        self.current_plan = None
        
        # Parámetros para filtrado
        self.distance_threshold = 0.05  # metros
        self.filtered_points = []
        self.last_filtered_points = []
        
        # Configurar plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        
        # Líneas para visualización
        self.current_line, = self.ax.plot([], [], 'g-', linewidth=2, 
                                        label='Plan Actual')
        self.filtered_line, = self.ax.plot([], [], 'r-', linewidth=2, 
                                         label='Ruta Completa')
        self.points_scatter = self.ax.scatter([], [], c='b', alpha=0.2, s=1,
                                            label='Puntos Históricos')
        
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Visualización de Planificación Nav2')
        self.ax.grid(True)
        self.ax.legend()

    def update_filtered_points(self, new_plan_points):
        """
        Actualiza los puntos filtrados basándose en el nuevo plan.
        """
        if not new_plan_points:
            return
        
        # Si es el primer plan, inicializar con estos puntos
        if not self.filtered_points:
            self.filtered_points = new_plan_points
            return
        
        # Encontrar el punto donde el nuevo plan diverge del anterior
        divergence_idx = None
        for i, new_point in enumerate(new_plan_points):
            if i >= len(self.filtered_points):
                break
                
            dist = np.hypot(new_point[0] - self.filtered_points[i][0],
                          new_point[1] - self.filtered_points[i][1])
            
            if dist > self.distance_threshold:
                divergence_idx = i
                break
        
        # Si no hay divergencia, mantener los puntos actuales
        if divergence_idx is None:
            return
        
        # Mantener los puntos hasta la divergencia y agregar los nuevos
        self.filtered_points = (self.filtered_points[:divergence_idx] + 
                              new_plan_points[divergence_idx:])

    def path_callback(self, msg):
        # Extraer puntos del nuevo plan
        new_points = [(pose.pose.position.x, pose.pose.position.y)
                     for pose in msg.poses]
        
        # Actualizar plan actual
        self.current_plan = new_points
        
        # Agregar al buffer
        self.path_buffer.append(new_points)
        
        # Actualizar puntos filtrados
        self.update_filtered_points(new_points)
        
        # Actualizar visualización
        self.update_plot()

    def get_all_points(self):
        """
        Obtiene todos los puntos históricos del buffer.
        """
        all_points = []
        for path in self.path_buffer:
            all_points.extend(path)
        return all_points

    def update_plot(self):
        # Limpiar scatter plot anterior
        self.points_scatter.remove()
        
        # Mostrar todos los puntos históricos
        all_points = self.get_all_points()
        if all_points:
            x_all, y_all = zip(*all_points)
            self.points_scatter = self.ax.scatter(x_all, y_all, c='b', alpha=0.2, 
                                                s=1, label='Puntos Históricos')
        
        # Mostrar plan actual
        if self.current_plan:
            x_curr, y_curr = zip(*self.current_plan)
            self.current_line.set_data(x_curr, y_curr)
        
        # Mostrar ruta filtrada
        if self.filtered_points:
            x_filt, y_filt = zip(*self.filtered_points)
            self.filtered_line.set_data(x_filt, y_filt)
        
        # Ajustar límites
        self.ax.relim()
        self.ax.autoscale_view()
        
        # Redibujar
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main():
    rclpy.init()
    path_visualizer = PathVisualizer()
    
    try:
        rclpy.spin(path_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        path_visualizer.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()