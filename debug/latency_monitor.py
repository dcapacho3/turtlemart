#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import statistics
import numpy as np
from datetime import datetime

class LatencyMonitor(Node):
    def __init__(self, duration=60):
        super().__init__('latency_monitor')
        
        self.duration = duration
        self.start_time = time.time()
        self.latencies = []
        self.movement_threshold = 0.001
        
        self.last_cmd_time = None
        self.last_cmd_vel = None
        self.last_odom_vel = None
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel_out',
            self.cmd_vel_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.get_logger().info(f'Iniciando monitoreo de latencia durante {duration} segundos...')
        
    def cmd_vel_callback(self, msg):
        if time.time() - self.start_time > self.duration:
            self.print_statistics()
            rclpy.shutdown()
            return
            
        current_time = time.time()
        
        if abs(msg.linear.x) > 0 or abs(msg.angular.z) > 0:
            if self.last_cmd_vel is None or (
                msg.linear.x != self.last_cmd_vel.linear.x or 
                msg.angular.z != self.last_cmd_vel.angular.z
            ):
                self.last_cmd_time = current_time
                self.last_cmd_vel = msg
    
    def odom_callback(self, msg):
        if self.last_cmd_time is None:
            return
            
        current_time = time.time()
        current_vel = msg.twist.twist
        
        if (abs(current_vel.linear.x) > self.movement_threshold or 
            abs(current_vel.angular.z) > self.movement_threshold):
            
            if self.last_odom_vel is None or (
                abs(current_vel.linear.x - self.last_odom_vel.linear.x) > self.movement_threshold or
                abs(current_vel.angular.z - self.last_odom_vel.angular.z) > self.movement_threshold
            ):
                latency = (current_time - self.last_cmd_time) * 1000
                self.latencies.append(latency)
                self.last_odom_vel = current_vel
                self.last_cmd_time = None
                
    def print_statistics(self):
        if not self.latencies:
            self.get_logger().warn('¡No se recolectaron mediciones de latencia!')
            return
            
        median = statistics.median(self.latencies)
        mean = statistics.mean(self.latencies)
        std_dev = statistics.stdev(self.latencies) if len(self.latencies) > 1 else 0
        min_lat = min(self.latencies)
        max_lat = max(self.latencies)
        
        percentiles = np.percentile(self.latencies, [25, 75, 90, 95])
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        report = f"""
Reporte de Latencia ({timestamp})
{'='*50}
Total de Muestras: {len(self.latencies)}
Duración: {self.duration} segundos

Resumen Estadístico (en milisegundos):
{'-'*50}
Latencia Mediana: {median:.2f}ms
Latencia Media: {mean:.2f}ms
Desviación Estándar: {std_dev:.2f}ms
Latencia Mínima: {min_lat:.2f}ms
Latencia Máxima: {max_lat:.2f}ms

Percentiles:
{'-'*50}
Percentil 25: {percentiles[0]:.2f}ms
Percentil 75: {percentiles[1]:.2f}ms
Percentil 90: {percentiles[2]:.2f}ms
Percentil 95: {percentiles[3]:.2f}ms
"""
        print(report)

def main():
    rclpy.init()
    
    import sys
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    
    latency_monitor = LatencyMonitor(duration)
    rclpy.spin(latency_monitor)
    
    latency_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
