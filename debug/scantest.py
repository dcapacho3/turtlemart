#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        
        # Create QoS profile matching YDLIDAR's settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Create subscription to LaserScan messages with the QoS profile
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        # Initialize plot
        plt.style.use('dark_background')  # Better visibility
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.fig.set_facecolor('#262626')  # Dark gray background
        self.ax.set_facecolor('#262626')
        
        self.ax.set_title('LiDAR Scan Visualization', color='white')
        self.ax.set_theta_zero_location('N')  # 0 degrees at North
        self.ax.set_theta_direction(-1)  # Clockwise
        self.ax.grid(True, color='gray', alpha=0.3)
        
        # Initialize empty line
        self.line, = self.ax.plot([], [], 'g.', markersize=2, alpha=0.6)
        
        # Store latest scan data
        self.angles = []
        self.ranges = []
        self.max_range = 0
        
        # Set up animation with proper parameters
        self.ani = FuncAnimation(
            self.fig, 
            self.update_plot,
            interval=100,  # Update every 100ms
            blit=True,
            cache_frame_data=False,  # Disable frame caching
            save_count=None  # Don't save frames
        )
        
        self.get_logger().info('LiDAR visualizer started with BEST_EFFORT reliability')

    def scan_callback(self, msg):
        # Convert scan data to numpy arrays
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        self.angles = np.arange(
            angle_min,
            angle_max + angle_increment,
            angle_increment
        )
        
        # Replace inf values with max_range
        self.ranges = np.array(msg.ranges)
        self.max_range = msg.range_max
        self.ranges[np.isinf(self.ranges)] = msg.range_max
        
        # Filter out invalid measurements (optional)
        mask = np.isfinite(self.ranges) & (self.ranges > msg.range_min) & (self.ranges < msg.range_max)
        self.angles = self.angles[mask]
        self.ranges = self.ranges[mask]
        
        # Log first message receipt
        if not hasattr(self, '_first_message_received'):
            self.get_logger().info('First scan message received successfully')
            self._first_message_received = True

    def update_plot(self, frame):
        if len(self.angles) > 0 and len(self.ranges) > 0:
            self.line.set_data(self.angles, self.ranges)
            
            # Dynamic range adjustment
            current_max = np.max(self.ranges)
            if current_max > 0:
                self.ax.set_rmax(min(current_max * 1.1, self.max_range))
            
            # Update grid and labels for better visibility
            for label in self.ax.get_xticklabels():
                label.set_color('white')
            for label in self.ax.get_yticklabels():
                label.set_color('white')
        
        return self.line,

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = LidarVisualizer()
    
    try:
        plt.show(block=True)
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()
        plt.close()

if __name__ == '__main__':
    main()
