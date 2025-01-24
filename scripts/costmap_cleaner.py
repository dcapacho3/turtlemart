#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap
import time

# Tiempo entre limpiezas en segundos
CLEAN_INTERVAL = 1.2

class CostmapCleaner(Node):
    def __init__(self):
        super().__init__('costmap_cleaner')
        self.client = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

    def clear_costmap(self):
        request = ClearEntireCostmap.Request()
        self.client.call_async(request)

def main():
    rclpy.init()
    cleaner = CostmapCleaner()
    
    try:
        while True:
            cleaner.clear_costmap()
            time.sleep(CLEAN_INTERVAL)
    except KeyboardInterrupt:
        pass
    
    cleaner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()