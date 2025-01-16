#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState, PointCloud2
from tf2_msgs.msg import TFMessage
import numpy as np
from std_msgs.msg import String

class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')

        # Subscription to Odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.odom_filtered_subscription = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_real_callback,
            10)
        

        # Subscription to LaserScan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            100)

        # Subscription to TFMessage topic
        self.tf_subscription = self.create_subscription(
            TFMessage,
            'tf',
            self.tf_callback,
            10)

        self.tf_static_subscription = self.create_subscription(
            TFMessage,
            'tf_static',
            self.tf_static_callback,
            10)

        # Subscription to Joint States topic
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        self.message_py_subscription = self.create_subscription(
            String,
            'message_py',
            self.message_py_callback,
            10)

        # Subscription to Point Cloud topic
        #self.point_cloud_subscription = self.create_subscription(
         #    PointCloud2,
          #   'point_cloud',
           #  self.point_cloud_callback,
            # 10)

        self.odom_subscription  # prevent unused variable warning
        self.odom_filtered_subscription  # prevent unused variable warning
        self.scan_subscription  # prevent unused variable warning
        self.tf_subscription  # prevent unused variable warning
        self.tf_static_subscription


        #self.point_cloud_subscription 

    def odom_callback(self, msg):
        # Accessing specific fields of the Odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        #self.get_logger().info(f'Odometry - Position: [{position.x}, {position.y}]')
        #self.get_logger().info(f'Odometry - Orientation: [{np.rad2deg(orientation.z)}]')
    

    def odom_real_callback(self, msg):
        # Accessing specific fields of the Odometry message
        position_filtered = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(f'Odometry Real - Position: [{position_filtered.x}, {position_filtered.y}]')
        #self.get_logger().info(f'Odometry - Orientation: [{np.rad2deg(orientation.z)}]')


    def scan_callback(self, msg):
        # Process LaserScan message
        # You can access the data from the LaserScan message here
        self.scan_ranges = msg.ranges
        leng = len(self.scan_ranges)
        #print("ScanLength: ", leng)
        #print("Scan: ", self.scan_ranges)
        #pass


    def tf_callback(self, msg):
        # Process TFMessage
        # for transform in msg.transforms:
        #self.get_logger().info(f'TF - Transform: {transform}')
        pass

    def tf_static_callback(self, msg):
        # Process TF Static Message
        # for transform in msg.transforms:
        #self.get_logger().info(f'TF Static - Transform: {transform}')
        pass

    def voltage_callback(self, msg):
        # Process Voltage message
        #self.get_logger().info(f'Voltage: {msg.data}')
        pass


    def joint_states_callback(self, msg):
        # Process Joint States message
        # self.get_logger().info(f'Joint States: {msg}')
        pass

    def point_cloud_callback(self, msg):
        # Process Point Cloud message
        #self.get_logger().info('Processing Point Cloud message')
        pass
    def message_py_callback(self, msg):
        #self.get_logger().info(f'Message Py: {msg.data}')
        pass

def main(args=None):
    rclpy.init(args=args)
    subscriber = MySubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

