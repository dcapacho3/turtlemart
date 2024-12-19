#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
import math

class TrajectoryVisualizerNode(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'trajectory_markers', 10)
        self.trajectory_points = []
        self.marker_array = MarkerArray()
        self.last_point = None
        self.min_distance = 0.1  # Minimum distance (in meters) to add a new point

        # Set up TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg):
        try:
            # Look up the transform from odom to map
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            
            # Transform the point from odom to map frame
            point_stamped = tf2_geometry_msgs.PointStamped()
            point_stamped.header = msg.header
            point_stamped.point = msg.pose.pose.position
            
            point_transformed = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            
            new_point = Point()
            new_point.x = point_transformed.point.x
            new_point.y = point_transformed.point.y
            new_point.z = point_transformed.point.z
            
            # Check if the new point is far enough from the last point
            if self.last_point is None or self.distance(new_point, self.last_point) >= self.min_distance:
                self.trajectory_points.append(new_point)
                self.last_point = new_point
                self.publish_trajectory_marker()
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform odometry to map frame: {ex}')

    def distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

    def publish_trajectory_marker(self):
        self.marker_array = MarkerArray()  # Reset the marker array

        # Create markers for all points
        for i, point in enumerate(self.trajectory_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectory_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color
            marker.lifetime.sec = 0  # Marker persists indefinitely
            self.marker_array.markers.append(marker)

        # Create the line strip marker
        line_strip = Marker()
        line_strip.header.frame_id = "map"
        line_strip.header.stamp = self.get_clock().now().to_msg()
        line_strip.ns = "trajectory_line"
        line_strip.id = 0
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.pose.orientation.w = 1.0
        line_strip.scale.x = 0.02  # Line width
        line_strip.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green color
        line_strip.points = self.trajectory_points
        line_strip.lifetime.sec = 0  # Marker persists indefinitely
        self.marker_array.markers.append(line_strip)

        self.marker_publisher.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()