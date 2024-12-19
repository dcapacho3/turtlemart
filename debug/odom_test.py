import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math

class CirclePathNode(Node):
    def __init__(self):
        super().__init__('circle_path_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Define circle parameters
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.2  # rad/s
        self.radius = self.linear_speed / self.angular_speed

        # Track odometry data
        self.actual_path_x = []
        self.actual_path_y = []
        
        # Initialize plotting
        self.plot_initialized = False
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Timer to publish twist messages
        self.timer = self.create_timer(0.1, self.move_in_circle)
    
    def move_in_circle(self):
        # Publish circular movement
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher.publish(twist)
        
        # Stop after a set duration
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        if elapsed_time > 30:  # Run for 30 seconds
            self.stop_robot()
            self.plot_paths()

    def stop_robot(self):
        twist = Twist()
        self.publisher.publish(twist)  # Stop the robot by sending zero velocities
        self.timer.cancel()

    def odom_callback(self, msg):
        # Get current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.actual_path_x.append(x)
        self.actual_path_y.append(y)

    def plot_paths(self):
        # Generate the desired circular path for comparison
        desired_path_x = []
        desired_path_y = []
        num_points = 100
        for i in range(num_points):
            theta = (2 * math.pi / num_points) * i
            x = self.radius * math.cos(theta)
            y = self.radius * math.sin(theta)
            desired_path_x.append(x)
            desired_path_y.append(y)

        # Plot actual vs desired paths
        plt.figure()
        plt.plot(desired_path_x, desired_path_y, 'g--', label="Desired Path (Circle)")
        plt.plot(self.actual_path_x, self.actual_path_y, 'b-', label="Actual Path")
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Robot Circle Path - Desired vs Actual")
        plt.legend()
        plt.grid()
        plt.axis('equal')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = CirclePathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
