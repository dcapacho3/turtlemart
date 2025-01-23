#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult
import math
import matplotlib.pyplot as plt
import numpy as np
from tf_transformations import quaternion_from_euler
from scipy.spatial.distance import cdist
from scipy.stats import describe
from datetime import datetime
import csv

class NavigationRepeatabilityTester(Node):
    def __init__(self):
        super().__init__('navigation_repeatability_tester')
        
        # Initialize points for navigation
        self.points = {
            # Main points
            'A': {'x': 0.15, 'y': 0.8},
            'B': {'x': 2.7, 'y': 0.1},
            # Secondary points
            #'A': {'x': 0.5, 'y': -2.6},
            #'B': {'x': -0.5, 'y': 1.0}
        }
        
        # Initialize navigator
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        # Create odometry subscription
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Initialize variables
        self.current_pose = None
        self.current_trajectory = []
        self.all_trajectories = []  # List of all trajectories
        self.iteration_metrics = []  # List to store metrics for each iteration
        
    def odom_callback(self, msg):
        """Callback for updating current position and recording trajectory"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }
        if self.current_trajectory is not None:
            self.current_trajectory.append(self.current_pose.copy())

    def create_goal_pose(self, target_point):
        """Create PoseStamped message for navigation goal"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = target_point['x']
        goal_pose.pose.position.y = target_point['y']
        
        # Calculate orientation towards goal
        if self.current_pose:
            yaw = math.atan2(
                target_point['y'] - self.current_pose['y'],
                target_point['x'] - self.current_pose['x']
            )
            q = quaternion_from_euler(0, 0, yaw)
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]
        else:
            goal_pose.pose.orientation.w = 1.0
        
        return goal_pose

    def calculate_trajectory_metrics(self, trajectory1, trajectory2=None):
        """Calculate metrics for trajectory comparison"""
        if not trajectory1:
            return None
            
        # Convert trajectory to numpy array
        traj1 = np.array([(p['x'], p['y']) for p in trajectory1])
        
        if trajectory2 is not None:
            # Compare two trajectories
            traj2 = np.array([(p['x'], p['y']) for p in trajectory2])
            
            # Calculate point-wise distances
            distances = cdist(traj1, traj2)
            min_distances = np.min(distances, axis=1)
            
            metrics = {
                'mean_deviation': float(np.mean(min_distances)),
                'max_deviation': float(np.max(min_distances)),
                'std_deviation': float(np.std(min_distances)),
                'path_length_diff': float(abs(self.calculate_path_length(traj1) - 
                                           self.calculate_path_length(traj2)))
            }
        else:
            # Single trajectory metrics
            metrics = {
                'path_length': float(self.calculate_path_length(traj1)),
               # 'smoothness': float(self.calculate_smoothness(traj1))
            }
            
        return metrics

    def calculate_path_length(self, trajectory):
        """Calculate total path length"""
        return np.sum(np.sqrt(np.sum(np.diff(trajectory, axis=0)**2, axis=1)))


    def print_iteration_summary(self, iteration, target, metrics):
        """Print summary for current iteration"""
        print(f"\n=== Iteration {iteration} Summary (Target: Point {target}) ===")
        print(f"Path Length: {metrics['path_length']:.3f} meters")
       # print(f"Path Smoothness: {metrics['smoothness']:.3f}")
        
        if len(self.all_trajectories) > 1:
            # Compare with previous trajectory to same target
            prev_trajectories = [t for t in self.all_trajectories[:-1] 
                               if t['target'] == target]
            if prev_trajectories:
                prev_traj = prev_trajectories[-1]['trajectory']
                comparison = self.calculate_trajectory_metrics(
                    self.current_trajectory, prev_traj)
                print("\nComparison with previous run:")
                print(f"Mean Deviation: {comparison['mean_deviation']:.3f} meters")
                print(f"Max Deviation: {comparison['max_deviation']:.3f} meters")
                print(f"Path Length Difference: {comparison['path_length_diff']:.3f} meters")

    def save_navigation_data(self):
        """Save navigation data to a single CSV file"""
        if not self.all_trajectories:
            print("No data to save")
            return None
            
        # Create timestamp for unique filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"nav2_data_{timestamp}.csv"
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Write header
            writer.writerow(['Section', 'Iteration', 'Target', 'Path_Length', 
                            'Mean_Deviation', 'Max_Deviation', 'Path_Length_Diff',
                            'Point_Index', 'X', 'Y'])
            
            # Write data for each trajectory
            test_trajectories = [traj for traj in self.all_trajectories]
            for i, traj in enumerate(test_trajectories):
                # Calculate comparison metrics with previous trajectory to same target
                if i > 0:
                    prev_traj = test_trajectories[i-1]
                    comparison = self.calculate_trajectory_metrics(
                        traj['trajectory'], prev_traj['trajectory'])
                    mean_dev = comparison['mean_deviation']
                    max_dev = comparison['max_deviation']
                    path_diff = comparison['path_length_diff']
                else:
                    mean_dev = None
                    max_dev = None
                    path_diff = None
                
                # Write trajectory points with full metrics
                for point_idx, point in enumerate(traj['trajectory']):
                    writer.writerow([
                        'trajectory',                # Section identifier
                        traj['iteration'],           # Iteration number
                        traj['target'],             # Target point (A or B)
                        traj['metrics']['path_length'], # Path length
                        mean_dev,                   # Mean deviation from previous
                        max_dev,                    # Max deviation from previous
                        path_diff,                  # Path length difference
                        point_idx,                  # Index of point in trajectory
                        point['x'],                 # X coordinate
                        point['y']                  # Y coordinate
                    ])
        
        print(f"\nData saved to file: {filename}")
        return filename



    def plot_results(self):
        """Generate visualization of all trajectories"""
        plt.figure(figsize=(12, 8))
        
        # Plot points A and B
        plt.plot(self.points['A']['x'], self.points['A']['y'], 'g*', 
                markersize=15, label='Point A')
        plt.plot(self.points['B']['x'], self.points['B']['y'], 'r*', 
                markersize=15, label='Point B')
        
        # Filter out initialization trajectory and plot remaining trajectories
        test_trajectories = [traj for traj in self.all_trajectories if traj['iteration'] > 0]
        colors = plt.cm.rainbow(np.linspace(0, 1, len(test_trajectories)))
        
        for traj_data, color in zip(test_trajectories, colors):
            trajectory = np.array([(p['x'], p['y']) for p in traj_data['trajectory']])
            plt.plot(trajectory[:, 0], trajectory[:, 1], '-', color=color, 
                    alpha=0.7, label=f"Run {traj_data['iteration']}: {traj_data['target']}")
        
        plt.title('Navigation Repeatability Analysis')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.grid(True)
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.axis('equal')
        
        # Add statistical annotations
        self.add_statistical_annotations()
        
        plt.tight_layout()
        plt.show()

    def add_statistical_annotations(self):
        """Add statistical information to the plot"""
        # Filter out initialization trajectory
        test_trajectories = [traj for traj in self.all_trajectories if traj['iteration'] > 0]
        
        if len(test_trajectories) < 2:
            return
            
        # Calculate overall statistics
        all_metrics = []
        for i in range(1, len(test_trajectories)):
            metrics = self.calculate_trajectory_metrics(
                test_trajectories[i]['trajectory'],
                test_trajectories[i-1]['trajectory']
            )
            if metrics:
                all_metrics.append(metrics)
        
        if not all_metrics:
            return
            
        # Calculate average metrics
        avg_metrics = {
            'mean_deviation': np.mean([m['mean_deviation'] for m in all_metrics]),
            'max_deviation': np.mean([m['max_deviation'] for m in all_metrics])
        }
        
        # Add text box with statistics
        stats_text = (
            f"Overall Statistics (n={len(test_trajectories)})\n"
            f"Average Mean Deviation: {avg_metrics['mean_deviation']:.3f}m\n"
            f"Average Max Deviation: {avg_metrics['max_deviation']:.3f}m"
        )
        
        plt.gcf().text(0.02, 0.98, stats_text,
                    bbox=dict(facecolor='white', alpha=0.8),
                    fontsize=8,
                    verticalalignment='top')

    def print_final_statistics(self):
        """Print comprehensive statistics at the end of testing"""
        # Filter out initialization trajectory
        test_trajectories = [traj for traj in self.all_trajectories if traj['iteration'] > 0]
        
        if len(test_trajectories) < 2:
            print("\nNot enough data for statistical analysis")
            return
            
        print("\n======== NAVIGATION REPEATABILITY ANALYSIS ========")
        print("================================================")
        
        # Separate statistics by target point
        targets = {'A': [], 'B': []}
        for traj_data in test_trajectories:
            targets[traj_data['target']].append(traj_data)
        
        # Global statistics storage
        all_deviations = []
        all_path_lengths = []
        
        # Analyze each target point
        for target, trajectories in targets.items():
            if len(trajectories) < 2:
                continue
                
            print(f"\n=== Point {target} Statistics ===")
            print(f"Number of runs: {len(trajectories)}")
            
            # Calculate metrics for each trajectory
            path_lengths = [t['metrics']['path_length'] for t in trajectories]
            all_path_lengths.extend(path_lengths)
            
            # Calculate deviations between consecutive runs
            deviations = []
            for i in range(1, len(trajectories)):
                metrics = self.calculate_trajectory_metrics(
                    trajectories[i]['trajectory'],
                    trajectories[i-1]['trajectory']
                )
                if metrics:
                    deviations.append(metrics['mean_deviation'])
                    all_deviations.append(metrics['mean_deviation'])
            
            # Print detailed statistics for this target
            print("\nPath Length Statistics:")
            print(f"  Mean: {np.mean(path_lengths):.3f} m")
            print(f"  Std Dev: {np.std(path_lengths):.3f} m")
            print(f"  Min: {np.min(path_lengths):.3f} m")
            print(f"  Max: {np.max(path_lengths):.3f} m")
            
            if deviations:
                print("\nRepeatability Metrics:")
                print(f"  Mean Deviation: {np.mean(deviations):.3f} m")
                print(f"  Max Deviation: {np.max(deviations):.3f} m")
                print(f"  Min Deviation: {np.min(deviations):.3f} m")
                print(f"  Std Dev of Deviations: {np.std(deviations):.3f} m")
        
        # Print global statistics
        print("\n=== GLOBAL STATISTICS ===")
        print(f"Total number of trajectories: {len(test_trajectories)}")
        
        # Solo imprimir estadísticas si hay suficientes datos
        if len(all_path_lengths) > 0:
            print("\nOverall Path Length Statistics:")
            print(f"  Mean: {np.mean(all_path_lengths):.3f} m")
            if len(all_path_lengths) > 1:
                print(f"  Std Dev: {np.std(all_path_lengths):.3f} m")
                mean_path = np.mean(all_path_lengths)
                if mean_path > 0:
                    print(f"  Coefficient of Variation: {(np.std(all_path_lengths)/mean_path)*100:.2f}%")
        
        if all_deviations:  # Solo si hay desviaciones calculadas
            print("\nOverall Repeatability Metrics:")
            print(f"  Mean Deviation: {np.mean(all_deviations):.3f} m")
            print(f"  Max Deviation: {np.max(all_deviations):.3f} m")
            print(f"  Min Deviation: {np.min(all_deviations):.3f} m")
            if len(all_deviations) > 1:
                print(f"  Std Dev of Deviations: {np.std(all_deviations):.3f} m")
            
            print("\nPerformance Summary:")
            if len(all_path_lengths) > 1:
                print(f"  Average path length variation: ±{np.std(all_path_lengths):.3f} m")
            print(f"  Average deviation from previous run: {np.mean(all_deviations):.3f} m")
            
            mean_path = np.mean(all_path_lengths)
            mean_dev = np.mean(all_deviations)
            if mean_path > 0:
                consistency = 100 - (mean_dev/mean_path)*100
                print(f"  Overall navigation consistency: {consistency:.1f}%")
        
        print("\n================================================")
        print("Note: Lower deviation values indicate better repeatability")


    def initialize_position(self):
        """Initialize robot position at one of the test points"""
        if not self.current_pose:
            print("Waiting for odometry data...")
            while not self.current_pose:
                rclpy.spin_once(self, timeout_sec=0.1)
        
        # Calculate distances to both points
        dist_to_a = math.sqrt((self.current_pose['x'] - self.points['A']['x'])**2 + 
                            (self.current_pose['y'] - self.points['A']['y'])**2)
        dist_to_b = math.sqrt((self.current_pose['x'] - self.points['B']['x'])**2 + 
                            (self.current_pose['y'] - self.points['B']['y'])**2)
        
        print(f"\nCurrent position: ({self.current_pose['x']:.3f}, {self.current_pose['y']:.3f})")
        print(f"Distance to A: {dist_to_a:.3f} m")
        print(f"Distance to B: {dist_to_b:.3f} m")
        
        # Determine closest point
        closest_point = 'A' if dist_to_a < dist_to_b else 'B'
        print(f"\nClosest point is: {closest_point}")
        
        # Ask user which point to start from
        while True:
            print("\nSelect starting point (A/B):")
            start_point = input().strip().upper()
            if start_point in ['A', 'B']:
                break
            print("Invalid input. Please enter 'A' or 'B'")
        
        # Always navigate to the selected starting point
        print(f"\nNavigating to point {start_point}...")
        print("Press Enter to begin navigation to starting position...")
        input()
        
        # Initialize trajectory recording for the first navigation
        self.current_trajectory = []
        
        goal_pose = self.create_goal_pose(self.points[start_point])
        self.navigator.goToPose(goal_pose)
        
        # Wait for navigation completion
        while not self.navigator.isNavComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        result = self.navigator.getResult()
        if result != NavigationResult.SUCCEEDED:
            print(f"Failed to reach starting position. Result: {result}")
            return None
        
        # Calculate metrics for initial navigation
        metrics = self.calculate_trajectory_metrics(self.current_trajectory)
        
        # Store the initial trajectory data
        trajectory_data = {
            'iteration': 0,  # Use 0 to indicate initial positioning
            'target': start_point,
            'trajectory': self.current_trajectory.copy(),
            'metrics': metrics
        }
        self.all_trajectories.append(trajectory_data)
        
        print(f"\nSuccessfully reached point {start_point}")
        print("\nInitial navigation metrics:")
        print(f"Path Length: {metrics['path_length']:.3f} meters")
        
        print("\nPress Enter to begin repeatability test...")
        input()
        
        return start_point


    def run_repeatability_test(self):
        """Execute the repeatability test"""
        print("\nNavigation points:")
        print(f"Point A: ({self.points['A']['x']}, {self.points['A']['y']})")
        print(f"Point B: ({self.points['B']['x']}, {self.points['B']['y']})")
        
        # Initialize position
        start_point = self.initialize_position()
        if not start_point:
            print("Failed to initialize position. Aborting test.")
            return
            
        iteration = 1
        current_target = 'B' if start_point == 'A' else 'A'  # Start with opposite point
        
        while True:
            print(f"\n[Iteration #{iteration}]")
            print("Press 'c' + Enter to continue or 'z' + Enter to finish")
            command = input().strip().lower()
            
            if command == 'z':
                if self.all_trajectories:
                    print("\nSaving navigation data...")
                    self.save_navigation_data()  # Save data before generating statistics
                    print("\nGenerating final statistics...")
                    self.print_final_statistics()
                    print("\nGenerating visualization...")
                    self.plot_results()
                return
            elif command != 'c':
                print("Invalid command. Use 'c' to continue or 'z' to finish")
                continue
            
            # Reset trajectory recording
            self.current_trajectory = []
            
            print(f"\nNavigating to point {current_target}...")
            goal_pose = self.create_goal_pose(self.points[current_target])
            self.navigator.goToPose(goal_pose)
            
            # Wait for navigation completion
            while not self.navigator.isNavComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            
            result = self.navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                print("Navigation successful")
                
                # Calculate metrics
                metrics = self.calculate_trajectory_metrics(self.current_trajectory)
                
                # Store trajectory data
                trajectory_data = {
                    'iteration': iteration,
                    'target': current_target,
                    'trajectory': self.current_trajectory.copy(),
                    'metrics': metrics
                }
                self.all_trajectories.append(trajectory_data)
                
                # Print iteration summary
                self.print_iteration_summary(iteration, current_target, metrics)
                
                # Switch target for next iteration
                current_target = 'A' if current_target == 'B' else 'B'
                iteration += 1
            else:
                print(f"Navigation failed with result: {result}")

def main(args=None):
    rclpy.init(args=args)
    tester = NavigationRepeatabilityTester()
    
    try:
        tester.run_repeatability_test()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        tester.navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()