from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
 
    use_sim_time = LaunchConfiguration('use_sim_time')
   
    obstacle_node = Node(
            package='turtlemart',
            executable='naive_obstacle_avoidance.py'
         )


    joy_teleop_node = Node(
            package='turtlemart',
            executable='joy_teleop.py'
         )
    
   # trajectory_node= Node( package='turtlemart',executable='trajectory_visualization.py')


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        obstacle_node,
        joy_teleop_node,
        trajectory_node,

        # twist_stamper       
    ])
