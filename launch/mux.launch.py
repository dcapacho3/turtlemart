from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/cmd_vel_out',
            description='cmd vel output topic'),


    use_sim_time = LaunchConfiguration('use_sim_time')

    mux_params = os.path.join(get_package_share_directory('turtlemart'),'config','mux.yaml')

    mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[mux_params, {'use_sim_time': use_sim_time}],
            remappings=[
            ('/cmd_vel_out', '/cmd_vel_in') ]
         )

    # Joy node configuration
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='teleop_joy',
        parameters=[{
            'dev': '/dev/input/js0',  # Change jsX to the correct device
            'deadzone': 0.12
        }],
        respawn=True
    )

    
    
    speed_limit_node= Node( package='turtlemart',executable='speed_limit.py')
    teleop_node= Node( package='turtlemart',executable='joy_teleop.py')
    avoidance_node= Node( package='turtlemart',executable='naive_obstacle_avoidance.py')
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        mux_node,
        joy_node,
        speed_limit_node,
        teleop_node,
        avoidance_node,

        # twist_stamper       
    ])
