# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Archivo de lanzamiento para SLAM utilizando cartographer en el robot SARA simulado
# Implementa la configuración de lanzamiento para un sistema de mapeo y localización
# simultánea (SLAM) utilizando Cartographer en el robot SARA simulado en Gazebo.
# La arquitectura permite la integración de sensores, visualización en RViz,
# generación de mapas de ocupación y simulación del entorno en un supermercado virtual.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():


    
    # Configuración de rutas a archivos y carpetas necesarios para la simulación
    # Define las ubicaciones de los modelos, configuraciones y mundos virtuales
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package='turtlemart').find('turtlemart')
    default_model_path = os.path.join(pkg_share, 'models/turtlemart.urdf.xacro')  
    default_rviz_config_path = os.path.join(get_package_share_directory('turtlemart'),
                                   'rviz', 'cartographer_config.rviz')
    world_file_name = 'turtlemart_world/Supermarket.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # Configuración específica para Cartographer
    # Define los directorios y parámetros para el algoritmo SLAM
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlemart')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'params'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer_params.lua')

    # Configuración de parámetros para el mapa de ocupación
    # Define la resolución y frecuencia de publicación del mapa generado
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    
    # Variables de configuración específicas para la simulación
    headless = LaunchConfiguration('headless')
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')



    # Declaración de argumentos de lanzamiento
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_cartographer_dir_cmd = DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load')
    
    declare_cartographer_param_cmd = DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer')
    
    declare_resolution = DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid')

    decalare_publish_period = DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period')

    


    # Especificación de acciones para el lanzamiento
    # Iniciar el servidor de Gazebo
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())

    # Iniciar el cliente de Gazebo
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))


    # Se suscribe a los estados de las articulaciones y publica la pose 3D de cada eslabón
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', model])}],
        arguments=[default_model_path])

    # Lanzamiento de RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    # Lanzamiento del nodo Cartographer para SLAM
    start_cartographer_cmd = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename])
    
    # Lanzamiento del generador de mapas de ocupación
    start_ocupancy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'occupancy_grid.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time, 
            'resolution': resolution,
            'publish_period_sec': publish_period_sec
        }.items(),
    )

    # Creación de la descripción de lanzamiento y población de acciones
    ld = LaunchDescription()

    # Declaración de las opciones de lanzamiento
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_cartographer_dir_cmd)
    ld.add_action(declare_cartographer_param_cmd)
    ld.add_action(declare_resolution)
    ld.add_action(decalare_publish_period)

    # Adición de las acciones a ejecutar
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_cartographer_cmd)
    ld.add_action(start_ocupancy)

    return ld
