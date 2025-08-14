# Autor: David Capacho Parra
# Fecha: Febrero 2025
# Descripción: Archivo de lanzamiento para visualización del robot SARA en simulación
# Implementa la configuración de lanzamiento para la visualización básica del robot SARA
# en un entorno Gazebo simulado, permitiendo la observación de los marcos de coordenadas
# (TF) y el modelo URDF. Este lanzador proporciona una plataforma para demostrar
# el movimiento básico del robot y verificar la correcta configuración cinemática
# sin los sistemas completos de navegación o SLAM.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Configuración de rutas a archivos y carpetas
  # Define las ubicaciones de los recursos para la simulación
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package='turtlemart').find('turtlemart')
  default_model_path = os.path.join(pkg_share, 'models/turtlemart.urdf.xacro')  
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
  world_file_name = 'turtlemart_world/Supermarket.world'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)
  
  # Variables de configuración específicas para la simulación
  # Controlan aspectos visuales y funcionales del entorno simulado
  headless = LaunchConfiguration('headless')
  model = LaunchConfiguration('model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')

  # Declaración de los argumentos de lanzamiento
  # Permite la configuración dinámica del comportamiento de la visualización
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
   
  # Especificación de las acciones para el lanzamiento
  # Define los componentes que se iniciarán y su configuración

  # Iniciar el servidor de Gazebo
  # Gestiona la simulación física del entorno y el robot SARA
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())

  # Iniciar el cliente de Gazebo
  # Proporciona la interfaz visual para la simulación    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    

  # Publicador del estado del robot SARA
  # Publica las transformaciones (TF) entre los diferentes eslabones del robot
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', model])}],
    arguments=[default_model_path])

  # Lanzamiento de RViz para visualización
  # Permite observar el modelo 3D del robot SARA y sus transformaciones
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
  
  # Creación de la descripción de lanzamiento y población
  ld = LaunchDescription()

  # Declaración de las opciones de lanzamiento
  # Añade todos los argumentos configurables
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  # Adición de las acciones a ejecutar
  # Configura el orden de inicio de los componentes para la visualización
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)

  return ld