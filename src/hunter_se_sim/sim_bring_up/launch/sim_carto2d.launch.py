import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.descriptions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
          
  gazebo_pkg_share_dir = get_package_share_directory('sim_gazebo')
  carto_pkg_share_dir = get_package_share_directory('sim_carto')
  urdf_pkg_share_dir = get_package_share_directory('hunter_se_description')

  urdf_file_path = os.path.join(urdf_pkg_share_dir, 'urdf/hunter_se.urdf.xacro')
  gazebo_world_path = os.path.join(gazebo_pkg_share_dir, 'world', 'house.world')
  rviz_config_dir = os.path.join(carto_pkg_share_dir, 'config', 'carto2d.rviz')
  

  robot_name = LaunchConfiguration('robot_name')
  start_x = LaunchConfiguration('start_x')
  start_y = LaunchConfiguration('start_y')
  start_z = LaunchConfiguration('start_z')
  start_yaw = LaunchConfiguration('start_yaw')
  
	# 是否使用仿真时间
  use_sim_time = LaunchConfiguration('use_sim_time')
  # 地图的分辨率
  resolution = LaunchConfiguration('resolution')
  # 地图的发布周期
  publish_period_sec = LaunchConfiguration('publish_period_sec')
  # 配置文件夹路径
  configuration_directory = LaunchConfiguration('configuration_directory')
  # 配置文件
  configuration_basename = LaunchConfiguration('configuration_basename')
  
  gazebo_simulator = IncludeLaunchDescription(
			PythonLaunchDescriptionSource([
					os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
			]),
			launch_arguments={
				'gui': 'true',
				'world': gazebo_world_path,
				'verbose': 'true'
			}.items()
	)
  robot_state_publisher = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		parameters=[
			{
				'robot_description': launch_ros.descriptions.ParameterValue(
				launch.substitutions.Command(['xacro ', urdf_file_path]), value_type=str),
				'use_sim_time': use_sim_time
			}
		],
		output='screen',
	)	      

  spawn_entity = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		arguments=[
				'-entity', robot_name,
				'-topic', 'robot_description',
				'-x', start_x,
				'-y', start_y,
				'-z', start_z,
				'-Y', start_yaw,
				'-timeout', '1000'
		],
		output='screen'
	)

  cartographer_node = Node(
      package='cartographer_ros',
      executable='cartographer_node',
      name='cartographer_node',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time}],
      remappings=[
        ('odom', 'odom'),],
      arguments=['-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename])

  cartographer_occupancy_grid_node = Node(
      package='cartographer_ros',
      executable='cartographer_occupancy_grid_node',
      name='cartographer_occupancy_grid_node',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time}],
      arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config_dir],
      parameters=[{'use_sim_time': use_sim_time}],
      output='screen')

  ld = LaunchDescription([
    DeclareLaunchArgument('use_sim_time', default_value='true'),
		DeclareLaunchArgument('robot_name', default_value='hunter_se'),
		DeclareLaunchArgument('start_x', default_value='0.0'),
		DeclareLaunchArgument('start_y', default_value='0.0'),
		DeclareLaunchArgument('start_z', default_value='0.0'),
		DeclareLaunchArgument('start_yaw', default_value='0.0'),
    DeclareLaunchArgument('resolution', default_value='0.05'),
    DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
    DeclareLaunchArgument('configuration_directory', default_value=os.path.join(carto_pkg_share_dir, 'config')),
    DeclareLaunchArgument('configuration_basename', default_value='carto_2d.lua')
	])
  ld.add_action(robot_state_publisher)
  ld.add_action(gazebo_simulator)
  ld.add_action(spawn_entity)
  ld.add_action(cartographer_node)
  ld.add_action(cartographer_occupancy_grid_node)
  ld.add_action(rviz_node)


  return ld