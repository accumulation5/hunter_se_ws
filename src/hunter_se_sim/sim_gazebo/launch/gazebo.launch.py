import os
import launch
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

	pkg_share_dir = get_package_share_directory('sim_gazebo')
	urdf_pkg_share_dir = get_package_share_directory('hunter_se_description')
	urdf_file_path = os.path.join(urdf_pkg_share_dir, 'urdf/hunter_se.urdf.xacro')
	gazebo_world_path = os.path.join(pkg_share_dir, 'world', 'house.world')
	default_rviz_config_path =os.path.join(pkg_share_dir, 'config/rviz.rviz')
	use_sim_time = LaunchConfiguration('use_sim_time')
	use_rviz = LaunchConfiguration('use_rviz')
	
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
				launch.substitutions.Command(['xacro ', urdf_file_path]),
				value_type=str),
				'use_sim_time': use_sim_time
			}
		],
		output='screen',
	)	      


	spawn_entity = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		arguments=[
				'-entity', LaunchConfiguration('robot_name'),
				'-topic', 'robot_description',
				'-x', LaunchConfiguration('start_x'),
				'-y', LaunchConfiguration('start_y'),
				'-z', LaunchConfiguration('start_z'),
				'-Y', LaunchConfiguration('start_yaw'),
				'-timeout', '1000'
		],
		output='screen'
	)

	rviz_node = Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			output='screen',
			arguments=['-d', LaunchConfiguration('rviz_config')],
			condition=IfCondition(use_rviz)
	)

	return LaunchDescription([
		DeclareLaunchArgument('use_sim_time', default_value='true'),
		DeclareLaunchArgument('gui', default_value='true'),
		DeclareLaunchArgument('start_x', default_value='0.0',
													description='X coordinate of starting position'),
		DeclareLaunchArgument('start_y', default_value='0.0',
													description='Y coordinate of starting position'),
		DeclareLaunchArgument('start_z', default_value='0.0',
													description='Z coordinate of starting position'),
		DeclareLaunchArgument('start_yaw', default_value='0.0',
													description='Yaw angle of starting orientation'),
		DeclareLaunchArgument('robot_name', default_value='',
													description='Name and prefix for this robot'),
		DeclareLaunchArgument(name='use_rviz', default_value='false',
																			description='Whether to start rviz'),                           
		DeclareLaunchArgument(name='rviz_config', default_value=str(default_rviz_config_path),
																			description='Absolute path to rviz config file'),
		spawn_entity,
		gazebo_simulator,
		robot_state_publisher,
		rviz_node																	
	])
