import os
import launch
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

	gazebo_pkg_share_dir = get_package_share_directory('sim_gazebo')
	urdf_pkg_share_dir = get_package_share_directory('hunter_se_description')
	nav2_pkg_share_dir = get_package_share_directory('sim_nav2')
	map_share_dir = get_package_share_directory("sim_carto")
	nav2_bringup_dir = get_package_share_directory('nav2_bringup')
	
	urdf_file_path = os.path.join(urdf_pkg_share_dir, 'urdf/hunter_se.urdf.xacro')
	gazebo_world_path = os.path.join(gazebo_pkg_share_dir, 'world', 'house.world')
	rviz_config_dir = os.path.join(nav2_pkg_share_dir,'config','rviz.rviz')
	
	use_sim_time = LaunchConfiguration('use_sim_time')
	robot_name = LaunchConfiguration('robot_name')
	start_x = LaunchConfiguration('start_x')
	start_y = LaunchConfiguration('start_y')
	start_z = LaunchConfiguration('start_z')
	start_yaw = LaunchConfiguration('start_yaw')
	map_name = LaunchConfiguration('map_name')
	map_yaml_path = LaunchConfiguration('map_yaml_path')
	nav2_param_path = LaunchConfiguration('nav2_param_path')
	

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
				launch.substitutions.Command(['xacro ',urdf_file_path]),
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

	# 3d点云转2d激光
	pointcloud_to_laserscan = Node(
		package='pointcloud_to_laserscan', 
		executable='pointcloud_to_laserscan_node',
		name='pointcloud_to_laserscan',
		remappings=[
			('cloud_in', '/velodyne_points'),
			('scan', '/scan')
		],
		parameters=[{
				# 'target_frame': '',
				'transform_tolerance': 0.1,
				'min_height': 0.1,		 #点云过滤：选择要转换的点云的最低高度
				'max_height': 0.5,
				'angle_min': -3.1415926,  # -M_PI 要转换的点云的角度范围最小值
				'angle_max': 3.1415926,  
				# (2*M_PI/360.0）* x 转换后激光的角度分辨率，根据点云角度分辨率转为弧度；这里的角度分辨率是1度因此转换为弧度制为0.01745
				'angle_increment': 0.01745, 
				# (1/10)/(360/1) 即同一个采样周期下每一个激光点的采样时间间隔，即（1s/雷达扫描频率）/（360/角度分辨率（单位是度）） 
				# 'time_increment': 0.000278, 
				'scan_time': 0.1,	# 1/频率	#即采样周期，扫描一圈所花的时间，1s/扫描频率
				'range_min': 0.1,	# 即雷达的最近可识别距离，一般按照雷达的最近视野盲区设置
				'range_max': 50.0, # 即雷达最大的量程
				'use_inf': True,	# 启用inf替换那些超出雷达扫描范围的值
				'inf_epsilon': 1.0
		}],
  )

	nav2_bringup_launch = IncludeLaunchDescription(
					PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
					launch_arguments={
							'map': map_yaml_path,
							'use_sim_time': use_sim_time,
							'params_file': nav2_param_path}.items(),
			)
	
	rviz_node =  Node(
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
		DeclareLaunchArgument('map_name', default_value='map'),
		DeclareLaunchArgument('map_yaml_path', default_value=PathJoinSubstitution([map_share_dir, 'map', [map_name, '.yaml']])),
		DeclareLaunchArgument('nav2_param_path', default_value=os.path.join(nav2_pkg_share_dir,'param','nav2_param.yaml'))
	])
	ld.add_action(robot_state_publisher)
	ld.add_action(gazebo_simulator)
	ld.add_action(spawn_entity)
	ld.add_action(pointcloud_to_laserscan)
	ld.add_action(nav2_bringup_launch)
	ld.add_action(rviz_node)

	return ld