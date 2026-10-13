import os
import launch
import yaml
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
  
	bring_up_pkg_share_dir = get_package_share_directory('bring_up')       
	urdf_pkg_share_dir = get_package_share_directory('hunter_se_description')
	nav2_pkg_share_dir = get_package_share_directory('nav2')
	map_share_dir = get_package_share_directory("slam_carto")
	nav2_bringup_dir = get_package_share_directory('nav2_bringup')
	
	urdf_file_path = os.path.join(urdf_pkg_share_dir, 'urdf/hunter_se.urdf.xacro')
	rviz_config_dir = os.path.join(nav2_pkg_share_dir,'config','rviz.rviz')
	
	use_sim_time = LaunchConfiguration('use_sim_time')
	# nav2相关参数
	map_name = LaunchConfiguration('map_name')
	map_yaml_path = LaunchConfiguration('map_yaml_path')
	nav2_param_path = LaunchConfiguration('nav2_param_path')
	# 底盘相关参数
	port_name = LaunchConfiguration('port_name')
	odom_frame = LaunchConfiguration('odom_frame')
	base_frame = LaunchConfiguration('base_frame')
	odom_topic = LaunchConfiguration('odom_topic')
	robot_model = LaunchConfiguration('robot_model')
	publish_tf = LaunchConfiguration('publish_tf')
	simulated_robot = LaunchConfiguration('simulated_robot')
	sim_control_rate = LaunchConfiguration('control_rate')

  # vlp16驱动节点参数
	driver_params_file = os.path.join(bring_up_pkg_share_dir, 'config','VLP16', 'VLP16_driver_node_params.yaml')
	convert_params_file = os.path.join(bring_up_pkg_share_dir, 'config','VLP16', 'VLP16_transform_node_params.yaml')
	laserscan_params_file = os.path.join(bring_up_pkg_share_dir,'config','VLP16', 'VLP16_laserscan_node_params.yaml') 
	vlp16_device_ip = LaunchConfiguration('vlp16_device_ip')
	vlp16_frame_id = LaunchConfiguration('vlp16_frame_id')
	vlp16_port = LaunchConfiguration('vlp16_port')

	with open(convert_params_file, 'r') as f:
		convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
	convert_params['calibration'] = os.path.join(bring_up_pkg_share_dir, 'config','VLP16', 'VLP16db.yaml')

	with open(driver_params_file, 'r') as f:
			driver_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']
	driver_params['device_ip'] = vlp16_device_ip
	driver_params['frame_id'] = vlp16_frame_id
	driver_params['port'] = vlp16_port
	
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

	# 底盘节点
	hunter_base_node = launch_ros.actions.Node(
		package='hunter_base',
		executable='hunter_base_node',
		output='screen',
		emulate_tty=True,
		parameters=[{
					'use_sim_time': use_sim_time,
					'port_name': port_name,                
					'odom_frame': odom_frame,
					'base_frame': base_frame,
					'odom_topic_name': odom_topic,
					'simulated_robot': simulated_robot,
					'control_rate': sim_control_rate,
					'robot_model': robot_model,
					'publish_tf': publish_tf,
	}])	

	# 雷达节点
	velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
																									executable='velodyne_driver_node',
																									output='both',
																									parameters=[driver_params])



	velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
																										executable='velodyne_transform_node',
																										output='both',
																										parameters=[convert_params])


	velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
																										executable='velodyne_laserscan_node',
																										output='both',
																										parameters=[laserscan_params_file])



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
				'target_frame': '',
				'transform_tolerance': 0.1,
				'min_height': 0.1,		 #点云过滤：选择要转换的点云的最低高度
				'max_height': 1.0,
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

	# nav2节点
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
		DeclareLaunchArgument('use_sim_time', default_value='false'),
		DeclareLaunchArgument('robot_name', default_value='hunter_se'),
		DeclareLaunchArgument('map_name', default_value='map'),
		DeclareLaunchArgument('map_yaml_path', default_value=PathJoinSubstitution([map_share_dir, 'map', [map_name, '.yaml']])),
		DeclareLaunchArgument('nav2_param_path', default_value=os.path.join(nav2_pkg_share_dir,'param','nav2_param.yaml')),
		DeclareLaunchArgument('port_name', default_value='can0', description='CAN bus name, e.g. can0'),
    DeclareLaunchArgument('odom_frame', default_value='odom', description='Odometry frame id'),
    DeclareLaunchArgument('base_frame', default_value='base_link', description='Base link frame id'),
    DeclareLaunchArgument('odom_topic', default_value='odom', description='Odometry topic name'),
    DeclareLaunchArgument('robot_model', default_value='hunter_se', description='robot_model'),
    DeclareLaunchArgument('publish_tf', default_value='true', description='Whether to publish odom->base_footprint TF'),
    DeclareLaunchArgument('simulated_robot', default_value='false', description='Whether running with simulator'),
    DeclareLaunchArgument('control_rate', default_value='50', description='Simulation control loop update rate'),
		DeclareLaunchArgument('vlp16_device_ip', default_value='192.168.1.201', description='VLP16 LiDAR IP address'),
    DeclareLaunchArgument('vlp16_frame_id', default_value='lidar', description='VLP16 LiDAR frame id'),
    DeclareLaunchArgument('vlp16_port', default_value='2368', description='VLP16 LiDAR data port'),
	])
	ld.add_action(robot_state_publisher)
	ld.add_action(hunter_base_node)
	ld.add_action(velodyne_driver_node)
	ld.add_action(velodyne_transform_node)
	ld.add_action(velodyne_laserscan_node)
	ld.add_action(pointcloud_to_laserscan)
	ld.add_action(nav2_bringup_launch)
	ld.add_action(rviz_node)

	return ld