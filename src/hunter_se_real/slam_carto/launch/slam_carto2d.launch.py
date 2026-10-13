import os
import yaml
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.descriptions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
          
  bring_up_pkg_share_dir = get_package_share_directory('bring_up')       
  carto_pkg_share_dir = get_package_share_directory('slam_carto')
  urdf_pkg_share_dir = get_package_share_directory('hunter_se_description')
  urdf_file_path = os.path.join(urdf_pkg_share_dir, 'urdf/hunter_se.urdf.xacro')
  rviz_config_dir = os.path.join(carto_pkg_share_dir, 'config', 'carto2d.rviz')

  # 底盘驱动节点参数
  use_sim_time = LaunchConfiguration('use_sim_time')
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

  # slam_carto2d参数
  resolution = LaunchConfiguration('resolution')  # 地图的分辨率
  publish_period_sec = LaunchConfiguration('publish_period_sec') # 地图的发布周期
  configuration_directory = LaunchConfiguration('configuration_directory') # 配置文件夹路径
  configuration_basename = LaunchConfiguration('configuration_basename') # 配置文件
  
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

  # carto2d节点
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
    DeclareLaunchArgument('use_sim_time', default_value='false'),
		DeclareLaunchArgument('robot_name', default_value='hunter_se'),
    DeclareLaunchArgument('resolution', default_value='0.05'),
    DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
    DeclareLaunchArgument('configuration_directory', default_value=os.path.join(carto_pkg_share_dir, 'config')),
    DeclareLaunchArgument('configuration_basename', default_value='carto_2d.lua'),
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
  ld.add_action(cartographer_node)
  ld.add_action(cartographer_occupancy_grid_node)
  ld.add_action(rviz_node)

  return ld