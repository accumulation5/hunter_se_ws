import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # ====================== 读取参数 ======================
    carto_pkg_share_dir = get_package_share_directory('slam_carto')
    rviz_config_dir = os.path.join(carto_pkg_share_dir, 'config', 'octomap.rviz')

	# 是否使用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time')
    pcd_path = LaunchConfiguration('pcd_file_path')
    resolution = LaunchConfiguration('octomap_resolution')
    height_min = LaunchConfiguration('height_min')
    height_max = LaunchConfiguration('height_max')
    map_frame = LaunchConfiguration('map_frame')
    cloud_topic = LaunchConfiguration('pointcloud_topic')
    pcd_frame_id = LaunchConfiguration('pcd_frame_id')

    # ====================== 节点定义 ======================
    # 1. 静态TF：解决PCD点云默认frame与地图frame不匹配问题（恒等变换）
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cloud_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', map_frame, 'base_link']
    )

    # 2. PCD点云发布节点
    pcd_publisher_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='pcd_loader',
        parameters=[{
            'file_name': pcd_path,
            'tf_frame': pcd_frame_id,
            'rate': 1.0,
        }],
        remappings=[
            ('/cloud_pcd', cloud_topic)
        ]
    )

    # 3. OctoMap 服务节点：核心构建与2D投影
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'resolution': resolution,
            'frame_id': map_frame,
            # 2D投影参数配置
            'occupancy_min_z': height_min,     
            'occupancy_max_z': height_max,     
            'publish_free_space': True,
            # 概率模型参数
            'sensor_model.hit': 0.7,          
            'sensor_model.miss': 0.4,         
            'sensor_model.min': 0.12,        
            'sensor_model.max': 0.97,        
            'sensor_model.max_range': 30.0,     
            'latch': True,                     
        }],
        remappings=[
            ('cloud_in', cloud_topic),       
            ('projected_map', '/map'),        
            ('octomap_binary', '/octomap_binary'),
            ('octomap_full', '/octomap_full'),
        ]
    )

    rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config_dir],
      parameters=[{'use_sim_time': use_sim_time}],
      output='screen'
    )

    ld = LaunchDescription([
        # 全局参数声明
        DeclareLaunchArgument('use_sim_time', default_value='false', description='是否使用仿真时间'),
        DeclareLaunchArgument('pcd_file_path', 
            default_value='/home/amass/amass_ws/src/hunter_se_real/slam_carto/map/carto3d_map.pcd',
            description='输入的3D点云PCD文件绝对路径'),
        DeclareLaunchArgument('octomap_resolution', default_value='0.05', description='八叉树体素分辨率,单位:米'),
        DeclareLaunchArgument('height_min', default_value='0.1', description='2D投影高度下限(map坐标系Z轴,单位:米)'),
        DeclareLaunchArgument('height_max', default_value='0.4', description='2D投影高度上限(map坐标系Z轴,单位:米)'),
        DeclareLaunchArgument('map_frame', default_value='map', description='地图参考坐标系'),
        DeclareLaunchArgument('pointcloud_topic', default_value='velodyne_points', description='输入点云话题名'),
        DeclareLaunchArgument('pcd_frame_id', default_value='map', description='PCD点云的frame_id'),
    ])
    # ld.add_action(static_tf_node)
    ld.add_action(pcd_publisher_node)
    ld.add_action(octomap_server_node)
    ld.add_action(rviz_node)
    return ld