'''
  描述: Nav2 launch启动文件
'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    pkg_share_dir = get_package_share_directory('sim_nav2')
    map_share_dir = get_package_share_directory("sim_carto")
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    #=============================2.声明参数，获取配置文件路径===================================================
    # use_sim_time 这里要设置成true,因为gazebo是仿真环境，其时间是通过/clock话题获取，而不是系统时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(map_share_dir,'map','maze.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(pkg_share_dir,'param','nav2_param.yaml'))
    rviz_config_dir = os.path.join(pkg_share_dir,'config','rviz.rviz')

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
    
    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
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
    
    return LaunchDescription([
      pointcloud_to_laserscan,
      nav2_bringup_launch,
      rviz_node
    ])