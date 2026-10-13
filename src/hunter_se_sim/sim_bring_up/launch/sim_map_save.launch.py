# 使用nav2_map_server的map_saver_cli来保存地图

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    sim_bring_up_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    pkg_src = os.path.join(os.path.dirname(sim_bring_up_dir), 'sim_carto')
    pkg_share_dir = get_package_share_directory("sim_carto")

    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration('use_sim_time')
    # 栅格占据/空闲阈值
    free_thresh = LaunchConfiguration("free_thresh")
    occupied_thresh = LaunchConfiguration("occupied_thresh")

    map_save_src_path = PathJoinSubstitution([pkg_src, 'map', map_name])
    map_save_share_path = PathJoinSubstitution([pkg_share_dir, 'map', map_name])

    # 保存到src目录
    saver_src_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver_src',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'save_map_timeout': 10000.0,
            'free_thresh':free_thresh,
            'occupied_thresh': occupied_thresh,
        }],
        arguments=[
            '-f', map_save_src_path,
            '--ros-args', '-r', '/map:=/map',
        ],
    )

    # 保存到share目录
    saver_share_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver_share',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'save_map_timeout': 10000.0,
            'free_thresh':free_thresh,
            'occupied_thresh': occupied_thresh,
        }],
        arguments=[
            '-f', map_save_share_path,
            '--ros-args', '-r', '/map:=/map',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='map',
            description='map name which you want to save'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use_sim_time'),
        # 栅格地图占据/空闲阈值
        DeclareLaunchArgument(
            'free_thresh',
            default_value='0.25',
            description='free threshold for src map'),
        DeclareLaunchArgument(
            'occupied_thresh',
            default_value='0.65',
            description='occupied threshold for src map'),
        saver_src_node,
        saver_share_node,
    ])
