import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess

"""
    carto3d建图-ros2bag录制
"""

def generate_launch_description():
    pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    bag_name = LaunchConfiguration("bag_name")
    bag_output = PathJoinSubstitution([pkg_src, 'bag', bag_name])

    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '-o', bag_output,
             '/velodyne_points',
             '/imu/data',
             '/odom',
             '/tf',
             '/tf_static'],
        name='record_bag',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_name',
            default_value='carto3d_bag',
            description='Bag file name (saved to sim_carto/bag/)'),
        record_bag,
    ])