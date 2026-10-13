import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    
    sim_bring_up_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    pkg_src = os.path.join(os.path.dirname(sim_bring_up_dir), 'sim_carto')

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