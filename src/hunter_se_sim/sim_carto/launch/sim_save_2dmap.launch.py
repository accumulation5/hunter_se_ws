import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

""" 
    采用carto原生的地图保存方式
    1. 调用/write_state服务,保存.pbstream文件
    2. 启动cartographer_pbstream_to_ros_map节点转换为.pgm和.yaml格式的地图文件
"""

def generate_launch_description():
    pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    pkg_share_dir = get_package_share_directory("sim_carto")

    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')

    # 路径前缀: .../src/.../map/{map_name}
    map_src_stem = PathJoinSubstitution([pkg_src, 'map', map_name])

    # 带扩展名的完整路径（用 PythonExpression 拼接，避免 PathJoinSubstitution 插入 /）
    pbstream_file = PythonExpression(["'", map_src_stem, "' + '.pbstream'"])
    pgm_file      = PythonExpression(["'", map_src_stem, "' + '.pgm'"])
    yaml_file     = PythonExpression(["'", map_src_stem, "' + '.yaml'"])

    map_share_dir = PathJoinSubstitution([pkg_share_dir, 'map'])

    # 动态构造 YAML: {filename: "/path/map_name.pbstream", include_unfinished_submaps: true}
    write_state_yaml = PythonExpression([
        "'{filename: \"' + '",
        map_src_stem,
        "' + '.pbstream\", include_unfinished_submaps: true}'"
    ])

    # ==================== 步骤1: 调用 /write_state 保存 pbstream ====================
    save_pbstream = ExecuteProcess(
        cmd=['ros2', 'service', 'call',
             '/write_state',
             'cartographer_ros_msgs/srv/WriteState',
             write_state_yaml],
        name='save_pbstream',
        output='screen',
    )

    # ==================== 步骤2: pbstream → pgm + yaml ====================
    convert_map = ExecuteProcess(
        cmd=['ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
             '-pbstream_filename', pbstream_file,
             '-map_filestem', map_src_stem,
             '-resolution', resolution],
        name='convert_map',
        output='screen',
    )

    # ==================== 步骤3: 安装三件套到 share ====================
    install_to_share = ExecuteProcess(
        cmd=['install', '-D', '-m', '644', '-t', map_share_dir,
             pbstream_file,
             pgm_file,
             yaml_file],
        name='install_to_share',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='map',
            description='Map name which you want to save'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of the output grid map (m/pixel)'),

        # ---- 链式执行: save → convert → install ----
        save_pbstream,

        RegisterEventHandler(
            OnProcessExit(
                target_action=save_pbstream,
                on_exit=[convert_map],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=convert_map,
                on_exit=[install_to_share],
            )
        ),
    ])