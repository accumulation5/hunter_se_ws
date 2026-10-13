import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

""" 
    采用carto原生的地图保存方式
    1. 调用/write_state服务,保存.pbstream文件
    2. 根据录制的bag文件,启动cartographer_assets_writer节点转换为.ply格式3d地图格式
    3. 使用pcl_ply2pcd工具,将ply文件转换为pcd文件
"""

def generate_launch_description():
    pkg_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    urdf_pkg_dir = get_package_share_directory("hunter_se_description")

    map_name = LaunchConfiguration("map_name")

    # ---- 路径定义 ----
    map_src_stem = PathJoinSubstitution([pkg_src, 'map', map_name])
    config_dir   = PathJoinSubstitution([pkg_src, 'config'])
    bag_dir      = PathJoinSubstitution([pkg_src, 'bag', map_name])

    pbstream_file = PythonExpression(["'", map_src_stem, "' + '.pbstream'"])
    ply_file      = PythonExpression(["'", map_src_stem, "' + '.ply'"])
    pcd_file      = PythonExpression(["'", map_src_stem, "' + '.pcd'"])

    xacro_file = PathJoinSubstitution([urdf_pkg_dir, 'urdf', 'hunter_se.urdf.xacro'])
    tmp_urdf   = PythonExpression(["'/tmp/hunter_se_", map_name, ".urdf'"])

    # 前缀加 _ 防止和 lua 文件名粘连（my_3dmap + _ + points.ply = my_3dmap_points.ply）
    output_prefix = PythonExpression(["'", map_src_stem, "' + '_'"])

    # ---- /write_state YAML ----
    write_state_yaml = PythonExpression([
        "'{filename: \"' + '",
        map_src_stem,
        "' + '.pbstream\", include_unfinished_submaps: false}'"
    ])

    # ========== 步骤1: 保存 pbstream ==========
    save_pbstream = ExecuteProcess(
        cmd=['ros2', 'service', 'call',
             '/write_state',
             'cartographer_ros_msgs/srv/WriteState',
             write_state_yaml],
        name='save_pbstream',
        output='screen',
    )

    # ========== 步骤2: assets_writer → PLY ==========
    convert_to_ply = ExecuteProcess(
        cmd=['bash', '-c',
             PythonExpression([
                 "'xacro ' + '", xacro_file,
                 "' + ' > ' + '", tmp_urdf,
                 "' + ' && ros2 run cartographer_ros cartographer_assets_writer "
                 "-configuration_directory ' + '", config_dir,
                 "' + ' -configuration_basename assets_writer_3d.lua "
                 "-urdf_filename ' + '", tmp_urdf,
                 "' + ' -bag_filenames ' + '", bag_dir,
                 "' + ' -pose_graph_filename ' + '", pbstream_file,
                 "' + ' -output_file_prefix ' + '", output_prefix,
                 "' + ' && mv ' + '", output_prefix,
                 "' + 'points.ply' + ' ' + '", ply_file,
                 "' + ' && rm -f ' + '", tmp_urdf, "'"
             ]),
        ],
        name='convert_to_ply',
        output='screen',
    )

    # ========== 步骤3: PLY → PCD ==========
    convert_to_pcd = ExecuteProcess(
        cmd=['bash', '-c',
             PythonExpression([
                 "'pcl_ply2pcd ' + '", ply_file,
                 "' + ' ' + '", pcd_file, "'"
             ]),
        ],
        name='convert_to_pcd',
        output='screen',
    )

    # ========== 步骤4: PCD → PCD (Voxel Filter) ==========
    convert_to_pcd_filtered = ExecuteProcess(
        cmd=['bash', '-c',
             PythonExpression([
                 "'pcl_voxel_grid ' + '", pcd_file,
                 "' + ' ' + '", pcd_file,   # 原地覆盖
                 "' + ' -leaf 0.1,0.1,0.1'"
             ]),
        ],
        name='voxel_filter',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='3dmap',
            description='3D map name (also used as bag folder name)'),
        save_pbstream,

        RegisterEventHandler(
            OnProcessExit(
                target_action=save_pbstream,
                on_exit=[convert_to_ply],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=convert_to_ply,
                on_exit=[convert_to_pcd],
            )
        ),

        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=convert_to_pcd,
        #         on_exit=[convert_to_pcd_filtered],
        #     )
        # ),
    ])