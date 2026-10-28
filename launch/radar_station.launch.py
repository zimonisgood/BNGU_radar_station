import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """BNGU 雷达站总入口 launch(编排所有功能包,不隶属任何单个功能包)。

    本文件位于工作区顶层 launch/ 目录,用路径方式启动:
        ros2 launch launch/radar_station.launch.py
    这样读取的始终是源文件本身,无需 colcon build / install 同步。

    一次性拉起:Livox 驱动 + preprocess 预处理 + segment 分割 + RViz2(全链路)。

    数据流:
      /livox/lidar
        -> roi_filter          (ROI 立方体裁剪)      -> /preprocess/roi
        -> invalid_removal     (去除 NaN/Inf 无效点) -> /preprocess/valid
        -> statistical_outlier (统计离群点去除)       -> /preprocess/output
        -> euclidean_cluster   (欧式聚类)            -> /segment/clusters

    每个 RViz 窗口都有独立的开关参数,临时隐藏某个窗口时在命令行传 `xxx:=false`
    即可,无需改动本文件(例如: `rviz_clusters:=false`)。
    """
    preprocess_share = get_package_share_directory('preprocess')
    segment_share = get_package_share_directory('segment')
    registration_share = get_package_share_directory('registration')

    preprocess_config = os.path.join(preprocess_share, 'config', 'preprocess_params.yaml')
    segment_config = os.path.join(segment_share, 'config', 'segment_params.yaml')
    registration_config = os.path.join(registration_share, 'config', 'registration_params.yaml')
    registration_stl = os.path.join(registration_share, 'mesh', 'field.stl')

    rviz_raw_config = os.path.join(preprocess_share, 'config', 'roi_raw.rviz')
    rviz_cropped_config = os.path.join(preprocess_share, 'config', 'roi_cropped.rviz')
    rviz_filtered_config = os.path.join(preprocess_share, 'config', 'roi_filtered.rviz')
    rviz_clusters_config = os.path.join(segment_share, 'config', 'clusters.rviz')

    # ---- 每个 RViz 窗口一个独立开关 ----
    rviz_args = [
        DeclareLaunchArgument('rviz_raw', default_value='false',
                              description='RViz 窗口:原始点云 /livox/lidar'),
        DeclareLaunchArgument('rviz_roi', default_value='true',
                              description='RViz 窗口:ROI 裁剪 /preprocess/roi'),
        DeclareLaunchArgument('rviz_output', default_value='true',
                              description='RViz 窗口:预处理输出 /preprocess/output'),
        DeclareLaunchArgument('rviz_clusters', default_value='true',
                              description='RViz 窗口:欧式聚类 /segment/clusters'),
    ]

    # ---- 1) Livox MID-70 驱动(xfer_format=0 -> /livox/lidar 发布 PointCloud2)----
    livox_launch_dir = os.path.join(
        get_package_share_directory('livox_ros2_driver'), 'launch')
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_launch_dir, 'livox_lidar_launch.py')))

    # 雷达倒置:绕 X 轴翻转 180°,base_link 为转正后的坐标系
    flip_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_flip_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '3.14159', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'livox_frame',
        ],
    )

    # ---- 2) preprocess 预处理流水线节点 ----
    roi_node = Node(
        package='preprocess', executable='roi_node', name='roi_filter',
        output='screen', parameters=[preprocess_config],
    )
    invalid_removal_node = Node(
        package='preprocess', executable='invalid_removal_node', name='invalid_removal',
        output='screen', parameters=[preprocess_config],
    )
    statistical_outlier_node = Node(
        package='preprocess', executable='statistical_outlier_node', name='statistical_outlier',
        output='screen', parameters=[preprocess_config],
    )

    # ---- 3) segment 分割流水线节点(输入为 preprocess 的最终输出 /preprocess/output)----
    euclidean_cluster_node = Node(
        package='segment', executable='euclidean_cluster_node',
        name='euclidean_cluster',
        output='screen', parameters=[segment_config],
    )

    icp_node = Node(
        package  = 'registration', executable = 'icp_node',
        name = "icp",
        output = 'screen',
        parameters = [registration_config, {'stl_file': registration_stl}],
    )

    # ---- 4) RViz2 窗口(每个用各自的开关参数控制)----
    rviz_raw = Node(
        package='rviz2', executable='rviz2', name='rviz2_raw',
        arguments=['-d', rviz_raw_config], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz_raw')),
    )
    rviz_roi = Node(
        package='rviz2', executable='rviz2', name='rviz2_roi',
        arguments=['-d', rviz_cropped_config], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz_roi')),
    )
    rviz_output = Node(
        package='rviz2', executable='rviz2', name='rviz2_output',
        arguments=['-d', rviz_filtered_config], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz_output')),
    )
    rviz_clusters = Node(
        package='rviz2', executable='rviz2', name='rviz2_clusters',
        arguments=['-d', rviz_clusters_config], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz_clusters')),
    )

    return LaunchDescription([
        *rviz_args,
        livox_driver,
        flip_tf,
        roi_node,
        invalid_removal_node,
        statistical_outlier_node,
        euclidean_cluster_node,
        icp_node,
        rviz_raw,
        rviz_roi,
        rviz_output,
        rviz_clusters,
    ])
