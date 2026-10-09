import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('roi')
    rviz_raw_config = os.path.join(pkg_share, 'config', 'roi_raw.rviz')
    rviz_cropped_config = os.path.join(pkg_share, 'config', 'roi_cropped.rviz')
    roi_config = os.path.join(pkg_share, 'config', 'roi_params.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动 RViz2 可视化(两个窗口:原始点云 / 裁剪后点云)')

    # 1) 启动 Livox MID-70 驱动(xfer_format=0 -> /livox/lidar 发布 PointCloud2)
    livox_launch_dir = os.path.join(
        get_package_share_directory('livox_ros2_driver'), 'launch')
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_launch_dir, 'livox_lidar_launch.py')))

    # 2) 启动 ROI 裁剪节点
    roi_node = Node(
        package='roi',
        executable='roi_node',
        name='roi_filter',
        output='screen',
        parameters=[roi_config],
    )

    # 3) RViz2 窗口一:原始点云 /livox/lidar(彩虹按 intensity 上色)
    rviz_raw = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_raw',
        arguments=['-d', rviz_raw_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # 4) RViz2 窗口二:裁剪后点云 /livox/lidar/roi(彩虹按 intensity 上色)
    rviz_cropped = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_cropped',
        arguments=['-d', rviz_cropped_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

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

    return LaunchDescription([
        rviz_arg,
        livox_driver,
        roi_node,
        rviz_raw,
        rviz_cropped,
        flip_tf,
    ])
