import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('roi')
    config = os.path.join(pkg_share, 'config', 'roi_params.yaml')
    rviz_raw_config = os.path.join(pkg_share, 'config', 'roi_raw.rviz')
    rviz_cropped_config = os.path.join(pkg_share, 'config', 'roi_cropped.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动 RViz2 可视化(两个窗口:原始点云 / 裁剪后点云)')

    roi_node = Node(
        package='roi',
        executable='roi_node',
        name='roi_filter',
        output='screen',
        parameters=[config],
    )

    # RViz2 窗口一:原始点云 /livox/lidar
    rviz_raw = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_raw',
        arguments=['-d', rviz_raw_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # RViz2 窗口二:裁剪后点云 /livox/lidar/roi
    rviz_cropped = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_cropped',
        arguments=['-d', rviz_cropped_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        roi_node,
        rviz_raw,
        rviz_cropped,
    ])
