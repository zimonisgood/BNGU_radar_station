import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """点云预处理流水线(不含驱动):

    /livox/lidar
        -> roi_filter           (ROI 立方体裁剪)      -> /preprocess/roi
        -> invalid_removal      (去除 NaN/Inf 无效点) -> /preprocess/valid
        -> statistical_outlier  (统计离群点去除)       -> /preprocess/output
    """
    pkg_share = get_package_share_directory('preprocess')
    config = os.path.join(pkg_share, 'config', 'preprocess_params.yaml')
    rviz_raw_config = os.path.join(pkg_share, 'config', 'roi_raw.rviz')
    rviz_cropped_config = os.path.join(pkg_share, 'config', 'roi_cropped.rviz')
    rviz_filtered_config = os.path.join(pkg_share, 'config', 'roi_filtered.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动 RViz2 可视化(三个窗口:原始 / ROI 裁剪 / 统计离群去除后)')

    # ---- 预处理流水线节点(多节点架构)----
    roi_node = Node(
        package='preprocess',
        executable='roi_node',
        name='roi_filter',
        output='screen',
        parameters=[config],
    )

    invalid_removal_node = Node(
        package='preprocess',
        executable='invalid_removal_node',
        name='invalid_removal',
        output='screen',
        parameters=[config],
    )

    statistical_outlier_node = Node(
        package='preprocess',
        executable='statistical_outlier_node',
        name='statistical_outlier',
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

    # RViz2 窗口二:ROI 裁剪后点云 /preprocess/roi
    rviz_cropped = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_roi',
        arguments=['-d', rviz_cropped_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # RViz2 窗口三:统计离群点去除后点云 /preprocess/output
    rviz_filtered = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_output',
        arguments=['-d', rviz_filtered_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        roi_node,
        invalid_removal_node,
        statistical_outlier_node,
        rviz_raw,
        rviz_cropped,
        rviz_filtered,
    ])
