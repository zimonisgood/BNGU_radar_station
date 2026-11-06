import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """配准流水线: /preprocess/valid -> icp -> /registration/icp -> background_filter -> /registration/foreground"""
    pkg_share = get_package_share_directory('registration')
    config = os.path.join(pkg_share, 'config', 'registration_params.yaml')
    # pcd_path = os.path.join(pkg_share, 'mesh', 'target.pcd')
    pcd_path = os.path.join(pkg_share, 'mesh', 'desk_south.pcd')

    icp_node = Node(
        package = 'registration',
        executable='icp_node',
        name = 'icp',
        output = 'screen',
        parameters=[
            config,
            {'target_file': pcd_path},
        ],
    )

    # 先验占据栅格与 ICP 共用同一份 PCD,保证坐标系一致
    background_filter_node = Node(
        package = 'registration',
        executable='background_filter_node',
        name = 'background_filter',
        output = 'screen',
        parameters=[
            config,
            {'prior_file': pcd_path},
        ],
    )

    return LaunchDescription([
        icp_node,
        background_filter_node,
    ])
