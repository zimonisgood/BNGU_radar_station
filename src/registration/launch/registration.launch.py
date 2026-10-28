import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('registration')
    config = os.path.join(pkg_share, 'config', 'registration_params.yaml')
    stl_path = os.path.join(pkg_share, 'mesh', 'field.stl')

    icp_node = Node(
        package = 'registration',
        executable='icp_node',
        name = 'icp',
        output = 'screen',
        parameters=[
            config,
            {'stl_file': stl_path},
        ],
    )

    return LaunchDescription([
        icp_node,
    ])