from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pointcloud_preprocessing'), 'config', 'field_filter.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='pointcloud_preprocessing', executable='field_filter_node',
             name='field_filter_node', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
