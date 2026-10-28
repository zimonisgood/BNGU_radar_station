import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """点云分割流水线(不含驱动与预处理):

    /preprocess/output
        -> euclidean_cluster (欧式聚类)  -> /segment/clusters
    """
    pkg_share = get_package_share_directory('segment')
    config = os.path.join(pkg_share, 'config', 'segment_params.yaml')

    # ---- 分割流水线节点(多节点架构)----
    euclidean_cluster_node = Node(
        package='segment',
        executable='euclidean_cluster_node',
        name='euclidean_cluster',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        euclidean_cluster_node,
    ])
