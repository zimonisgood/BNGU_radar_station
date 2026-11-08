import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """离线调试欧式聚类：/livox/lidar -> debug ROI -> /preprocess/debug_roi -> 聚类。

    只启动 ROI、聚类、静态 TF 和 RViz；不启动 ICP、背景过滤、SOR。
    离线调试：
        ros2 launch launch/cluster_debug.launch.py bag_path:=target_01
    实时调试（先单独启动 Livox 驱动）：
        ros2 launch launch/cluster_debug.launch.py rviz:=true
    注意：不要同时启动 radar_station.launch.py，否则聚类节点会重复。

    调参示例：
        ros2 launch launch/cluster_debug.launch.py bag_path:=target_01 \
            cluster_tolerance:=0.2 min_cluster_size:=20 max_cluster_size:=10000
        # 地面/墙没裁干净时，收紧 debug ROI
        ros2 launch launch/cluster_debug.launch.py bag_path:=target_01 \
            debug_z_min:=-1.0 debug_z_max:=1.2

    注意：原始场景（未去背景）在 0.4 m 下会整体连通成一个大簇，
    调试先用 0.2 m + 较大的 max_cluster_size，让聚类节点有输出；
    等接入背景过滤、只留机器人前景后，再把 tolerance 调大。
    """
    segment_share = get_package_share_directory('segment')
    rviz_config = os.path.join(segment_share, 'config', 'clusters.rviz')

    # ---- 离线 bag 播放 ----
    bag_path = LaunchConfiguration('bag_path')
    bag_rate = LaunchConfiguration('bag_rate')
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--rate', bag_rate],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", bag_path, "' != ''"])
        ),
    )

    # ---- 调试 ROI：直接从 /livox/lidar 裁剪，输出 /preprocess/debug_roi ----
    roi_debug = Node(
        package='preprocess', executable='roi_node', name='roi_filter_debug',
        output='screen',
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/preprocess/debug_roi',
            'x_min': ParameterValue(
                LaunchConfiguration('debug_x_min'), value_type=float),
            'x_max': ParameterValue(
                LaunchConfiguration('debug_x_max'), value_type=float),
            'y_min': ParameterValue(
                LaunchConfiguration('debug_y_min'), value_type=float),
            'y_max': ParameterValue(
                LaunchConfiguration('debug_y_max'), value_type=float),
            'z_min': ParameterValue(
                LaunchConfiguration('debug_z_min'), value_type=float),
            'z_max': ParameterValue(
                LaunchConfiguration('debug_z_max'), value_type=float),
            'negative': False,
        }],
    )

    # ---- 聚类：输入固定为 /preprocess/debug_roi ----
    cluster = Node(
        package='segment', executable='euclidean_cluster_node',
        name='euclidean_cluster',
        output='screen',
        parameters=[{
            'input_topic': '/preprocess/debug_roi',
            'output_topic': '/segment/clusters',
            'cluster_tolerance': ParameterValue(
                LaunchConfiguration('cluster_tolerance'), value_type=float),
            'min_cluster_size': ParameterValue(
                LaunchConfiguration('min_cluster_size'), value_type=int),
            'max_cluster_size': ParameterValue(
                LaunchConfiguration('max_cluster_size'), value_type=int),
            'log_cluster_centers': ParameterValue(
                LaunchConfiguration('log_cluster_centers'), value_type=bool),
        }],
    )

    # 雷达倒置：base_link -> livox_frame 绕 X 轴翻转 180°
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

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2_clusters_debug',
        arguments=['-d', rviz_config], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path', default_value='',
            description='离线 bag 路径；为空则不播放 bag（需要外部驱动）'),
        DeclareLaunchArgument(
            'bag_rate', default_value='0.5',
            description='bag 播放速率'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'debug_x_min', default_value='0.5',
            description='调试 ROI 的 x 最小值（米，livox_frame）'),
        DeclareLaunchArgument(
            'debug_x_max', default_value='5.0',
            description='调试 ROI 的 x 最大值（米，livox_frame）'),
        DeclareLaunchArgument(
            'debug_y_min', default_value='-2.0',
            description='调试 ROI 的 y 最小值（米，livox_frame）'),
        DeclareLaunchArgument(
            'debug_y_max', default_value='2.0',
            description='调试 ROI 的 y 最大值（米，livox_frame）'),
        DeclareLaunchArgument(
            'debug_z_min', default_value='-1.2',
            description='调试 ROI 的 z 最小值（米，livox_frame）'),
        DeclareLaunchArgument(
            'debug_z_max', default_value='1.5',
            description='调试 ROI 的 z 最大值（米，livox_frame）'),
        DeclareLaunchArgument(
            'cluster_tolerance', default_value='0.2',
            description='欧式聚类距离阈值（米）；原始场景 0.4 会整体连通'),
        DeclareLaunchArgument(
            'min_cluster_size', default_value='30',
            description='最小聚类点数'),
        DeclareLaunchArgument(
            'max_cluster_size', default_value='10000',
            description='最大聚类点数；原始场景常合并成 8–9k 点的大簇'),
        DeclareLaunchArgument(
            'log_cluster_centers', default_value='true',
            description='每帧打印每个簇的质心'),
        flip_tf,
        roi_debug,
        cluster,
        rviz,
        # 给节点留 2 秒发现时间，再播放 bag，避免开头几帧被丢掉
        TimerAction(period=2.0, actions=[bag_play]),
    ])
