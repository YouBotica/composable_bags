from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory
import os

def get_share_file(package_name: str, *args: str) -> str:
    """Convert package-relative path to absolute path. Any additional args
    will be appended to the package_name, separated by '/'.

    Args:
        package_name (str): Package name.

    Returns:
        os.path: Absolute path.
    """
    return os.path.join(get_package_share_directory(package_name), *args)

def generate_launch_description():

    container = Node(
        name='bag_playback_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        output='both',
    )

    load_composable_nodes = LoadComposableNodes(
        target_container='bag_playback_container',
        composable_node_descriptions=[
            ComposableNode(
                package='bag_reading_cpp',
                plugin='lidar_components::PlaybackSpoofedNode',
                name='play_bag_spoofed_node',
                parameters=[
                    {'bag_path': "/home/pair-andres/Desktop/test_bag_play/rosbag2_lidar_2024-10-27_141152_1.mcap"},
                    {'loop': True}
                ],
                # namespace='',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='cloud_transform',
                plugin='cloud_transform::CloudTransformerAlt',
                name='alt_cloud_transform',
                parameters=[
                    get_share_file('cloud_transform', 'config', 'lidar_fusion.param.yaml'),
                    {'use_sim_time': True}, # FIXME: From launc_arg_dict
                ],
                namespace='/perception/post',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
    )

    delayed_load_composable_nodes = TimerAction(
        period=1.0,
        actions=[load_composable_nodes]
    )

    return LaunchDescription([container, delayed_load_composable_nodes])

