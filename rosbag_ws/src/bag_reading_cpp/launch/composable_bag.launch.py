from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='bag_playback_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='bag_reading_cpp',
                plugin='bag_reading_cpp::PlaybackSpoofedNode',
                name='play_bag_spoofed_node',
                parameters=[
                    {'bag_path': "/home/pair-andres/Desktop/test_bag_play/rosbag2_lidar_2024-10-27_141152_1.mcap"},
                    {'loop': True}
                ]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
