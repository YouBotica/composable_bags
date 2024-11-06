# save this as `composable_example_launch.py`
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='example_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='demo_nodes_cpp',
                plugin='demo_nodes_cpp::Talker',
                name='talker',
                namespace='example'
            ),
            ComposableNode(
                package='demo_nodes_cpp',
                plugin='demo_nodes_cpp::Listener',
                name='listener',
                namespace='example'
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
