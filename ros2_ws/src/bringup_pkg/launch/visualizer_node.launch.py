from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    ld = LaunchDescription()
    visualizer_node = Node(
        package='visualizer_pkg',
        executable='visualizer_node',
        name='visualizer'
    )
    ld.add_action(visualizer_node)
    return ld
