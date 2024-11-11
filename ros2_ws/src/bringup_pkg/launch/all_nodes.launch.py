from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    ld = LaunchDescription()
    data_generator_node = Node(
        package='mdg_pkg',
        executable='mdg_node',
        name='data_generator'
    )
    pid_controller_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='pid_controller'
    )
    visualizer_node = Node(
        package='visualizer_pkg',
        executable='visualizer_node',
        name='visualizer'
    )

    input_node = Node(
        package='input_pkg',
        executable='input_node',
        name='input'
    )
    ld.add_action(data_generator_node)
    ld.add_action(pid_controller_node)
    ld.add_action(visualizer_node)
    ld.add_action(input_node)
    return ld
