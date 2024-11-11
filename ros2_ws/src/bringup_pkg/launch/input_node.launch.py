from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    ld = LaunchDescription()
    input_node = Node(
        package='input_pkg',
        executable='input_node',
        name='input'
    )
    ld.add_action(input_node)
    return ld