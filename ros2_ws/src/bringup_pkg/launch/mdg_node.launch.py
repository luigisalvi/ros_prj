from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    ld = LaunchDescription()
    data_generator_node = Node(
        package='mdg_pkg',
        executable='mdg_node',
        name='data_generator'
    )
    ld.add_action(data_generator_node)
    return ld
