from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    ld = LaunchDescription()
    pid_controller_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='pid_controller'
    )
    ld.add_action(pid_controller_node)
    return ld
