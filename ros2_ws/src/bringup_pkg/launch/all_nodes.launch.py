# my_launch_file.launch.py
from ros2_ws.src.bringup_pkg.bringup_pkg import LaunchDescription
from launch_ros.actions import Node # type: ignore

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='data_generator_pkg',
            executable='data_generator_node',
            name='data_generator'
        ),
        Node(
            package='pid_controller_pkg',
            executable='pid_controller_node',
            name='pid_controller'
        ),
        Node(
            package='visualizer_pkg',
            executable='visualizer_node',
            name='visualizer'
        ),
    ])
