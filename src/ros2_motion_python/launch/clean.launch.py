from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='ros2_motion_python',
            executable='cleaner',
            name='cleaner',
            output='screen'
        )
    ])