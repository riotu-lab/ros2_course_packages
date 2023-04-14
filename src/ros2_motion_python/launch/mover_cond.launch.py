from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create a LaunchConfiguration object to access launch arguments
    launch_arg = LaunchConfiguration('start_mover')

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            # Execute this node only if 'start_mover' is set to 'true'
        ),
        Node(
            package='ros2_motion_python',
            executable='mover',
            name='mover',
            output='screen',
            # Execute this node only if 'start_mover' is set to 'false' or not set at all
            #condition=UnlessCondition(launch_arg)
            condition=IfCondition(launch_arg)
        ),
        # Declare the 'start_mover' launch argument with a default value of 'false'
        DeclareLaunchArgument(
            'start_mover',
            default_value='false',
            description='A boolean value to decide if the mover node should be launched'
        )
    ])
