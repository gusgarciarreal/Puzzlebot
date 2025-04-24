from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='puzzlebot_pid_controller',
            executable='pid_controller_node',
            name='pid_controller',
            parameters=['config/params.yaml'],
            output='screen'
        )
    ])
