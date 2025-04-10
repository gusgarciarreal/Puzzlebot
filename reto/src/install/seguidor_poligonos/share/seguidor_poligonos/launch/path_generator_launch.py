from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seguidor_poligonos',
            executable='path_generator',
            name='path_generator',
            output='screen',
            parameters=[
                {'waypoints': '[(0,0),(1,0),(0.5,1)]'},
                {'mode': 'speeds'},
                {'linear_velocity': 0.2},
                {'angular_velocity': 0.5235987756},  # 30 degrees in radians
                {'total_time': 0.0}
            ]
        )
    ])