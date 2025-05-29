#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to start the complete line follower pipeline,
    including camera preprocessing, line detection, traffic light detection,
    and speed moderation.
    """

    # Node for camera preprocessing
    preprocess_cam_node = Node(
        package="preprocess_cam",
        executable="preprocess_cam_node",
        name="preprocess_cam_node",
        output="screen",
    )

    # Node for line following logic
    # This node now publishes to /line_follower/raw_cmd_vel
    line_follower_node = Node(
        package="line_follower",
        executable="line_follower",
        name="line_follower_node",
        output="screen",
    )

    # Node for traffic light detection
    traffic_light_detector_node = Node(
        package='traffic_light_detector', # Nombre del paquete que creamos
        executable='detector_node',       # Nombre del ejecutable definido en setup.py
        name='traffic_light_detector_node', # Nombre del nodo en el sistema ROS
        output='screen',
        parameters=[ # The parameters can be adjusted if needed
            # {"hough_dp": 1.2},
            # {"hough_min_dist": 100},
            # {"hough_param1": 50},
            # {"hough_param2": 10},
            # {"hough_min_radius": 10},
            # {"hough_max_radius": 70},
        ]
    )

    # Node for speed moderation based on traffic light signals
    speed_moderator_node = Node(
        package="puzzlebot_control_utils",
        executable="speed_moderator",
        name="speed_moderator_node",
        output="screen",
    )

    return LaunchDescription([
        preprocess_cam_node,
        line_follower_node,
        traffic_light_detector_node, # Asegúrate de añadir el nuevo nodo a la lista
        speed_moderator_node,
    ])