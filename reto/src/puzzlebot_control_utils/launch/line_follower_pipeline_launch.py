#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file to start the complete line follower pipeline,
    including camera preprocessing, line detection, and speed moderation.
    """

    # Node for camera preprocessing
    preprocess_cam_node = Node(
        package="preprocess_cam",  # As per your file structure
        executable="preprocess_cam_node",  # As defined in its setup.py
        name="preprocess_cam_node",
        output="screen",
    )

    # Node for line following logic
    # This node now publishes to /line_follower/raw_cmd_vel
    line_follower_node = Node(
        package="line_follower",  # As per your file structure
        executable="line_follower",  # As defined in its setup.py
        name="line_follower_node",  # Renamed for clarity in the launch file
        output="screen",
    )

    # Node for speed moderation based on traffic light signals
    speed_moderator_node = Node(
        package="puzzlebot_control_utils",  # The new package for the moderator
        executable="speed_moderator",  # As defined in its setup.py
        name="speed_moderator_node",  # Renamed for clarity
        output="screen",
    )

    return LaunchDescription(
        [preprocess_cam_node, line_follower_node, speed_moderator_node]
    )
