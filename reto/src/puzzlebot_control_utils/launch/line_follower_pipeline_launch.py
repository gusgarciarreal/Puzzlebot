#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os # Import os to handle file paths
from ament_index_python.packages import get_package_share_directory # To find the package directory

def generate_launch_description():
    """
    Launch file to start the complete line follower pipeline,
    including camera preprocessing, line detection, traffic light detection,
    signal detection, and speed moderation.
    """

    # Define the path to the new YAML configuration file for line_follower
    line_follower_config = os.path.join(
        get_package_share_directory('line_follower'), # Or your config package if different
        'config', # Assuming you will place your yaml in a 'config' subdirectory
        'line_follower_params.yaml'
    )

    # Node for camera preprocessing
    preprocess_cam_node = Node(
        package="preprocess_cam",
        executable="preprocess_cam_node",
        name="preprocess_cam_node",
        output="screen",
    )

    # Node for line following logic, now with parameters from YAML
    line_follower_node = Node(
        package="line_follower",
        executable="line_follower",
        name="line_follower_node",
        output="screen",
        parameters=[line_follower_config] # Load parameters from the YAML file
    )

    # Node for traffic light detection
    traffic_light_detector_node = Node(
        package='traffic_light_detector',
        executable='detector_node',
        name='traffic_light_detector_node',
        output='screen',
        parameters=[
            # {"hough_dp": 1.2}, # Example parameters, uncomment if needed
            # {"hough_min_dist": 100},
            # {"hough_param1": 50},
            # {"hough_param2": 10},
            # {"hough_min_radius": 10},
            # {"hough_max_radius": 70},
        ]
    )

    # Node for YOLOv8 signals detection
    signals_detection_node = Node(
        package="signals_detection",
        executable="yolo_detection_node", # As defined in its setup.py
        name="signals_detection_node",
        output="screen",
        # You can add YOLOv8 specific parameters here if needed,
        # for example, path to model, confidence threshold, etc.
        # parameters=[
        #     {"model_path": "/path/to/your/model.pt"},
        #     {"confidence_threshold": 0.5}
        # ]
    )

    # Node for speed moderation based on traffic light and signals
    speed_moderator_node = Node(
        package="puzzlebot_control_utils",
        executable="speed_moderator", # As defined in its setup.py
        name="speed_moderator_node",
        output="screen",
    )

    return LaunchDescription([
        preprocess_cam_node,
        line_follower_node,
        traffic_light_detector_node,
        signals_detection_node, # Added the new signals detection node
        speed_moderator_node,
    ])