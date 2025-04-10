#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodo path_generator: se asignan parámetros de ejemplo, cámbialos según necesites.
    path_generator_node = Node(
        package='seguidor_poligonos',  # Reemplaza con el nombre real de tu paquete
        executable='path_generator',  # Nombre del ejecutable del nodo path_generator
        name='path_generator',
        output='screen',
        parameters=[
            {"waypoints": "[(0,0), (1,0), (1,1), (0,1), (0,0)]"},  # Lista de waypoints
            {"mode": "time"},  # "speeds" o "time"
            {"linear_velocity": 0.2},
            {"angular_velocity": 0.5235987755982988},  # 30 grados en radianes
            {"total_time": 16.4}  # Se utiliza únicamente en modo "time"
        ]
    )

    # Nodo controller
    controller_node = Node(
        package='seguidor_poligonos',  # Reemplaza con el nombre real de tu paquete
        executable='controller',      # Nombre del ejecutable del nodo controller
        name='controller',
        output='screen'
    )

    return LaunchDescription([
        path_generator_node,
        controller_node
    ])
