#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from extended_pose_interface.msg import ExtendedPose

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        # Declarar parámetros:
        # - waypoints: cadena que representa una lista de tuplas; por ejemplo: "[(0,0),(1,0),(0.5,1)]"
        # - mode: "speeds" o "time"
        #    • En "speeds" se utilizan los parámetros: linear_velocity y angular_velocity
        #    • En "time" se utiliza el parámetro total_time (tiempo total para completar la ruta completa)
        self.declare_parameter("waypoints", "[(0,0),(1,0),(0.5,1)]")
        self.declare_parameter("mode", "speeds")  # "speeds" o "time"
        self.declare_parameter("linear_velocity", 0.2)  # en m/s (modo speeds)
        self.declare_parameter("angular_velocity", math.radians(30))  # en rad/s (modo speeds)
        self.declare_parameter("total_time", 0.0)  # en segundos para la ruta completa (modo time)

        # Parámetros de configuración
        waypoints_str = self.get_parameter("waypoints").value
        try:
            self.waypoints = eval(waypoints_str)
            if not isinstance(self.waypoints, list) or len(self.waypoints) < 2:
                raise ValueError("Debe proporcionar al menos dos puntos")
        except Exception as e:
            self.get_logger().error(f"Error al interpretar los waypoints: {e}")
            self.waypoints = []
        
        self.mode = self.get_parameter("mode").value.lower()
        self.user_linear_velocity = self.get_parameter("linear_velocity").value
        self.user_angular_velocity = self.get_parameter("angular_velocity").value
        self.total_time = self.get_parameter("total_time").value  # para modo time

        # Límites máximos
        self.max_linear = 0.3         # m/s
        self.max_angular = math.radians(90)  # 90°/s en rad/s

        # Publisher para /pose con mensaje ExtendedPose
        self.pose_pub = self.create_publisher(ExtendedPose, 'pose', 10)
        
        # Variables internas
        self.current_segment = 0
        self.segments = []  # se llenará con diccionarios con info de cada segmento
        self.initial_yaw = 0.0  # Se asume orientación inicial 0 rad
        
        # Preparar la lista de segmentos calculando:
        #   • distancia entre puntos y ángulo deseado (diferencia de orientación)
        self.prepare_segments()
        if not self.segments:
            self.get_logger().error("No se pudo calcular la ruta (lista de segmentos vacía).")
            return

        # Si modo "speeds": validar que los parámetros no excedan los límites
        if self.mode == "speeds":
            if self.user_linear_velocity > self.max_linear or abs(self.user_angular_velocity) > self.max_angular:
                self.get_logger().warn("Las velocidades proporcionadas exceden los límites permitidos. Ruta ignorada.")
                return
        elif self.mode == "time":
            # Calcular el tiempo mínimo total requerido usando velocidades máximas para cada segmento
            self.T_min_total = 0.0
            for seg in self.segments:
                t_rot_min = abs(seg["delta_angle"]) / self.max_angular
                t_lin_min = seg["distance"] / self.max_linear
                seg["t_rot_min"] = t_rot_min
                seg["t_lin_min"] = t_lin_min
                seg["t_min"] = t_rot_min + t_lin_min
                self.T_min_total += seg["t_min"]
            if self.total_time <= self.T_min_total:
                self.get_logger().warn(
                    f"El tiempo total proporcionado ({self.total_time:.2f} s) es menor o igual al mínimo requerido ({self.T_min_total:.2f} s). Ruta ignorada."
                )
                return
        else:
            self.get_logger().error("Modo de operación incorrecto. Use 'speeds' o 'time'.")
            return

        # Publicar cada segmento con un timer (se publican de a uno cada 1 s)
        self.timer = self.create_timer(1.0, self.publish_next_segment)
        self.get_logger().info("Nodo path_generator iniciado.")

    def prepare_segments(self):
        """
        Prepara la lista de segmentos a partir de la lista de waypoints.
        Cada segmento es un diccionario que contiene:
          - start: (x,y)
          - end: (x,y)
          - distance: distancia entre start y end
          - desired_yaw: ángulo (en rad) entre start y end
          - delta_angle: diferencia entre el ángulo deseado y la orientación inicial del segmento
        """
        previous_yaw = self.initial_yaw
        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i+1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            distance = math.sqrt(dx**2 + dy**2)
            desired_yaw = math.atan2(dy, dx)
            # Diferencia de orientación (se normaliza entre -pi y pi)
            delta_angle = desired_yaw - previous_yaw
            delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))
            seg = {
                "start": start,
                "end": end,
                "distance": distance,
                "desired_yaw": desired_yaw,
                "delta_angle": delta_angle
            }
            self.segments.append(seg)
            previous_yaw = desired_yaw

    def publish_next_segment(self):
        if self.current_segment >= len(self.segments):
            self.get_logger().info("Se han publicado todos los segmentos. Ruta completada.")
            self.destroy_timer(self.timer)
            return

        seg = self.segments[self.current_segment]
        end = seg["end"]
        desired_yaw = seg["desired_yaw"]

        msg = ExtendedPose()
        # Llenar la pose (se usa solo position.x, position.y y la orientación en z, w para representar yaw)
        msg.pose.position.x = float(end[0])
        msg.pose.position.y = float(end[1])
        # Conversión de yaw a cuaternión (asumiendo solo giro en Z)
        msg.pose.orientation.z = math.sin(desired_yaw / 2.0)
        msg.pose.orientation.w = math.cos(desired_yaw / 2.0)
        
        # Modo de operación
        if self.mode == "speeds":
            # Se usan velocidades constantes proporcionadas por el usuario.
            t_rot = abs(seg["delta_angle"]) / self.user_angular_velocity if self.user_angular_velocity != 0 else 0.0
            t_lin = seg["distance"] / self.user_linear_velocity if self.user_linear_velocity != 0 else 0.0
            total_seg_time = t_rot + t_lin
            msg.linear_velocity = float(self.user_linear_velocity)
            # Se respeta el signo de delta_angle para la rotación
            msg.angular_velocity = self.user_angular_velocity if seg["delta_angle"] >= 0 else -self.user_angular_velocity
            msg.time_stamp = float(total_seg_time)
        elif self.mode == "time":
            # Para distribuir el tiempo total de la ruta:
            # Se calcula el tiempo extra disponible para este segmento.
            seg_min = seg["t_min"]  # tiempo mínimo para este segmento al usar velocidades máximas
            extra_time_seg = (seg_min / self.T_min_total) * (self.total_time - self.T_min_total)
            # Distribuir el extra time de forma proporcional entre la parte de rotación y la de traslación
            if seg["t_min"] > 0:
                t_rot = seg["t_rot_min"] + extra_time_seg * (seg["t_rot_min"] / seg["t_min"])
                t_lin = seg["t_lin_min"] + extra_time_seg * (seg["t_lin_min"] / seg["t_min"])
            else:
                t_rot, t_lin = 0.0, 0.0
            total_seg_time = t_rot + t_lin
            # Velocidades requeridas:
            v = seg["distance"] / t_lin if t_lin > 0 else 0.0
            # La velocidad angular se estima usando el valor absoluto y se asigna signo
            w = abs(seg["delta_angle"]) / t_rot if t_rot > 0 else 0.0
            w = w if seg["delta_angle"] >= 0 else -w

            # Verificar límites
            if v > self.max_linear or abs(w) > self.max_angular:
                self.get_logger().warn("Las velocidades calculadas para este segmento exceden los límites permitidos. Ruta ignorada.")
                self.destroy_timer(self.timer)
                return

            msg.linear_velocity = float(v)
            msg.angular_velocity = float(w)
            msg.time_stamp = float(total_seg_time)
        else:
            self.get_logger().error("Modo desconocido en path_generator.")
            return

        self.get_logger().info(
            f"Segmento {self.current_segment+1}/{len(self.segments)}: "
            f"Posición final=({end[0]:.2f}, {end[1]:.2f}), "
            f"yaw={desired_yaw:.2f} rad, "
            f"v={msg.linear_velocity:.2f} m/s, w={msg.angular_velocity:.2f} rad/s, "
            f"tiempo={msg.time_stamp:.2f} s"
        )
        self.pose_pub.publish(msg)
        self.current_segment += 1

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""Blanco y negro
Rectangulo con margen del 10% de la resolución de mi pantalla
Donde doy click un círculo"""