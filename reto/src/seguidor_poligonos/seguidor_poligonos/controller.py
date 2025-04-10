#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time

from geometry_msgs.msg import Twist
from extended_pose_interface.msg import ExtendedPose

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        # Suscribirse al tópico /pose (ExtendedPose)
        self.subscription = self.create_subscription(ExtendedPose, 'pose', self.pose_callback, 10)
        # Publicador para enviar comandos de velocidad al tópico /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Nodo controller iniciado.")

        # Estado simulado del robot (posición y orientación actual)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0  # en radianes

    def pose_callback(self, msg):
        # Extraer la pose objetivo del mensaje
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        # Se extrae el yaw a partir del cuaternión (asumiendo giro solo en Z)
        target_yaw = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)

        self.get_logger().info(
            f"Recibido segmento: objetivo=({target_x:.2f}, {target_y:.2f}), yaw={target_yaw:.2f} rad, "
            f"v={msg.linear_velocity:.2f} m/s, w={msg.angular_velocity:.2f} rad/s, "
            f"tiempo segmentado={msg.time_stamp:.2f} s"
        )

        # 1. Realizar la rotación requerida (se asume que se rota primero)
        delta_angle = target_yaw - self.current_yaw
        # Normalizar entre -pi y pi
        delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))
        if abs(delta_angle) > 0.01:
            # Se utiliza la velocidad angular del mensaje
            w = msg.angular_velocity if delta_angle >= 0 else -msg.angular_velocity
            t_rot = abs(delta_angle) / abs(msg.angular_velocity)  if abs(msg.angular_velocity) > 0 else 0.0
            self.get_logger().info(f"Rotando {delta_angle:.2f} rad durante {t_rot:.2f} s")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = w
            start_time = time.time()
            while (time.time() - start_time) < t_rot:
                self.cmd_pub.publish(twist)
                time.sleep(0.1)
            self.stop_robot()
            # Actualizar orientación actual
            self.current_yaw = target_yaw
        else:
            self.get_logger().info("No se requiere rotación.")

        # 2. Realizar el movimiento recto hacia la posición objetivo
        distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        if distance > 0.01:
            t_lin = distance / msg.linear_velocity if msg.linear_velocity > 0 else 0.0
            self.get_logger().info(f"Moviendo en línea recta {distance:.2f} m durante {t_lin:.2f} s")
            twist = Twist()
            twist.linear.x = msg.linear_velocity
            twist.angular.z = 0.0
            start_time = time.time()
            while (time.time() - start_time) < t_lin:
                self.cmd_pub.publish(twist)
                time.sleep(0.1)
            self.stop_robot()
            # Actualizar posición actual
            self.current_x = target_x
            self.current_y = target_y
        else:
            self.get_logger().info("No se requiere movimiento lineal.")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
