#!/usr/bin/env python3
import math
import rclpy                                  # ROS 2 Python client library.
from rclpy.node import Node                  # Base class to create ROS 2 nodes.
from rclpy.duration import Duration          # Para duraciones en ROS 2
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from extended_pose_interface.msg import ExtendedPose

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("Controller node started.")

        # Estado interno
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.state = "IDLE"         # IDLE, MOVING
        self.current_target = None
        self.pose_queue = []        # Cola de mensajes ExtendedPose

        # Suscripciones
        self.create_subscription(
            ExtendedPose,
            'pose',
            self.pose_callback,
            10
        )
        self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        # Publicador de comandos de velocidad
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def pose_callback(self, msg: ExtendedPose):
        """Recibe un ExtendedPose y lo añade a la cola de objetivos."""
        self.pose_queue.append(msg)

    def odom_callback(self, msg: Odometry):
        """Procesa la odometría, actualiza estado y ejecuta control hacia el target."""
        # Actualiza pose actual a partir de odometría
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        self.get_logger().info(
            f"Odometry → X={self.current_x:.2f}, Y={self.current_y:.2f}, Yaw={self.current_yaw:.2f}"
        )

        # Si estamos libres y hay objetivos pendientes, arrancamos el siguiente
        if self.state == "IDLE" and self.pose_queue:
            self.current_target = self.pose_queue.pop(0)
            self.state = "MOVING"

        if self.state == "MOVING" and self.current_target is not None:
            # Extrae target
            tx = self.current_target.pose.position.x
            ty = self.current_target.pose.position.y
            q_t = self.current_target.pose.orientation
            # Cálculo robusto de yaw objetivo desde cuaternión
            siny_t = 2.0 * (q_t.w * q_t.z + q_t.x * q_t.y)
            cosy_t = 1.0 - 2.0 * (q_t.y * q_t.y + q_t.z * q_t.z)
            target_yaw = math.atan2(siny_t, cosy_t)

            v_cmd = self.current_target.linear_velocity
            w_cmd = self.current_target.angular_velocity

            # Banderas de finalización
            flag_rotated = False
            flag_translated = False

            # 1) Rotación hacia yaw deseado
            delta = self._normalize_angle(target_yaw - self.current_yaw)
            if abs(delta) > 1e-2 and w_cmd != 0.0:
                self._execute_motion(0.0, math.copysign(w_cmd, delta), 0.01)
            else:
                flag_rotated = True
                self.get_logger().info("Rotation skipped")

            # 2) Traslación hacia posición deseada
            dx = tx - self.current_x
            dy = ty - self.current_y
            dist = math.hypot(dx, dy)
            if dist > 1e-2 and v_cmd > 0.0:
                self._execute_motion(v_cmd, 0.0, 0.1)
            else:
                flag_translated = True
                self.get_logger().info("Translation skipped")

            # 3) Si ya rotamos y trasladamos, marcamos completado
            if flag_rotated and flag_translated:
                self.get_logger().info("Target reached → advancing to next")
                self.current_target = None
                self.state = "IDLE"

    def _execute_motion(self, lin_vel: float, ang_vel: float, duration: float):
        """
        Publica un Twist constante durante 'duration' segundos de forma no bloqueante
        para el bucle principal de callbacks.
        """
        twist = Twist()
        twist.linear.x = lin_vel
        twist.angular.z = ang_vel

        # Convierte duration (float) a Duration(sec, nanosec)
        sec = int(duration)
        nanosec = int((duration - sec) * 1e9)
        end_time = self.get_clock().now() + Duration(sec, nanosec)
        sleep_step = Duration(0, int(0.01 * 1e9))

        while self.get_clock().now() < end_time:
            self.cmd_pub.publish(twist)
            # Cede control a otros callbacks
            self.get_clock().sleep_for(sleep_step)

        # Para completamente
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Envuelve un ángulo a [-π, π]."""
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()