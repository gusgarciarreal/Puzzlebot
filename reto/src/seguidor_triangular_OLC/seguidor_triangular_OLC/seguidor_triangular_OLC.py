#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class SeguidorTriangular(Node):
    def __init__(self):
        super().__init__('seguidor_triangular')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.orientacion_actual = 0.0

        # Definición de los vértices del triángulo
        self.A = (0.0, 0.0)
        self.B = (1.0, 0.0)
        self.C = (0.5, 1.0)
        self.segmentos = [(self.A, self.B), (self.B, self.C), (self.C, self.A)]

        self.velocidad_lineal = 0.2  # m/s
        self.velocidad_angular = math.radians(30)  # rad/s
        self.idx_segmento = 0
        self.moviendo = False
        self.iniciar_trayectoria()

    def iniciar_trayectoria(self):
        # Ejecuta la trayectoria completa de forma secuencial.
        for (inicio, fin) in self.segmentos:
            dx = fin[0] - inicio[0]
            dy = fin[1] - inicio[1]
            angulo_deseado = math.atan2(dy, dx)
            # Calcular la diferencia de orientación
            giro = angulo_deseado - self.orientacion_actual
            # Normalizar el ángulo entre -pi y pi
            giro = math.atan2(math.sin(giro), math.cos(giro))
            self.get_logger().info(f"Rotando {giro:.2f} radianes")
            self.rotar(giro)
            self.orientacion_actual = angulo_deseado

            distancia = math.sqrt(dx**2 + dy**2)
            self.get_logger().info(
                f"Moviendo en línea recta {distancia:.2f} metros")
            self.mover_recto(distancia)

        self.get_logger().info("Recorrido triangular completado.")
        rclpy.shutdown()

    def mover_recto(self, distancia):
        # Calcula el tiempo que se debe mover
        t_total = abs(distancia / self.velocidad_lineal)
        twist = Twist()
        twist.linear.x = self.velocidad_lineal
        twist.angular.z = 0.0
        inicio = time.time()
        while (time.time() - inicio) < t_total:
            self.publisher_.publish(twist)
            time.sleep(self.timer_period)
        self.detener()

    def rotar(self, angulo):
        # Calcula el tiempo que se debe rotar
        t_total = abs(angulo / self.velocidad_angular)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.velocidad_angular if angulo >= 0 else -self.velocidad_angular
        inicio = time.time()
        while (time.time() - inicio) < t_total:
            self.publisher_.publish(twist)
            time.sleep(self.timer_period)
        self.detener()

    def detener(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)

    def timer_callback(self):
        # Este callback puede quedar vacío o utilizarse para otras tareas periódicas
        pass


def main(args=None):
    rclpy.init(args=args)
    seguidor = SeguidorTriangular()
    rclpy.spin(seguidor)


if __name__ == '__main__':
    main()
