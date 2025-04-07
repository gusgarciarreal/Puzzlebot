#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
# Importar la librería de pygame para manejar el joystick
import pygame


class Interprete(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        # Creamos el publicador para el tópico 'cmd_vel' con tipo Twist
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # Publicar a 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Inicializar pygame y el joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info('Joystick encontrado: ' + self.joystick.get_name())
        else:
            self.get_logger().error('¡No se encontró ningún joystick!')
            self.joystick = None

    def timer_callback(self):
        if self.joystick is None:
            return

        # Procesar la cola de eventos de pygame
        pygame.event.pump()

        # Crear el mensaje Joy
        joy_msg = Joy()

        # Leer los ejes del joystick
        axes = []
        for i in range(self.joystick.get_numaxes()):
            axis_value = self.joystick.get_axis(i)
            axes.append(axis_value)
        joy_msg.axes = axes

        # Leer los botones del joystick
        buttons = []
        for i in range(self.joystick.get_numbuttons()):
            button_state = self.joystick.get_button(i)
            buttons.append(button_state)
        joy_msg.buttons = buttons

        # Info del joystick
        self.get_logger().info(
            'Mensaje publicado -> Ejes: {} Botones: {}'.format(joy_msg.axes, joy_msg.buttons))

        # Publicar el mensaje
        twist_msg = Twist()
        # Asignamos la velocidad lineal en x y la velocidad angular en z
        # Invertimos y normalizamos la velocidad lineal
        twist_msg.linear.x = joy_msg.axes[1]*-10.0
        # Normalizamos la velocidad angular
        twist_msg.angular.z = joy_msg.axes[2]*10
        # redondear a 1 decimal excepto si es menor a 0.1 (entonces 2 decimales)
        if abs(twist_msg.linear.x) < 0.1:
            twist_msg.linear.x = round(twist_msg.linear.x, 2)
        else:
            twist_msg.linear.x = round(twist_msg.linear.x, 1)
        if abs(twist_msg.angular.z) < 0.1:
            twist_msg.angular.z = round(twist_msg.angular.z, 2)
        else:
            twist_msg.angular.z = round(twist_msg.angular.z, 1)
        
        # Mantener los otros valores en 0 para mandar el mensaje
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0

        # Publicar el mensaje en el tópico 'cmd_vel'
        self.pub_cmd_vel.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Interprete()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
