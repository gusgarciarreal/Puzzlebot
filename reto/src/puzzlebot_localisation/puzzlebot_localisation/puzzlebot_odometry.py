import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import numpy as np

class DeadReckoning(Node):
    def __init__(self):
        super().__init__('dead_reckoning')

        # Parámetros del robot
        self.r = 0.05  # radio de las ruedas [m]
        self.l = 0.19  # distancia entre ruedas [m]

        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v_r = 0.0
        self.v_l = 0.0
        self.V = 0.0
        self.Omega = 0.0

        # Subscripciones a velocidades de ruedas
        self.create_subscription(Float32, 'VelocityEncR', self.encR_callback, 10)
        self.create_subscription(Float32, 'VelocityEncL', self.encL_callback, 10)

        # Publicador de odometría
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Timer para actualizar la odometría (100Hz)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.update_odom)

        self.last_time = self.get_clock().now()

        self.get_logger().info("Dead reckoning node started")

    def encR_callback(self, msg):
        self.v_r = self.r * msg.data

    def encL_callback(self, msg):
        self.v_l = self.r * msg.data

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt == 0.0:
            return

        self.last_time = now

        # Cálculo de velocidades
        self.V = (self.v_r + self.v_l) / 2.0
        self.Omega = (self.v_r - self.v_l) / self.l

        # Integración por método de Euler
        self.x += self.V * math.cos(self.theta) * dt
        self.y += self.V * math.sin(self.theta) * dt
        self.theta += self.Omega * dt
        self.theta = self.wrap_to_pi(self.theta)

        # Publicación del mensaje Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # Cuaternión sin transforms3d
        odom_msg.pose.pose.orientation = Quaternion()
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom_msg.twist.twist.linear.x = self.V
        odom_msg.twist.twist.angular.z = self.Omega

        self.odom_pub.publish(odom_msg)

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
