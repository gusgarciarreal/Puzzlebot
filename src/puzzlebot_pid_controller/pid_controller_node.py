import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Declaración de parámetros desde params.yaml
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.0)
        self.declare_parameter('kp_angular', 4.0)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.1)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)

        # Lectura de parámetros
        self.kp_linear = self.get_parameter('kp_linear').value
        self.ki_linear = self.get_parameter('ki_linear').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.ki_angular = self.get_parameter('ki_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        # Variables PID
        self.integral_linear = 0.0
        self.prev_error_linear = 0.0
        self.integral_angular = 0.0
        self.prev_error_angular = 0.0

        # Waypoints (camino cuadrado)
        self.waypoints = [
            (2.0, 0.0, 0.0),
            (2.0, 2.0, math.pi/2),
            (0.0, 2.0, math.pi),
            (0.0, 0.0, -math.pi/2)
        ]
        self.current_goal_idx = 0

        # Estado del robot
        self.current_pose = None
        self.state = 'rotate'  # 'rotate' o 'move_forward'

        # Subs y pubs
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('PID Controller Node has started!')

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        # Quaternion to yaw
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (position.x, position.y, yaw)

    def control_loop(self):
        if self.current_pose is None:
            return

        goal_x, goal_y, goal_theta = self.waypoints[self.current_goal_idx]
        x, y, theta = self.current_pose

        dx = goal_x - x
        dy = goal_y - y
        distance_error = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - theta)

        twist = Twist()

        if self.state == 'rotate':
            twist.angular.z = self.compute_pid_angular(angle_error)
            if abs(angle_error) < self.angle_tolerance:
                self.state = 'move_forward'
                self.reset_pid_angular()
        elif self.state == 'move_forward':
            twist.linear.x = self.compute_pid_linear(distance_error)
            twist.angular.z = self.compute_pid_angular(angle_error)
            if distance_error < self.position_tolerance:
                self.current_goal_idx = (self.current_goal_idx + 1) % len(self.waypoints)
                self.state = 'rotate'
                self.reset_pid_linear()

        self.cmd_vel_pub.publish(twist)

    def compute_pid_linear(self, error):
        dt = 0.05
        self.integral_linear += error * dt
        derivative = (error - self.prev_error_linear) / dt
        output = (self.kp_linear * error +
                  self.ki_linear * self.integral_linear +
                  self.kd_linear * derivative)
        self.prev_error_linear = error
        return self.saturate(output, 0.3)

    def compute_pid_angular(self, error):
        dt = 0.05
        self.integral_angular += error * dt
        derivative = (error - self.prev_error_angular) / dt
        output = (self.kp_angular * error +
                  self.ki_angular * self.integral_angular +
                  self.kd_angular * derivative)
        self.prev_error_angular = error
        return self.saturate(output, 1.0)

    def reset_pid_linear(self):
        self.integral_linear = 0.0
        self.prev_error_linear = 0.0

    def reset_pid_angular(self):
        self.integral_angular = 0.0
        self.prev_error_angular = 0.0

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def saturate(self, value, limit):
        return max(min(value, limit), -limit)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
