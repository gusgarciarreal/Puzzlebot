#!/usr/bin/env python3
import rclpy                                  # ROS 2 Python client library.
from rclpy.node import Node                   # Base class to create ROS 2 nodes.
from rclpy import qos
import math                                   # Library for mathematical functions.
import time                                   # Module to handle delays and timestamps.
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import transforms3d

# Import the ROS message type for velocity commands.
from geometry_msgs.msg import Twist
# Import the custom message type ExtendedPose from the extended_pose_interface package.
from extended_pose_interface.msg import ExtendedPose

class Controller(Node):
    def __init__(self):
        # Initialize the node with the name 'controller'
        super().__init__('controller')

        # Subscriptions
        # Create a subscription to the 'pose' topic where ExtendedPose messages are published.
        # This is the same topic on which the path_generator node publishes segments.
        self.subscription = self.create_subscription(
            ExtendedPose,          # Message type expected on the topic.
            'pose',                # The topic name.
            self.pose_callback,    # Callback function to process received messages.
            10                     # Queue size.
        )
        self.sub_odom = self.create_subscription(Odometry,'odom',self.odom_callback, qos.qos_profile_sensor_data)
        
        # Create a publisher for the 'cmd_vel' topic to publish velocity commands to the robot.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a publisher for the 'anglel' topic to debug
        self.angle_pub = self.create_publisher(Float32, 'angle', 10)

        self.get_logger().info("Controller node started.")
        
        # Initialize the simulated robot state (current position and orientation).
        self.current_x = 0.0     # Current x-coordinate position of the robot.
        self.current_y = 0.0     # Current y-coordinate position of the robot.
        self.current_yaw = 0.0   # Current orientation (yaw angle in radians).

        # Initialize the range of error and the begin of state machine
        self.tolerance = 0.05  # Umbral de tolerancia en radianes
        self.state = "IDLE"  # It could be "IDLE", "ROTATING", "MOVING"
        self.current_target = None

        #Queue to manage data from ExtendedPose
        self.pose_queue = []  

    def odom_callback(self, msg):

        q = msg.pose.pose.orientation
        quaternion = [q.w, q.x, q.y, q.z]
        euler = transforms3d.euler.quat2euler(quaternion)

        self.current_yaw = euler[2]
        self.current_x  = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        self.get_logger().info(f'Received odometry: X={self.current_x:.2f}, Y={self.current_y:.2f}, Yaw={ self.current_yaw:.2f}')

        if self.state == "IDLE" and self.pose_queue:
            self.current_target = self.pose_queue[0]
            self.state = "ROTATING"
        
        if self.state == "ROTATING":
            target_yaw = 2 * math.atan2(self.current_target.pose.orientation.z, self.current_target.pose.orientation.w) # Compute the target yaw from the quaternion.
            delta_angle = target_yaw - self.current_yaw
            delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))

            if abs(delta_angle) > self.tolerance:
                twist = Twist()
                w = self.current_target.angular_velocity if delta_angle > 0 else -self.current_target.angular_velocity
                twist.angular.z = w
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info(f"Giro completado")
                self.stop_robot()
                self.state = "MOVING"
        
        if self.state == "MOVING":
            target_x = self.current_target.pose.position.x
            target_y = self.current_target.pose.position.y
            distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

            if distance > self.tolerance:
                twist = Twist()
                twist.linear.x = self.current_target.linear_velocity  # Set the forward speed.
                twist.angular.z = 0.0                  # No rotation during linear motion.
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info(f"Avance completado a X = {target_x:.2f} y Y = {target_y:.2f}")
                self.stop_robot()
                self.pose_queue.pop(0)
                self.current_target = None
                self.state = "IDLE"


    def pose_callback(self, msg):
        self.pose_queue.append(msg)
        self.get_logger().info("Nuevo waypoint recibido y agregado a la cola.")


    def stop_robot(self):
        """
        Sends a zero velocity command to stop all robot movements.
        
        This function publishes a Twist message with zero linear and angular velocities.
        It then sleeps for a second to ensure the robot has come to a full stop.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.1)

def main(args=None):
    # Initialize the ROS 2 Python client library.
    rclpy.init(args=args)
    # Create an instance of the Controller node.
    node = Controller()
    # Keep the node running to listen for messages and handle callbacks.
    rclpy.spin(node)
    # Cleanup on exit.
    node.destroy_node()
    rclpy.shutdown()

# Execute the main function when the script is run directly.
if __name__ == '__main__':
    main()
