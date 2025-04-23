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

    def odom_callback(self, msg):
        self.current_x  = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quaternion = [q.w, q.x, q.y, q.z]
        euler = transforms3d.euler.quat2euler(quaternion)

        self.current_yaw = euler[2]

        self.get_logger().info(f'Received odometry: X={self.current_x:.2f}, Y={self.current_y:.2f}, Yaw={ self.current_yaw:.2f}')

    def pose_callback(self, msg):
        """
        Callback function triggered upon receiving an ExtendedPose message.
        
        It extracts the target position (x, y) and orientation (yaw) from the message.
        The controller then performs two actions:
          1. Rotates the robot in place to match the desired yaw.
          2. Moves the robot in a straight line to the target position.
        """
        # Extract target position from the message.
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        
        # Compute the target yaw from the quaternion.
        # Since only rotation around the Z-axis is assumed, only z and w components are used.
        target_yaw = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        
        # Log the received segment with target pose and velocity commands.
        self.get_logger().info(
            f"Received segment: target=({target_x:.2f}, {target_y:.2f}), yaw={target_yaw:.2f} rad, "
            f"v={msg.linear_velocity:.2f} m/s, w={msg.angular_velocity:.2f} rad/s, "
            f"segment time={msg.time_stamp:.2f} s"
        )
        
        # --------- 1. Rotation Phase -----------
        # First, calculate the required rotation (delta_angle) to align the current orientation with target_yaw.
        delta_angle = target_yaw - self.current_yaw
        # Normalize the angle to be within the range of -pi to pi.
        delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))

        tolerance = 0.05  # Umbral de tolerancia en radianes
        
        # Check if rotation is needed by comparing against a small threshold.
        if abs(delta_angle) > tolerance:
            # Determine the direction of rotation using the angular velocity provided in the message.
            w = msg.angular_velocity if delta_angle >= 0 else -msg.angular_velocity
            self.angle_pub.publish(w)
            
            # Create a Twist message to command the rotation.
            twist = Twist()
            twist.linear.x = 0.0         # No linear movement during rotation.
            twist.angular.z = w          # Angular speed as computed.
            
            while abs(delta_angle) > tolerance:
                self.cmd_pub.publish(twist)
                delta_angle = target_yaw - self.current_yaw # Actualizamos delta_angle en cada iteración
                delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))
                time.sleep(0.05) # Short sleep to allow repeated commands over time.

            self.get_logger().info(f"Rotación completada a {target_yaw:.2f} rad.")
            # After rotation, stop the robot.
            self.stop_robot()

        else:
            self.get_logger().info("Rotation not required.")
        
        # --------- 2. Translation Phase -----------
        # Compute the Euclidean distance to the target position.
        distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        
        # Check if movement is needed based on distance.
        if distance > tolerance:
            
            # Create a Twist message to command the linear movement.
            twist = Twist()
            twist.linear.x = msg.linear_velocity  # Set the forward speed.
            twist.angular.z = 0.0                  # No rotation during linear motion.

            while distance > tolerance:
                self.cmd_pub.publish(twist)
                distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
                time.sleep(0.05)          # Short sleep to allow repeated commands.

            self.get_logger().info(f"Avance completado a X = {target_x:.2f} y Y = {target_y:.2f}")
            # After moving, stop the robot.
            self.stop_robot()

        else:
            self.get_logger().info("Linear movement not required.")

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
