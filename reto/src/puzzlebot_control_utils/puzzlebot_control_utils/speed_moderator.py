#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class SpeedModerator(Node):
    def __init__(self):
        super().__init__('speed_moderator')
        
        # Store the last received command from the line follower
        self.last_raw_cmd_vel = Twist()
        # Store the current traffic light flag (default to 3 - stop)
        self.traffic_light_flag = 3 

        # Subscriber to the raw commands from the line follower
        self.raw_cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/line_follower/raw_cmd_vel', # Subscribes to the modified line_follower output
            self.raw_cmd_vel_callback,
            10)

        # Subscriber to the traffic light status
        self.traffic_light_subscriber = self.create_subscription(
            Int32,
            '/traffic_light_status',
            self.traffic_light_callback,
            10)

        # Publisher for the moderated commands to the robot
        self.moderated_cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel', # Final command topic for the robot
            10)

        # Timer to process and publish commands periodically
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.process_and_publish_cmd)

        self.get_logger().info('Speed Moderator node has been started.')

    def raw_cmd_vel_callback(self, msg):
        """Callback for receiving raw velocity commands."""
        self.last_raw_cmd_vel = msg
        # self.get_logger().info(f'Received raw_cmd_vel: Linear.x: {msg.linear.x}, Angular.z: {msg.angular.z}')


    def traffic_light_callback(self, msg):
        """Callback for receiving traffic light status."""
        self.traffic_light_flag = msg.data
        self.get_logger().info(f'Received traffic_light_flag: {self.traffic_light_flag}')

    def process_and_publish_cmd(self):
        """Processes the last raw command based on the traffic light flag and publishes it."""
        moderated_cmd = Twist()
        
        if self.traffic_light_flag == 1: # Green light: Pass through
            moderated_cmd = self.last_raw_cmd_vel
            # self.get_logger().info('Flag 1: Publishing original speed.')
        elif self.traffic_light_flag == 2: # Yellow light: Reduce speed by half
            moderated_cmd.linear.x = self.last_raw_cmd_vel.linear.x / 2.0
            moderated_cmd.linear.y = self.last_raw_cmd_vel.linear.y / 2.0
            moderated_cmd.linear.z = self.last_raw_cmd_vel.linear.z / 2.0
            moderated_cmd.angular.x = self.last_raw_cmd_vel.angular.x / 2.0
            moderated_cmd.angular.y = self.last_raw_cmd_vel.angular.y / 2.0
            moderated_cmd.angular.z = self.last_raw_cmd_vel.angular.z / 2.0
            # self.get_logger().info('Flag 2: Publishing halved speed.')
        elif self.traffic_light_flag == 3: # Red light: Stop
            moderated_cmd.linear.x = 0.0
            moderated_cmd.linear.y = 0.0
            moderated_cmd.linear.z = 0.0
            moderated_cmd.angular.x = 0.0
            moderated_cmd.angular.y = 0.0
            moderated_cmd.angular.z = 0.0
            # self.get_logger().info('Flag 3: Publishing zero speed.')
        else: # Unknown flag, default to stop for safety
            moderated_cmd.linear.x = 0.0
            moderated_cmd.linear.y = 0.0
            moderated_cmd.linear.z = 0.0
            moderated_cmd.angular.x = 0.0
            moderated_cmd.angular.y = 0.0
            moderated_cmd.angular.z = 0.0
            self.get_logger().warn(f'Unknown traffic_light_flag: {self.traffic_light_flag}. Stopping robot.')

        self.moderated_cmd_vel_publisher.publish(moderated_cmd)

def main(args=None):
    rclpy.init(args=args)
    speed_moderator_node = SpeedModerator()
    try:
        rclpy.spin(speed_moderator_node)
    except KeyboardInterrupt:
        pass
    finally:
        speed_moderator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()