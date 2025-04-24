import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class DeadReckoning(Node):
    """ROS2 Node for computing 2D odometry via dead-reckoning from wheel encoder data."""

    def __init__(self):
        super().__init__('dead_reckoning')

        # — Robot physical parameters —
        self.wheel_radius = 0.05   # [m] wheel radius
        self.wheel_base   = 0.19   # [m] distance between wheels
        self.sample_time  = 0.01   # [s] minimum integration interval
        self.rate_hz      = 200.0  # [Hz] timer frequency

        # — State variables —
        self.x     = 0.0  # [m]  robot x-position
        self.y     = 0.0  # [m]  robot y-position
        self.theta = 0.0  # [rad] robot heading (yaw)

        # — Wheel angular velocities (rad/s) —
        self.wr = 0.0  # right wheel
        self.wl = 0.0  # left wheel

        # Initialize last update time
        self.last_time = self.get_clock().now()

        # — Subscriptions to encoder topics —
        self.create_subscription(
            Float32,
            'VelocityEncR',
            self._enc_r_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            'VelocityEncL',
            self._enc_l_callback,
            qos_profile_sensor_data
        )

        # — Publisher for computed odometry —
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            qos_profile_sensor_data
        )

        # — Timer to trigger periodic state updates —
        self.create_timer(1.0 / self.rate_hz, self._update)

        # Graceful shutdown on Ctrl+C
        signal.signal(signal.SIGINT, self._stop_handler)

        self.get_logger().info("DeadReckoning node started.")

    def _enc_r_callback(self, msg: Float32):
        """Store latest right-wheel angular velocity."""
        self.wr = msg.data

    def _enc_l_callback(self, msg: Float32):
        """Store latest left-wheel angular velocity."""
        self.wl = msg.data

    def _update(self):
        """Integrate pose if enough time has passed since last update."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt < self.sample_time:
            return  # skip if too soon

        # Compute each wheel’s linear speed
        vr = self.wheel_radius * self.wr
        vl = self.wheel_radius * self.wl

        # Robot’s forward and angular velocities
        v     = 0.5 * (vr + vl)
        omega = (vr - vl) / self.wheel_base

        # Integrate into pose
        self.theta += omega * dt
        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt

        self.last_time = now
        self._publish_odometry(v, omega)

    def _publish_odometry(self, linear_vel: float, angular_vel: float):
        """Fill and publish an Odometry message with current pose and velocities."""
        # Quaternion from yaw only (roll = pitch = 0)
        qz = math.sin(self.theta * 0.5)
        qw = math.cos(self.theta * 0.5)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id    = 'odom'
        odom.child_frame_id     = 'base_footprint'

        # Pose
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Twist
        odom.twist.twist.linear.x   = linear_vel
        odom.twist.twist.angular.z  = angular_vel

        self.odom_pub.publish(odom)
        # Use debug level to avoid flooding logs at high rate
        self.get_logger().debug(
            f"Pose → x: {self.x:.3f}, y: {self.y:.3f}, θ: {self.theta:.3f} | "
            f"v: {linear_vel:.3f}, ω: {angular_vel:.3f}"
        )

    def _stop_handler(self, signum, frame):
        """Cleanly shut down on SIGINT."""
        self.get_logger().info("Shutdown signal received, exiting.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()