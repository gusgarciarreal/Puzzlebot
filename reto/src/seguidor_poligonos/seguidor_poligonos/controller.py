import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from extended_pose_interface.msg import ExtendedPose


class Controller(Node):
    """
    A simple two-phase (rotate → translate) controller that
    consumes ExtendedPose segments and publishes cmd_vel accordingly.
    """

    def __init__(self):
        super().__init__('controller')
        self.get_logger().info("Controller node started.")

        # --- State ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # --- ROS interfaces ---
        self.create_subscription(
            ExtendedPose, 'pose', self._on_pose, 10
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def _on_pose(self, msg: ExtendedPose):
        """Process one segment: rotate in place, then move straight."""
        # Extract target pose & speeds
        tx, ty = msg.pose.position.x, msg.pose.position.y
        target_yaw = 2.0 * \
            math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        v_cmd = msg.linear_velocity
        w_cmd = msg.angular_velocity

        self.get_logger().info(
            f"Segment → target=({tx:.2f},{ty:.2f}), yaw={target_yaw:.2f}, "
            f"v={v_cmd:.2f}, w={w_cmd:.2f}, t={msg.time_stamp:.2f}"
        )

        # 1) Rotation
        delta = self._normalize_angle(target_yaw - self.current_yaw)
        if abs(delta) > 1e-2 and w_cmd != 0.0:
            self._execute_motion(0.0, math.copysign(
                w_cmd, delta), abs(delta) / abs(w_cmd))
            self.current_yaw = target_yaw
        else:
            self.get_logger().info("Rotation skipped")

        # 2) Translation
        dx, dy = tx - self.current_x, ty - self.current_y
        dist = math.hypot(dx, dy)
        if dist > 1e-2 and v_cmd > 0.0:
            self._execute_motion(v_cmd, 0.0, dist / v_cmd)
            self.current_x, self.current_y = tx, ty
        else:
            self.get_logger().info("Translation skipped")

    def _execute_motion(self, lin_vel: float, ang_vel: float, duration: float):
        """
        Publish constant Twist for `duration` seconds, non-blocking.
        Uses rclpy.Clock.sleep_for() to yield to other ROS events.
        """
        twist = Twist()
        twist.linear.x = lin_vel
        twist.angular.z = ang_vel

        end_time = self.get_clock().now() + Duration(seconds=duration)
        while self.get_clock().now() < end_time:
            self.cmd_pub.publish(twist)
            # yield control so callbacks can run
            self.get_clock().sleep_for(Duration(seconds=0.05))

        # ensure full stop
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wrap to [-π, π]."""
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
