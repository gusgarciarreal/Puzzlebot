# path_generator_optimized.py
import math
import ast

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from extended_pose_interface.msg import ExtendedPose


class PathGenerator(Node):
    """
    Publishes a sequence of ExtendedPose messages, one per waypoint segment,
    either at fixed speeds or to fill a given total time.
    """

    def __init__(self):
        super().__init__('path_generator')

        # --- Declare & read parameters ---
        self.declare_parameter("waypoints", "[(0,0),(1,0),(0.5,1)]")
        # "speeds" or "time"
        self.declare_parameter("mode", "speeds")
        self.declare_parameter("linear_velocity", 0.2)        # m/s
        self.declare_parameter("angular_velocity", math.radians(30))  # rad/s
        self.declare_parameter("total_time", 0.0)             # s

        self._read_parameters()
        self._validate_waypoints()
        self._compute_segments()

        # --- Publish /pose once per second ---
        self.timer = self.create_timer(1.0, self._publish_next_segment)
        self.current_segment = 0

        # --- Allow dynamic reconfiguration ---
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("PathGenerator node started.")

    def _read_parameters(self):
        """Fetches and normalizes parameters from ROS."""
        self.mode = self.get_parameter("mode").value.lower()
        self.max_lin = 0.3
        self.max_ang = math.radians(90)
        self.user_v = self.get_parameter("linear_velocity").value
        self.user_w = self.get_parameter("angular_velocity").value
        self.total_time = self.get_parameter("total_time").value

        raw = self.get_parameter("waypoints").value
        if isinstance(raw, str):
            try:
                raw = ast.literal_eval(raw)
            except Exception as e:
                self.get_logger().error(f"Bad waypoints string: {e}")
                raw = []
        self.waypoints = [(float(x), float(y)) for (x, y) in raw]

    def _validate_waypoints(self):
        """Ensure we have at least two (x,y) pairs."""
        if not isinstance(self.waypoints, list) or len(self.waypoints) < 2:
            self.get_logger().error("Need ≥2 waypoints")
            self.waypoints = []

    def _compute_segments(self):
        """
        From successive waypoints build:
          distance, desired_yaw, delta_angle
        """
        self.segments = []
        prev_yaw = 0.0
        for (x0, y0), (x1, y1) in zip(self.waypoints, self.waypoints[1:]):
            dx, dy = x1 - x0, y1 - y0
            dist = math.hypot(dx, dy)
            yaw = math.atan2(dy, dx)
            delta = math.atan2(math.sin(yaw - prev_yaw),
                               math.cos(yaw - prev_yaw))
            self.segments.append({
                "end": (x1, y1),
                "distance": dist,
                "desired_yaw": yaw,
                "delta_angle": delta
            })
            prev_yaw = yaw

        if self.mode == "time":
            # precompute minimum times
            self.T_min = sum(
                abs(s["delta_angle"]) / self.max_ang +
                s["distance"] / self.max_lin
                for s in self.segments
            )

    def parameter_callback(self, params):
        """Recompute the route upon any relevant parameter update."""
        updated = False
        for p in params:
            if p.name in ("waypoints", "mode", "linear_velocity", "angular_velocity", "total_time"):
                updated = True
        if updated:
            self._read_parameters()
            self._validate_waypoints()
            self._compute_segments()
            self.current_segment = 0
            self.get_logger().info("Parameters changed - segments recomputed.")

            # Reiniciar temporizador de publicación de segmentos
            try:
                self.timer.cancel()
            except Exception:
                pass
            self.timer = self.create_timer(1.0, self._publish_next_segment)
            self.get_logger().info("Timer restarted to replay path with updated parameters.")

        res = SetParametersResult()
        res.successful = True
        return res

    def _publish_next_segment(self):
        """Publish one ExtendedPose per timer tick until done."""
        if self.current_segment >= len(self.segments):
            self.get_logger().info("Done publishing all segments.")
            self.timer.cancel()
            return

        seg = self.segments[self.current_segment]
        x, y = seg["end"]
        yaw = seg["desired_yaw"]
        msg = ExtendedPose()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw/2)
        msg.pose.orientation.w = math.cos(yaw/2)

        # compute time/v, w depending on mode
        if self.mode == "speeds":
            t_rot = abs(seg["delta_angle"])/self.user_w if self.user_w else 0.0
            t_lin = seg["distance"]/self.user_v if self.user_v else 0.0
            msg.linear_velocity = float(self.user_v)
            msg.angular_velocity = float(math.copysign(
                self.user_w, seg["delta_angle"]))
            msg.time_stamp = float(t_rot + t_lin)

        else:  # time mode
            extra = max(0.0, self.total_time - self.T_min)
            seg_min = abs(seg["delta_angle"])/self.max_ang + \
                seg["distance"]/self.max_lin
            if seg_min > 0:
                ratio = seg_min / self.T_min
                t_rot = (abs(seg["delta_angle"])/self.max_ang) + extra * \
                    ratio * (abs(seg["delta_angle"])/self.max_ang)/seg_min
                t_lin = (seg["distance"]/self.max_lin) + extra * \
                    ratio * (seg["distance"]/self.max_lin)/seg_min
            else:
                t_rot = t_lin = 0.0
            v = seg["distance"]/t_lin if t_lin else 0.0
            w = abs(seg["delta_angle"])/t_rot if t_rot else 0.0
            msg.linear_velocity = float(v)
            msg.angular_velocity = math.copysign(w, seg["delta_angle"])
            msg.time_stamp = float(t_rot + t_lin)

        self.get_logger().info(
            f"Pub seg {self.current_segment+1}/{len(self.segments)} → "
            f"pos=({x:.2f},{y:.2f}), yaw={yaw:.2f}, "
            f"v={msg.linear_velocity:.2f}, w={msg.angular_velocity:.2f}, t={msg.time_stamp:.2f}"
        )
        self.create_publisher(ExtendedPose, 'pose', 10).publish(msg)
        self.current_segment += 1


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
