#!/usr/bin/env python3
import rclpy                                 # ROS Client Library for Python.
from rclpy.node import Node                  # Base class for creating ROS2 nodes.
import math                                  # For mathematical functions (sqrt, trigonometry, etc.).
from rcl_interfaces.msg import SetParametersResult   # Message type for parameter update results.

# Import the message type to be published (ExtendedPose is defined in the package extended_pose_interface).
from extended_pose_interface.msg import ExtendedPose

class PathGenerator(Node):
    def __init__(self):
        # Initialize the node with the name 'path_generator'
        super().__init__('path_generator')

        # Declare parameters with their default values.
        # 'waypoints': A string representation of a list of tuples representing path coordinates.
        self.declare_parameter("waypoints", "[(0,0),(1,0),(0.5,1)]")
        # 'mode': The mode to compute speeds, either by directly assigning speeds ("speeds") or time ("time").
        self.declare_parameter("mode", "speeds")  # Can be "speeds" or "time"
        # 'linear_velocity': The default linear velocity (in m/s) used in "speeds" mode.
        self.declare_parameter("linear_velocity", 0.2)  # m/s (for "speeds" mode)
        # 'angular_velocity': The default angular velocity (in rad/s) used in "speeds" mode.
        self.declare_parameter("angular_velocity", math.radians(30))  # rad/s (for "speeds" mode)
        # 'total_time': The total duration for the path (in seconds) used in "time" mode.
        self.declare_parameter("total_time", 0.0)  # s, only for "time" mode

        # Retrieve the initial parameters.
        # The waypoints parameter is parsed with the helper function parse_waypoints.
        self.waypoints = self.parse_waypoints(self.get_parameter("waypoints").value)
        # Validate that waypoints is a list with at least two points.
        if not isinstance(self.waypoints, list) or len(self.waypoints) < 2:
            self.get_logger().error("The initial waypoints must be a list with at least two points.")
            self.waypoints = []

        # Get the rest of the parameters.
        self.mode = self.get_parameter("mode").value.lower()  # Normalize mode to lowercase.
        self.user_linear_velocity = self.get_parameter("linear_velocity").value
        self.user_angular_velocity = self.get_parameter("angular_velocity").value
        self.total_time = self.get_parameter("total_time").value

        # Define maximum allowed velocities.
        self.max_linear = 0.3                # Maximum linear speed in m/s.
        self.max_angular = math.radians(90)    # Maximum angular speed in rad/s (90°/s).

        # Create a publisher for the topic '/pose' to publish ExtendedPose messages.
        self.pose_pub = self.create_publisher(ExtendedPose, 'pose', 10)

        # Internal variables for managing the path segments.
        self.current_segment = 0             # Index of the current segment to be published.
        self.segments = []                   # List that will store each segment of the path.
        self.initial_yaw = 0.0               # Assumed initial orientation (0 radians).

        # Prepare the segments based on provided waypoints.
        self.prepare_segments()
        if not self.segments:
            self.get_logger().error("Could not compute the path. Please check the waypoints.")

        # Validate the parameters according to the selected mode.
        if self.mode == "speeds":
            # In "speeds" mode, check whether the provided speeds exceed the maximum limits.
            if self.user_linear_velocity > self.max_linear or abs(self.user_angular_velocity) > self.max_angular:
                self.get_logger().warn("Provided speeds exceed allowed limits. The path is ignored.")
                return
        elif self.mode == "time":
            # In "time" mode, compute the minimum total time required by using maximum speeds for each segment.
            self.T_min_total = 0.0
            for seg in self.segments:
                # Minimum rotation time for the segment is the absolute angular difference divided by max angular speed.
                t_rot_min = abs(seg["delta_angle"]) / self.max_angular if self.max_angular > 0 else 0.0
                # Minimum linear time for the segment is the distance divided by max linear speed.
                t_lin_min = seg["distance"] / self.max_linear if self.max_linear > 0 else 0.0
                seg["t_rot_min"] = t_rot_min  # Save minimum rotation time.
                seg["t_lin_min"] = t_lin_min  # Save minimum linear time.
                seg["t_min"] = t_rot_min + t_lin_min  # Total minimum time for the segment.
                self.T_min_total += seg["t_min"]  # Accumulate the minimum times for all segments.
            # Check if the provided total_time is less than or equal to the minimum time required.
            if self.total_time <= self.T_min_total:
                self.get_logger().warn(
                    f"Provided total time ({self.total_time:.2f} s) is less than or equal to the required minimum ({self.T_min_total:.2f} s). The path is ignored."
                )
                return
        else:
            # If mode is neither 'speeds' nor 'time', log an error and exit.
            self.get_logger().error("Unknown mode. Use 'speeds' or 'time'.")
            return

        # Register a callback to handle dynamic parameter updates.
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create a timer that will publish segments sequentially at a rate of one segment per second.
        self.timer = self.create_timer(1.0, self.publish_next_segment)
        self.get_logger().info("Path generator node started.")

    def parse_waypoints(self, value):
        """
        Processes the 'waypoints' parameter.
        
        If the parameter is a string, it attempts to evaluate it with eval() to convert it into a Python object.
        If it is a list (array) of numbers, it is assumed to be a flat list and grouped in pairs to form coordinates.
        """
        new_waypoints = None
        if isinstance(value, str):
            try:
                new_waypoints = eval(value)  # Evaluate the string to obtain a Python object.
            except Exception as e:
                self.get_logger().error(f"Error evaluating waypoints: {e}")
        elif isinstance(value, list):
            # If the list is composed of numeric elements and has an even number of values, group them into pairs.
            if len(value) % 2 == 0:
                try:
                    it = iter(value)
                    # Group the elements two by two and convert each to float.
                    new_waypoints = [(float(next(it)), float(next(it))) for _ in range(len(value)//2)]
                except Exception as e:
                    self.get_logger().error(f"Error processing waypoints (numeric list): {e}")
            else:
                self.get_logger().error("The waypoints list does not have an even number of elements.")
        else:
            self.get_logger().error("The 'waypoints' parameter must be a string or a list.")
        return new_waypoints

    def prepare_segments(self):
        """
        Prepare the list of segments from the waypoints.
        
        For each pair of consecutive waypoints, a segment is created as a dictionary containing:
          - start: the starting point coordinates (x, y)
          - end: the ending point coordinates (x, y)
          - distance: the Euclidean distance between start and end
          - desired_yaw: the target orientation (in radians) computed from the segment direction
          - delta_angle: the change in orientation required from the previous segment
        """
        self.segments = []  # Reset segments list.
        previous_yaw = self.initial_yaw
        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i+1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            # Calculate the distance between the two points (Euclidean distance).
            distance = math.sqrt(dx**2 + dy**2)
            # Calculate the desired yaw (orientation) for the segment.
            desired_yaw = math.atan2(dy, dx)
            # Compute the required change in orientation (delta angle) relative to the previous yaw.
            delta_angle = desired_yaw - previous_yaw
            # Normalize delta_angle so that it remains between -pi and pi.
            delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))
            # Create a dictionary for the segment.
            seg = {
                "start": start,
                "end": end,
                "distance": distance,
                "desired_yaw": desired_yaw,
                "delta_angle": delta_angle
            }
            self.segments.append(seg)
            previous_yaw = desired_yaw  # Update previous_yaw for the next segment.

    def parameter_callback(self, params):
        """
        Callback function invoked when any parameter is updated dynamically.
        
        It checks for changes in the parameters (waypoints, mode, linear_velocity, angular_velocity, total_time)
        and if any relevant changes are detected, it recalculates the path segments and resets the publishing timer.
        """
        update_route = False  # Flag to track if the route needs to be updated.
        for param in params:
            if param.name == "waypoints":
                # Attempt to parse the new waypoints.
                new_waypoints = self.parse_waypoints(param.value)
                if new_waypoints is None or not isinstance(new_waypoints, list) or len(new_waypoints) < 2:
                    self.get_logger().error("The 'waypoints' parameter must be a valid list (or string) with at least two points.")
                else:
                    self.waypoints = new_waypoints
                    update_route = True
                    self.get_logger().info(f"Dynamically updated waypoints: {self.waypoints}")
            elif param.name == "mode":
                self.mode = param.value.lower()
                update_route = True
                self.get_logger().info(f"Updated mode to: {self.mode}")
            elif param.name == "linear_velocity":
                self.user_linear_velocity = param.value
                update_route = True
                self.get_logger().info(f"Updated linear velocity to: {self.user_linear_velocity}")
            elif param.name == "angular_velocity":
                self.user_angular_velocity = param.value
                update_route = True
                self.get_logger().info(f"Updated angular velocity to: {self.user_angular_velocity}")
            elif param.name == "total_time":
                self.total_time = param.value
                update_route = True
                self.get_logger().info(f"Updated total time to: {self.total_time}")

        if update_route:
            # Recalculate the path segments with the new parameters.
            self.prepare_segments()
            self.current_segment = 0  # Reset the current segment index.
            if self.mode == "time":
                # Recalculate the minimum time needed for each segment based on maximum speeds.
                self.T_min_total = 0.0
                for seg in self.segments:
                    t_rot_min = abs(seg["delta_angle"]) / self.max_angular if self.max_angular > 0 else 0.0
                    t_lin_min = seg["distance"] / self.max_linear if self.max_linear > 0 else 0.0
                    seg["t_rot_min"] = t_rot_min
                    seg["t_lin_min"] = t_lin_min
                    seg["t_min"] = t_rot_min + t_lin_min
                    self.T_min_total += seg["t_min"]
                # Check if the new total_time is sufficient.
                if self.total_time <= self.T_min_total:
                    self.get_logger().warn(
                        f"Total time ({self.total_time:.2f} s) is less than or equal to the minimum required ({self.T_min_total:.2f} s). The path is ignored."
                    )
                    result = SetParametersResult()
                    result.successful = False
                    return result

            # Restart the timer: destroy the existing timer (if any) and create a new timer.
            try:
                self.destroy_timer(self.timer)
            except Exception:
                pass
            self.timer = self.create_timer(1.0, self.publish_next_segment)
            self.get_logger().info("Route recalculated and timer restarted with the new parameters.")
        result = SetParametersResult()
        result.successful = True
        return result

    def publish_next_segment(self):
        """
        Publishes each segment of the path sequentially to the '/pose' topic.
        
        It calculates the final pose (position and orientation) for the segment.
        Depending on the current mode ("speeds" or "time"), it computes the linear and angular speeds
        as well as the total time stamp for the segment.
        """
        # Check if all segments have been published.
        if self.current_segment >= len(self.segments):
            self.get_logger().info("All segments have been published.")
            try:
                self.destroy_timer(self.timer)
            except Exception:
                pass
            return

        # Get the current segment information.
        seg = self.segments[self.current_segment]
        end = seg["end"]
        desired_yaw = seg["desired_yaw"]

        msg = ExtendedPose()  # Create a new ExtendedPose message.
        # Fill in the position (x,y) and a simplified quaternion (only yaw is considered).
        msg.pose.position.x = float(end[0])
        msg.pose.position.y = float(end[1])
        # Calculate quaternion components for yaw rotation.
        msg.pose.orientation.z = math.sin(desired_yaw / 2.0)
        msg.pose.orientation.w = math.cos(desired_yaw / 2.0)

        # Process based on the selected mode.
        if self.mode == "speeds":
            # In "speeds" mode, calculate the time for rotation and translation based on user-defined velocities.
            t_rot = abs(seg["delta_angle"]) / self.user_angular_velocity if self.user_angular_velocity != 0 else 0.0
            t_lin = seg["distance"] / self.user_linear_velocity if self.user_linear_velocity != 0 else 0.0
            total_seg_time = t_rot + t_lin
            # Assign the user defined speeds to the message.
            msg.linear_velocity = float(self.user_linear_velocity)
            # Set angular velocity with sign depending on the rotation direction.
            msg.angular_velocity = self.user_angular_velocity if seg["delta_angle"] >= 0 else -self.user_angular_velocity
            msg.time_stamp = float(total_seg_time)
        elif self.mode == "time":
            # In "time" mode, distribute the extra time proportionally over the segments.
            seg_min = seg["t_min"]
            extra_time_seg = (seg_min / self.T_min_total) * (self.total_time - self.T_min_total)
            if seg_min > 0:
                # Increase the rotation and translation times proportionally with the extra time.
                t_rot = seg["t_rot_min"] + extra_time_seg * (seg["t_rot_min"] / seg_min)
                t_lin = seg["t_lin_min"] + extra_time_seg * (seg["t_lin_min"] / seg_min)
            else:
                t_rot, t_lin = 0.0, 0.0
            total_seg_time = t_rot + t_lin
            # Calculate the linear and angular velocities based on the computed times.
            v = seg["distance"] / t_lin if t_lin > 0 else 0.0
            w = abs(seg["delta_angle"]) / t_rot if t_rot > 0 else 0.0
            # Adjust the direction of angular velocity based on the sign of delta_angle.
            w = w if seg["delta_angle"] >= 0 else -w
            # Verify if the calculated velocities are within allowed limits.
            if v > self.max_linear or abs(w) > self.max_angular:
                self.get_logger().warn("Calculated velocities for this segment exceed allowed limits. The path is ignored.")
                try:
                    self.destroy_timer(self.timer)
                except Exception:
                    pass
                return
            msg.linear_velocity = float(v)
            msg.angular_velocity = float(w)
            msg.time_stamp = float(total_seg_time)
        else:
            # Log an error if the mode is unknown.
            self.get_logger().error("Unknown mode when publishing segment.")
            return

        # Log the details of the current segment being published.
        self.get_logger().info(
            f"Segment {self.current_segment+1}/{len(self.segments)}: "
            f"Final Position=({end[0]:.2f}, {end[1]:.2f}), yaw={desired_yaw:.2f} rad, "
            f"v={msg.linear_velocity:.2f} m/s, w={msg.angular_velocity:.2f} rad/s, "
            f"time={msg.time_stamp:.2f} s"
        )
        # Publish the ExtendedPose message.
        self.pose_pub.publish(msg)
        # Move to the next segment.
        self.current_segment += 1

def main(args=None):
    # Initialize the ROS client library.
    rclpy.init(args=args)
    # Create the PathGenerator node.
    node = PathGenerator()
    try:
        # Keep the node running so it can process timer events and parameter updates.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow clean exit on Ctrl+C.
        pass
    # Clean up by destroying the node and shutting down rclpy.
    node.destroy_node()
    rclpy.shutdown()

# Execute the main function when the script is run directly.
if __name__ == '__main__':
    main()
