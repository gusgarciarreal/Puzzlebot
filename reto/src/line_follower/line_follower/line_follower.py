import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class LineFollower(Node):
    """
    ROS2 node for line following using computer vision and a PID controller.
    Processes camera images to detect the line and publishes velocity commands
    for the robot to follow it. Also publishes a boolean status indicating
    whether the line is currently detected.
    """

    def __init__(self):
        super().__init__("line_follower")

        # Declare and get ROS2 parameters for configuration
        self.declare_parameter('kp', 0.008)
        self.declare_parameter('kd', 0.001)
        self.declare_parameter('dt', 0.1) # PID update rate, should match image topic rate
        self.declare_parameter('image_height', 240)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('canny_sigma', 0.05) # Common value is 0.33, but 0.05 works very well
        self.declare_parameter('line_timeout_seconds', 1.0)
        self.declare_parameter('linear_speed', 0.09)
        self.declare_parameter('angular_speed_limit', 0.8)
        self.declare_parameter('linear_speed_error_factor', 0.01)
        self.declare_parameter('min_line_angle_deg', 25.0)
        self.declare_parameter('max_line_angle_deg', 155.0)
        self.declare_parameter('hough_threshold', 30)
        self.declare_parameter('hough_min_line_length', 15)
        self.declare_parameter('hough_max_line_gap', 13)
        self.declare_parameter('roi_bottom_width_offset', 50)
        self.declare_parameter('clahe_clip_limit', 2.0)
        self.declare_parameter('clahe_tile_grid_size', [8, 8])
        self.declare_parameter('gaussian_blur_kernel_size', [11, 11])
        self.declare_parameter('gaussian_blur_sigma_x', 2.0)


        # Assign parameters to class attributes
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.dt = self.get_parameter('dt').value
        self.h = self.get_parameter('image_height').value
        self.w = self.get_parameter('image_width').value
        self.canny_sigma = self.get_parameter('canny_sigma').value
        self.line_timeout_duration = Duration(seconds=self.get_parameter('line_timeout_seconds').value)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').value
        self.linear_speed_error_factor = self.get_parameter('linear_speed_error_factor').value
        self.min_line_angle_deg = self.get_parameter('min_line_angle_deg').value
        self.max_line_angle_deg = self.get_parameter('max_line_angle_deg').value
        self.hough_threshold = self.get_parameter('hough_threshold').value
        self.hough_min_line_length = self.get_parameter('hough_min_line_length').value
        self.hough_max_line_gap = self.get_parameter('hough_max_line_gap').value
        self.roi_bottom_width_offset = self.get_parameter('roi_bottom_width_offset').value
        clahe_tile_grid_size = self.get_parameter('clahe_tile_grid_size').value
        gaussian_blur_kernel_size = self.get_parameter('gaussian_blur_kernel_size').value
        gaussian_blur_sigma_x = self.get_parameter('gaussian_blur_sigma_x').value

        # ROS2 publishers and subscribers configuration
        qos = rclpy.qos.qos_profile_sensor_data
        self.sub_img = self.create_subscription(
            Image, "/pre_processed_cam", self.img_callback, qos
        )
        self.pub_cmd = self.create_publisher(Twist, "/line_follower/raw_cmd_vel", 10)
        self.pub_debug = self.create_publisher(Image, "/line_follower/debug", 10)

        # Publisher for line detection status
        self.pub_line_status = self.create_publisher(
            Bool, "/line_follower/line_detected_status", 10
        )

        self.bridge = CvBridge()
        self.prev_err = 0.0

        # Region of Interest (ROI) mask
        self.tri_mask = self.build_triangle_mask(self.w, self.h)

        # Adaptive contrast enhancement object (CLAHE)
        self.clahe = cv2.createCLAHE(
            clipLimit=self.get_parameter('clahe_clip_limit').value,
            tileGridSize=tuple(clahe_tile_grid_size)
        )
        self.gaussian_blur_kernel_size = tuple(gaussian_blur_kernel_size)
        self.gaussian_blur_sigma_x = gaussian_blur_sigma_x

        # Variables for line status publisher
        self.last_line_seen_time = self.get_clock().now()
        self.line_currently_detected_flag = False

        # Pre-calculate ROI visualization points (Avoid Redundant Calculations)
        self.roi_visualization_pts = np.array(
            [
                [
                    (self.roi_bottom_width_offset, self.h - 1),
                    (self.w - self.roi_bottom_width_offset, self.h - 1),
                    (self.w // 2, self.h // 2),
                ]
            ],
            dtype=np.int32,
        )

        # Publish initial status (False)
        self._publish_line_status()

    def _publish_line_status(self):
        """Publishes the current line detection status."""
        status_msg = Bool()
        status_msg.data = self.line_currently_detected_flag
        self.pub_line_status.publish(status_msg)

    def build_triangle_mask(self, w: int, h: int) -> np.ndarray:
        """
        Builds a triangular mask to define the Region of Interest (ROI)
        in the lower part of the image.
        """
        mask = np.zeros((h, w), dtype=np.uint8)
        # Defines the triangle vertices for the ROI
        pts = np.array(
            [
                [
                    (self.roi_bottom_width_offset, h - 1),
                    (w - self.roi_bottom_width_offset, h - 1),
                    (w // 2, h // 2),
                ]
            ],
            dtype=np.int32,
        )
        cv2.fillPoly(mask, pts, 255)

        return mask

    def get_adaptive_canny_thresholds(self, image: np.ndarray):
        """
        Calculates adaptive thresholds for Canny edge detection
        based on the image intensity median.
        Uses a configurable sigma value.
        """
        median = np.median(image)
        lower = int(max(0, (1.0 - self.canny_sigma) * median))
        upper = int(min(255, (1.0 + self.canny_sigma) * median))
        return lower, upper

    def _preprocess_image(self, frame: np.ndarray) -> np.ndarray:
        """
        Performs image preprocessing for edge detection.
        Includes grayscale conversion, contrast enhancement, blurring,
        and Canny edge detection. The ROI mask is applied to the Canny output.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Applies CLAHE for local image contrast enhancement.
        clahe_output = self.clahe.apply(gray)

        # Blurs the image before Canny edge detection to reduce noise.
        blurred_image = cv2.GaussianBlur(
            clahe_output,
            self.gaussian_blur_kernel_size,
            self.gaussian_blur_sigma_x
        )

        # Detects edges using the Canny algorithm with adaptive thresholds.
        lower_canny, upper_canny = self.get_adaptive_canny_thresholds(blurred_image)
        edges = cv2.Canny(blurred_image, lower_canny, upper_canny)

        # Applies the Region of Interest (ROI) mask directly to the detected edges.
        edges_roi = cv2.bitwise_and(edges, self.tri_mask)

        return edges_roi

    def _detect_line(self, edges: np.ndarray) -> tuple[float | None, list]:
        """
        Detects lines in the edge image using the Hough Transform
        and calculates the average center of detected lines that meet the criteria.
        Prioritizes up to two valid lines.
        """
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap,
        )

        avg_line_center_x = None
        valid_lines_for_drawing = []
        candidate_mid_x_coords = []

        if lines is not None:
            # Process lines and filter them
            for line_arr in lines:
                x1, y1, x2, y2 = line_arr[0]

                # Calculates the line angle for filtering.
                angle_rad = np.arctan2(y2 - y1, x2 - x1)
                angle_deg = np.degrees(angle_rad)

                # Filters lines that are too horizontal.
                if abs(angle_deg) < self.min_line_angle_deg or abs(angle_deg) > self.max_line_angle_deg:
                    continue

                candidate_mid_x_coords.append((x1 + x2) / 2.0)
                valid_lines_for_drawing.append(((x1, y1), (x2, y2)))

                # Optimize Line Detection: Limit to up to 3 candidates
                # This ensures we don't process too many lines if not needed,
                # focusing on the most prominent ones.
                if len(candidate_mid_x_coords) >= 3:
                    break

            if candidate_mid_x_coords:
                avg_line_center_x = np.mean(candidate_mid_x_coords)

        return avg_line_center_x, valid_lines_for_drawing

    def _calculate_control(self, avg_line_center_x: float) -> Twist:
        """
        Calculates the robot's linear and angular velocities
        based on the line position error and a PID controller.
        """
        cmd = Twist()
        # Calculates the error: difference between line center and image center.
        err = avg_line_center_x - self.w / 2.0
        # Calculates the derivative term of the error.
        derr = (err - self.prev_err) / self.dt
        # Calculates angular velocity using the PD controller.
        omega = -(self.kp * err + self.kd * derr)
        omega = np.clip(omega, -self.angular_speed_limit, self.angular_speed_limit)

        # Dynamically adjusts linear speed: slower on sharp turns.
        linear_speed_factor = max(0.2, 1.0 - self.linear_speed_error_factor * abs(err))
        cmd.linear.x = self.linear_speed * linear_speed_factor

        cmd.angular.z = omega
        self.prev_err = err
        return cmd

    def _publish_debug_image(self, dbg_image: np.ndarray, original_msg_header):
        """
        Converts and publishes the debug image to a ROS2 topic.
        """
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(dbg_image, encoding="bgr8")
            debug_msg.header = original_msg_header
            self.pub_debug.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting debug image: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error publishing debug image: {e}")


    def img_callback(self, msg: Image):
        """
        Main callback function executed upon receiving a new image.
        Orchestrates image preprocessing, line detection, control calculation,
        and publication of commands and debug data.
        Also manages the line detection status publication.
        """
        current_time = self.get_clock().now()

        try:
            # Converts the ROS image message to OpenCV format.
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Resizes the image if it doesn't match expected dimensions.
            if frame.shape[0] != self.h or frame.shape[1] != self.w:
                frame = cv2.resize(
                    frame, (self.w, self.h), interpolation=cv2.INTER_AREA
                )
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error converting image: {e}")
            # Even if there's an image error, check for timeout
            if self.line_currently_detected_flag:
                time_since_last_seen = current_time - self.last_line_seen_time
                if time_since_last_seen >= self.line_timeout_duration:
                    self.line_currently_detected_flag = False
                    self.get_logger().info("Line lost due to image error and timeout.")
            self._publish_line_status()
            return
        except Exception as e:
            self.get_logger().error(f"Unexpected error processing image: {e}")
            if self.line_currently_detected_flag:
                time_since_last_seen = current_time - self.last_line_seen_time
                if time_since_last_seen >= self.line_timeout_duration:
                    self.line_currently_detected_flag = False
                    self.get_logger().info("Line lost due to image error and timeout.")
            self._publish_line_status()
            return

        # Defensive programming: Check if frame is valid after conversion
        if frame is None or frame.size == 0:
            self.get_logger().warn("Received empty or invalid image frame.")
            if self.line_currently_detected_flag:
                time_since_last_seen = current_time - self.last_line_seen_time
                if time_since_last_seen >= self.line_timeout_duration:
                    self.line_currently_detected_flag = False
                    self.get_logger().info("Line lost due to empty frame and timeout.")
            self._publish_line_status()
            return


        dbg_image = frame.copy()

        # Calls the function to preprocess the image, getting the edges directly.
        edges = self._preprocess_image(frame)

        # Calls the function to detect the line in the edge image.
        avg_line_center_x, valid_lines_for_drawing = self._detect_line(edges)

        cmd = Twist()
        line_detected_in_current_frame = False

        if avg_line_center_x is not None:
            # If a line is detected, calculate velocity commands.
            cmd = self._calculate_control(avg_line_center_x)
            line_detected_in_current_frame = True

            # Draws detected lines and reference point on the debug image.
            for p1, p2 in valid_lines_for_drawing:
                cv2.line(dbg_image, p1, p2, (0, 255, 0), 1)

            ref_y_for_center_dot = int(self.h * 0.85) # Y at 85% of img height
            cv2.circle(
                dbg_image,
                (int(avg_line_center_x), ref_y_for_center_dot),
                5, # Circle radius
                (0, 255, 0),
                -1,
            )
            cv2.circle(
                dbg_image, (self.w // 2, ref_y_for_center_dot), 5, (255, 255, 255), -1
            )
            cv2.line(
                dbg_image,
                (int(avg_line_center_x), ref_y_for_center_dot),
                (self.w // 2, ref_y_for_center_dot),
                (0, 0, 255),
                2,
            )
        else:
            # If no line is detected, stop the robot and reset the error.
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.prev_err = 0.0
            cv2.putText(
                dbg_image,
                "No line detected",
                (20, self.h - 20), # Text position
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, # Font scale
                (0, 0, 255),
                2, # Thickness
                cv2.LINE_AA,
            )

        # Line detection status logic
        previous_flag_state = self.line_currently_detected_flag

        if line_detected_in_current_frame:
            self.last_line_seen_time = current_time
            self.line_currently_detected_flag = True
        else:
            if self.line_currently_detected_flag:
                time_since_last_seen = current_time - self.last_line_seen_time
                if time_since_last_seen >= self.line_timeout_duration:
                    self.line_currently_detected_flag = False
                    self.get_logger().info("Line lost due to timeout.")

        # Publish if the status changed.
        if previous_flag_state != self.line_currently_detected_flag:
            self._publish_line_status()

        # Publishes velocity commands for the robot.
        self.pub_cmd.publish(cmd)

        # Draws the ROI contour (Avoid Redundant Calculations)
        cv2.polylines(
            dbg_image,
            self.roi_visualization_pts,
            isClosed=True,
            color=(0, 255, 255),
            thickness=1,
        )

        # Publishes the final debug image.
        self._publish_debug_image(dbg_image, msg.header)


def main(args=None):
    """
    Main function that initializes the ROS2 node and keeps it active.
    """
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On exit, ensure the robot stops.
        stop_cmd = Twist()
        node.pub_cmd.publish(stop_cmd)

        # Ensure False is published on shutdown if the line is not detected.
        node.line_currently_detected_flag = False
        node._publish_line_status()
        node.get_logger().info(
            "Shutting down, stopping robot, and setting line status to False."
        )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()