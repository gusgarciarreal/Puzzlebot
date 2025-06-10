import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from yolov8_msgs.msg import InferenceResult
import time


class SpeedModerator(Node):
    """
    The SpeedModerator node is responsible for arbitrating robot velocity commands
    based on various environmental inputs: line detection, traffic lights, and
    YOLOv8-detected traffic signs. It prioritizes actions based on a defined hierarchy.
    """

    def __init__(self):
        """
        Initializes the SpeedModerator node, sets up internal state variables,
        and creates subscribers and publishers.
        """
        super().__init__("speed_moderator")

        # --- System State Variables ---
        # Stores the last received raw command from the line follower, used as a base.
        self.last_raw_cmd_vel = Twist()
        # Traffic light status: 1=Green, 2=Yellow, 3=Red (default to Red for safety).
        self.traffic_light_flag = 3
        # Flag indicating if a line is currently detected by the line follower.
        self.line_detected = True
        # Stores the class name of the currently detected traffic sign (e.g., '1_stop').
        self.current_sign_flag = None
        # Stores the bounding box area of the current detected sign, used for filtering.
        self.current_sign_area = 0
        # Boolean flag to indicate if the robot is currently executing a predefined sign action (e.g., a turn).
        self.executing_sign_action = False
        # Timestamp when a sign action started, used to control action duration.
        self.sign_action_start_time = None
        # Minimum area threshold for a detected sign to be considered valid.
        self.area_threshold = 5000

        # --- Subscriptions ---
        # Subscribes to raw velocity commands from the line follower.
        self.create_subscription(
            Twist, "/line_follower/raw_cmd_vel", self.raw_cmd_vel_callback, 10
        )

        # Subscribes to the traffic light status.
        self.create_subscription(
            Int32, "/traffic_light_status", self.traffic_light_callback, 10
        )

        # Subscribes to the line detection flag.
        self.create_subscription(
            Int32, "/line_detected_flag", self.line_flag_callback, 10
        )

        # Subscribes to YOLOv8 inference results for traffic sign detection.
        self.create_subscription(
            InferenceResult, "/Yolov8_Inference", self.yolo_inference_callback, 10
        )

        # --- Publisher ---
        # Publishes the final moderated velocity commands to the robot.
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- Timer ---
        # Main timer that triggers the command processing and publishing logic.
        self.timer = self.create_timer(0.1, self.process_and_publish_cmd)

        self.get_logger().info("Speed Moderator node with YOLOv8 integration started.")

    # --- Callback Functions ---

    def raw_cmd_vel_callback(self, msg):
        """
        Callback for receiving raw velocity commands from the line follower.
        These commands serve as the default movement instructions.
        """
        self.last_raw_cmd_vel = msg

    def traffic_light_callback(self, msg):
        """
        Callback for receiving the traffic light status.
        Updates the internal traffic light flag.
        """
        self.traffic_light_flag = msg.data

    def line_flag_callback(self, msg):
        """
        Callback for receiving the line detected flag.
        Updates the internal boolean flag indicating line presence.
        """
        self.line_detected = bool(msg.data)

    def yolo_inference_callback(self, msg):
        """
        Callback for receiving YOLOv8 inference results.
        Processes detected traffic signs to determine robot behavior.
        """
        # Ignore signs if their detected area is too small (likely a distant or false positive).
        if msg.area < self.area_threshold:
            return

        # If already executing a sign action, ignore new sign detections to complete current action.
        if self.executing_sign_action:
            return

        # Define valid traffic signs that this node will respond to.
        valid_signs = ["0_straight", "3_turn_left", "4_turn_right", "1_stop"]

        # If a valid sign is detected, update the current sign flag and its area.
        if msg.class_name in valid_signs:
            self.current_sign_flag = msg.class_name
            self.current_sign_area = msg.area
            self.get_logger().info(
                f"Sign detected: {msg.class_name} with area {msg.area}"
            )

    def process_and_publish_cmd(self):
        """
        Main processing function that determines and publishes the robot's
        velocity commands based on a priority hierarchy:
        1. Executing a predefined sign action (highest priority).
        2. Responding to detected traffic signs.
        3. Responding to traffic light status (lowest priority).
        """
        cmd = Twist()  # Initialize an empty Twist message for commands

        # --- Priority 1: Execute Predefined Sign Action ---
        # If the robot is in the middle of a sign-triggered maneuver (e.g., a turn).
        if self.executing_sign_action:
            elapsed = time.time() - self.sign_action_start_time

            if self.current_sign_flag == "4_turn_right":
                # First, move straight for a bit to clear the intersection/position for the turn.
                if elapsed < 2.0:
                    cmd.linear.x = 0.15  # Move forward
                else:
                    # Then, execute the turn.
                    cmd.angular.z = -0.5  # Turn right (negative Z angular velocity)
                    cmd.linear.x = 0.0  # Stop linear movement during turn
                    # After a total duration, reset the action state.
                    if elapsed > 3.5:
                        self.executing_sign_action = False
                        self.current_sign_flag = None  # Clear the sign flag

            elif self.current_sign_flag == "3_turn_left":
                # Similar logic for turning left.
                if elapsed < 2.0:
                    cmd.linear.x = 0.15
                else:
                    cmd.angular.z = 0.5  # Turn left (positive Z angular velocity)
                    cmd.linear.x = 0.0
                    if elapsed > 3.5:
                        self.executing_sign_action = False
                        self.current_sign_flag = None

            self.cmd_pub.publish(cmd)  # Publish the command for the ongoing action.
            return  # Exit early as a higher priority action is being executed.

        # --- Priority 2: Respond to Detected Traffic Signs ---
        # If no sign action is currently in progress, check for new sign detections.
        if self.current_sign_flag == "1_stop":
            # 'Stop' sign: full stop.
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.current_sign_flag == "0_straight":
            # If a line is detected, let the line follower continue.
            # If no line is detected (e.g., intersection), use the last raw velocity to move forward.
            cmd = self.last_raw_cmd_vel
        elif self.current_sign_flag in ["3_turn_left", "4_turn_right"]:
            # 'Turn' signs: initiate a predefined turning action.
            # Only trigger the turn action if no line is detected (implies being at an intersection).
            if not self.line_detected:
                self.executing_sign_action = (
                    True  # Set flag to start executing the action.
                )
                self.sign_action_start_time = time.time()  # Record start time.
                return  # Exit this cycle; the turn action will begin in the next timer iteration (Priority 1).

        # --- Priority 3: Respond to Traffic Light Status ---
        # If no sign is active or being executed, apply traffic light logic.
        elif self.traffic_light_flag == 1:  # Green light: proceed with raw commands.
            cmd = self.last_raw_cmd_vel
        elif self.traffic_light_flag == 2:  # Yellow light: reduce speed by half.
            cmd.linear.x = self.last_raw_cmd_vel.linear.x / 2.0
            cmd.angular.z = self.last_raw_cmd_vel.angular.z / 2.0
        elif self.traffic_light_flag == 3:  # Red light: stop the robot.
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Publish the final determined command.
        self.cmd_pub.publish(cmd)


def main(args=None):
    """
    Main function to initialize the ROS 2 system and spin the SpeedModerator node.
    """
    rclpy.init(args=args)
    node = SpeedModerator()
    try:
        rclpy.spin(node)  # Keep the node alive and processing callbacks.
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully to shut down the node.
        pass
    finally:
        # Clean up resources before exiting.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
