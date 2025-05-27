import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class PreProcessCamNode(Node):
    def __init__(self):
        super().__init__("preprocess_cam_node")

        # Subscription from the video source.
        self.subscription = self.create_subscription(
            Image, "/video_source/raw", self.listener_callback, 10
        )

        # Publisher for the pre-processed image.
        self.publisher = self.create_publisher(Image, "/pre_processed_cam", 10)
        self.bridge = CvBridge()

        # Initialize image dimensions (height, width).
        self.h, self.w = 240, 320
        # Generate the radial mask for vignette correction once during initialization.
        self.magenta_mask = self.generate_radial_mask(self.w, self.h)

    def generate_radial_mask(self, w, h):
        # Calculate the center of the image.
        cx, cy = w // 2, h // 2
        # Create a grid of coordinates.
        Y, X = np.ogrid[:h, :w]
        # Calculate the distance of each pixel from the center.
        distance = np.sqrt((X - cx) ** 2 + (Y - cy) ** 2)
        # Calculate the maximum possible distance from the center (e.g., to a corner).
        max_dist = np.sqrt(cx**2 + cy**2)
        # Normalize the distance to a range of 0 to 1.
        radial_mask = distance / max_dist
        # Create an attenuation factor that decreases from the center outwards.
        # 40% is the strength of the vignette correction.
        attenuation = 1 - 0.4 * radial_mask
        # Clip the attenuation values to a specific range [0.6, 1.0] to avoid over-correction or under-correction.
        return np.clip(attenuation, 0.6, 1.0).astype(np.float32)

    def listener_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image (RGB8 format).
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Flip the image horizontally and vertically.
        frame = cv2.flip(frame, -1)

        # Apply color correction to the image channels.
        r, g, b = cv2.split(frame)
        r = (r.astype(np.float32) * self.magenta_mask).astype(np.uint8)
        b = (b.astype(np.float32) * self.magenta_mask).astype(np.uint8)
        g = (g.astype(np.float32) * 0.9).astype(np.uint8)
        # Merge the corrected channels back into an RGB image.
        processed_frame = cv2.merge((r, g, b))

        # Publish the processed image.
        out_msg = self.bridge.cv2_to_imgmsg(processed_frame, encoding="rgb8")
        # Preserve the header from the original message (timestamp, frame_id).
        out_msg.header = msg.header
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PreProcessCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
