import cv2
from ultralytics import YOLO
from ultralytics.solutions.solutions import SolutionAnnotator
from ultralytics.utils.plotting import colors

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__("camera_subscriber")

        # Assuming bestvol2.pt is correctly installed and located as per your previous fix.
        self.model = YOLO(
            "/home/ggm/Documents/sexto/Puzzlebot/reto/src/signals_detection/signals_detection/bestvol2.pt"
        )

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image, "/pre_processed_cam", self.camera_callback, 10
        )
        self.subscription

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")

        results = self.model(
            source=img, show=False, conf=0.5, save=False, imgsz=320, iou=0.2
        )
        # iou is the IoU threshold for non-max suppression
        # it is used to filter out overlapping bounding boxes based on their Intersection over Union (IoU) score.
        # so if two boxes overlap significantly, the one with the lower confidence score will be removed.
        
        # annotator = SolutionAnnotator(img) # This line is not used if results[0].plot() handles annotation

        boxes = results[0].boxes.xyxy.cpu()
        clss = results[0].boxes.cls.cpu().tolist()
        confs = results[0].boxes.conf.cpu().tolist()

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        # Clear previous inference results for the new frame
        self.yolov8_inference.yolov8_inference.clear()

        # Iterate directly over the zipped results (box, class, confidence)
        for box, cls, conf in zip(boxes, clss, confs):
            x1, y1, x2, y2 = map(int, box)

            self.inference_result = InferenceResult()

            # Corrected: Use 'cls' for class_name index and correct bounding box assignments
            self.inference_result.class_name = self.model.names[int(cls)]
            self.inference_result.top = int(y1)  # y1 is top
            self.inference_result.left = int(x1)  # x1 is left
            self.inference_result.bottom = int(y2)  # y2 is bottom
            self.inference_result.right = int(x2)  # x2 is right

            # Area calculation
            height = int(y2) - int(y1)
            width = int(x2) - int(x1)
            area = height * width
            self.inference_result.area = int(area)

            self.yolov8_inference.yolov8_inference.append(self.inference_result)

            self.yolov8_inference.yolov8_inference.append(self.inference_result)

            # # Corrected: Use self.model.names for labels
            # label = f"{self.model.names[cls]} {conf:.2f}"
            # color = colors(cls, True)
            # text_x = x1                         # misma posición horizontal que x1 (esquina izquierda)
            # text_y = y1 - 10 if y1 - 10 > 10 else y1 + 20

            color = colors(cls, True)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

            # cv2.putText(img, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX,0.6, color, 2, cv2.LINE_AA)

        # Corrected: Call get_logger() on the instance 'self'
        self.get_logger().info(f"YOLOv8 Inference Results: {self.yolov8_inference}")

        # results[0].plot() generates an annotated frame. If you've already drawn
        # on 'img' manually, these manual drawings will be overwritten by .plot() output
        # if 'annotated_frame' is used for publishing.
        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)


def main(args=None):
    rclpy.init(args=args)
    node = Camera_subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
