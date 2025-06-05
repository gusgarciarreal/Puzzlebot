import cv2
from ultralytics import YOLO

# from ultralytics.solutions.solutions import SolutionAnnotator # Keep if you plan to use it
from ultralytics.utils.plotting import colors

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference


class SignalDetectorNode(Node):

    def __init__(self):
        super().__init__("signal_detector_node")

        # Declare parameters
        self.declare_parameter(
            "model_path",
            "/home/ggm/Documents/sexto/Puzzlebot/reto/src/signals_detection/signals_detection/bestvol2.pt",
        )
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.1)
        self.declare_parameter("image_size", 320)
        self.declare_parameter("log_on_state_change", True)

        # Get parameters
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.confidence_threshold = (
            self.get_parameter("confidence_threshold")
            .get_parameter_value()
            .double_value
        )
        self.iou_threshold = (
            self.get_parameter("iou_threshold").get_parameter_value().double_value
        )
        self.image_size = (
            self.get_parameter("image_size").get_parameter_value().integer_value
        )
        self.log_on_state_change = (
            self.get_parameter("log_on_state_change").get_parameter_value().bool_value
        )

        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.yolov8_inference = Yolov8Inference()

        # Para el logging por cambio de estado
        self.previous_detected_classes_set = (
            frozenset()
        )  # Inicialmente, no se ha detectado nada

        self.subscription = self.create_subscription(
            Image, "/pre_processed_cam", self.camera_callback, 10
        )

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        self.get_logger().info(f"Signal Detector Node started with model: {model_path}")
        if self.log_on_state_change:
            self.get_logger().info("Logging will occur only on detection state change.")

    def camera_callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        results = self.model(
            source=img,
            show=False,
            conf=self.confidence_threshold,
            save=False,
            imgsz=self.image_size,
            iou=self.iou_threshold,
            verbose=False,  # Para silenciar los logs internos de Ultralytics
        )

        display_img = img.copy()

        boxes = results[0].boxes.xyxy.cpu()
        detected_classes_ids = results[0].boxes.cls.cpu().tolist()
        confidences = results[0].boxes.conf.cpu().tolist()

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()
        self.yolov8_inference.yolov8_inference.clear()

        # Construir el conjunto actual de clases detectadas
        current_detected_classes_list = []
        for box, cls_id, conf in zip(boxes, detected_classes_ids, confidences):
            x1, y1, x2, y2 = map(int, box)

            inference_result = InferenceResult()
            class_name = self.model.names[int(cls_id)]
            inference_result.class_name = class_name
            current_detected_classes_list.append(
                class_name
            )  # Añadir a la lista para el estado actual

            inference_result.top = int(y1)
            inference_result.left = int(x1)
            inference_result.bottom = int(y2)
            inference_result.right = int(x2)
            inference_result.area = int((x2 - x1) * (y2 - y1))

            self.yolov8_inference.yolov8_inference.append(inference_result)

            # Dibujo manual
            label = f"{class_name} {conf:.2f}"
            color = colors(int(cls_id), True)
            cv2.rectangle(display_img, (x1, y1), (x2, y2), color, 2)
            text_y = y1 - 10 if y1 - 10 > 10 else y1 + 20
            cv2.putText(
                display_img,
                label,
                (x1, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

        # Convertir la lista de clases actuales a un frozenset para comparación
        # Usamos un conjunto para ignorar el orden y los duplicados, y lo ordenamos antes para consistencia en el log
        current_detected_classes_set = frozenset(current_detected_classes_list)

        # Lógica de logging por cambio de estado
        if self.log_on_state_change:
            if current_detected_classes_set != self.previous_detected_classes_set:
                if not current_detected_classes_set:
                    self.get_logger().info(
                        "Detection state changed: No signals detected."
                    )
                else:
                    # Ordenamos la lista solo para el mensaje de log, para que sea consistente
                    sorted_current_classes = sorted(list(current_detected_classes_set))
                    self.get_logger().info(
                        f"Detection state changed: Now seeing {sorted_current_classes}"
                    )
                self.previous_detected_classes_set = current_detected_classes_set

        # Publicar imagen de depuración
        img_msg = self.bridge.cv2_to_imgmsg(display_img, encoding="bgr8")
        img_msg.header = data.header
        self.img_pub.publish(img_msg)

        # Publicar inferencias solo si hay alguna
        if self.yolov8_inference.yolov8_inference:
            self.yolov8_pub.publish(self.yolov8_inference)


def main(args=None):
    rclpy.init(args=args)
    node = SignalDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Signal Detector Node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
