#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrafficLightDetectorNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_node')

        # Parámetros para la detección de círculos
        self.declare_parameter('hough_dp', 1.2)
        self.declare_parameter('hough_min_dist', 100)
        self.declare_parameter('hough_param1', 50)
        self.declare_parameter('hough_param2', 10) # Parámetro actualizado
        self.declare_parameter('hough_min_radius', 10)
        self.declare_parameter('hough_max_radius', 70)

        self.hough_dp = self.get_parameter('hough_dp').get_parameter_value().double_value
        self.hough_min_dist = self.get_parameter('hough_min_dist').get_parameter_value().integer_value
        self.hough_param1 = self.get_parameter('hough_param1').get_parameter_value().integer_value
        self.hough_param2 = self.get_parameter('hough_param2').get_parameter_value().integer_value
        self.hough_min_radius = self.get_parameter('hough_min_radius').get_parameter_value().integer_value
        self.hough_max_radius = self.get_parameter('hough_max_radius').get_parameter_value().integer_value

        # Suscriptor a la imagen
        self.image_subscriber = self.create_subscription(
            Image,
            '/pre_processed_cam', # O el tópico de imagen que estés usando
            self.image_callback,
            10)

        # Publicador para el estado del semáforo
        self.status_publisher = self.create_publisher(Int32, '/traffic_light_status', 10)

        self.bridge = CvBridge()
        self.get_logger().info('Traffic Light Detector node has been started (Final Version).')

        # Rangos HSV (H: 0-179, S: 0-255, V: 0-255 para OpenCV) - Actualizados según tu solicitud
        self.hsv_ranges = {
            'red1': ((0, 100, 100), (10, 255, 255)),
            'red2': ((160, 100, 100), (179, 255, 255)),
            'yellow': ((10, 70, 100), (35, 255, 255)), # Rango de amarillo actualizado
            'green': ((40, 100, 100), (80, 255, 255)),
        }

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # El estado por defecto será 3 (rojo) si no se detecta nada concluyente.
        # La lógica de publicación se maneja dentro de detect_and_publish_if_found
        # o al final si no se encontró nada.

        # --- Detección de Rojo ---
        mask_red1 = cv2.inRange(hsv_image, self.hsv_ranges['red1'][0], self.hsv_ranges['red1'][1])
        mask_red2 = cv2.inRange(hsv_image, self.hsv_ranges['red2'][0], self.hsv_ranges['red2'][1])
        mask_red_combined = cv2.bitwise_or(mask_red1, mask_red2)
        processed_mask_red = self.apply_morphology(mask_red_combined)
        if self.detect_and_publish_if_found(processed_mask_red, 3, "Red"):
            return # Publicado rojo, no necesitamos seguir

        # --- Detección de Amarillo ---
        mask_yellow_combined = cv2.inRange(hsv_image, self.hsv_ranges['yellow'][0], self.hsv_ranges['yellow'][1])
        processed_mask_yellow = self.apply_morphology(mask_yellow_combined)
        if self.detect_and_publish_if_found(processed_mask_yellow, 2, "Yellow"):
            return # Publicado amarillo

        # --- Detección de Verde ---
        mask_green_combined = cv2.inRange(hsv_image, self.hsv_ranges['green'][0], self.hsv_ranges['green'][1])
        processed_mask_green = self.apply_morphology(mask_green_combined)
        if self.detect_and_publish_if_found(processed_mask_green, 1, "Green"):
            return # Publicado verde

        # Si no se detectó ningún color específico, publica el estado por defecto (3 = rojo)
        # default_status_msg = Int32()
        # default_status_msg.data = 3 # Rojo por defecto
        # self.status_publisher.publish(default_status_msg)
        # self.get_logger().debug('No specific light detected, publishing default status: 3 (Red)')


    def apply_morphology(self, mask):
        kernel_erode = np.ones((3,3), np.uint8)
        kernel_dilate = np.ones((7,7), np.uint8)
        mask = cv2.erode(mask, kernel_erode, iterations=1)
        mask = cv2.dilate(mask, kernel_dilate, iterations=2)
        return mask

    def detect_and_publish_if_found(self, mask, color_flag, color_name_for_log):
        blurred_mask = cv2.GaussianBlur(mask, (9, 9), 2, 2)

        circles = cv2.HoughCircles(
            blurred_mask,
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_min_dist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.hough_min_radius,
            maxRadius=self.hough_max_radius
        )

        if circles is not None:
            status_msg = Int32()
            status_msg.data = color_flag
            self.status_publisher.publish(status_msg)
            self.get_logger().info(f'Traffic light state: {color_name_for_log} ({color_flag}) detected.')
            return True # Indica que se detectó y publicó un color
        return False # No se detectaron círculos para este color

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # El logger se puede usar aquí si es necesario antes de apagar
        node.get_logger().info('Traffic Light Detector node shutting down due to KeyboardInterrupt.')
    finally:
        # Asegurarse de que el nodo se destruye correctamente
        if rclpy.ok(): # Solo destruir si rclpy sigue ok (no doble error)
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()