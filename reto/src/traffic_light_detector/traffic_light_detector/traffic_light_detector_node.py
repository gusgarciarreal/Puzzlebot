#!/usr/bin/env python3
"""
Nodo de ROS 2 para la detección de semáforos en una imagen.

Este nodo se suscribe a un tópico de imagen, procesa cada cuadro para detectar
luces de semáforo y publica el estado detectado. El nodo registra un mensaje 
únicamente cuando el estado del semáforo cambia. Si el modo de depuración está 
activado, publica la imagen con las detecciones dibujadas en un tópico.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from enum import Enum

# --- Constantes para mejorar la legibilidad y mantenibilidad ---
IMAGE_TOPIC_SUB = '/pre_processed_cam'
STATUS_TOPIC_PUB = '/traffic_light_status'
DEBUG_IMAGE_TOPIC_PUB = '/light_detector/debug'

BGR_GREEN = (0, 255, 0)
BGR_YELLOW = (0, 255, 255)
BGR_RED = (0, 0, 255)

class TrafficLightState(Enum):
    """Enumeración para representar el estado del semáforo de forma clara."""
    GREEN = 1
    YELLOW = 2
    RED = 3
    NONE = 0

class TrafficLightDetectorNode(Node):
    """
    Detecta el estado de un semáforo a partir de un flujo de imágenes.
    """
    def __init__(self):
        super().__init__('traffic_light_detector_node')

        # --- Declaración y obtención de parámetros ---
        self._declare_parameters()
        self.hough_params = self._get_hough_parameters()
        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value

        # --- Suscriptores y Publicadores ---
        self.image_subscriber = self.create_subscription(
            Image, IMAGE_TOPIC_SUB, self.image_callback, 10
        )
        self.status_publisher = self.create_publisher(Int32, STATUS_TOPIC_PUB, 10)
        self.debug_image_publisher = self.create_publisher(Image, DEBUG_IMAGE_TOPIC_PUB, 10)

        # --- Herramientas e Inicialización de Estado ---
        self.bridge = CvBridge()
        self.kernel_erode = np.ones((3, 3), np.uint8)
        self.kernel_dilate = np.ones((7, 7), np.uint8)

        # Variable para almacenar el último estado y detectar cambios
        self.last_detected_state = TrafficLightState.NONE

        self.color_detection_config = [
            {'name': 'Red', 'state': TrafficLightState.RED, 'hsv_ranges': [((0, 120, 120), (5, 255, 255)), ((170, 120, 120), (179, 255, 255))], 'draw_color': BGR_RED},
            {'name': 'Yellow', 'state': TrafficLightState.YELLOW, 'hsv_ranges': [((12, 100, 120), (30, 255, 255))], 'draw_color': BGR_YELLOW},
            {'name': 'Green', 'state': TrafficLightState.GREEN, 'hsv_ranges': [((40, 100, 100), (80, 255, 255))], 'draw_color': BGR_GREEN}
        ]
        
        self.get_logger().info('Traffic Light Detector node started.')
        if self.debug_mode:
            self.get_logger().info(f"Debug mode is ON. Publishing debug images to {DEBUG_IMAGE_TOPIC_PUB}")

    def _declare_parameters(self):
        """Declara los parámetros de ROS para la configuración."""
        param_defaults = {
            'hough_dp': 1.5, 'hough_min_dist': 220, 'hough_param1': 60,
            'hough_param2': 25, 'hough_min_radius': 5, 'hough_max_radius': 15
        }
        for name, default in param_defaults.items():
            self.declare_parameter(name, default)

    def _get_hough_parameters(self) -> dict:
        """Obtiene los parámetros de Hough y los devuelve en un diccionario."""
        return {
            'dp': self.get_parameter('hough_dp').get_parameter_value().double_value,
            'minDist': self.get_parameter('hough_min_dist').get_parameter_value().integer_value,
            'param1': self.get_parameter('hough_param1').get_parameter_value().integer_value,
            'param2': self.get_parameter('hough_param2').get_parameter_value().integer_value,
            'minRadius': self.get_parameter('hough_min_radius').get_parameter_value().integer_value,
            'maxRadius': self.get_parameter('hough_max_radius').get_parameter_value().integer_value
        }

    def image_callback(self, msg: Image):
        """
        Callback principal. Procesa la imagen y gestiona los cambios de estado.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        debug_image = cv_image.copy() if self.debug_mode else None
        
        detected_state = TrafficLightState.NONE

        for config in self.color_detection_config:
            mask = self._create_color_mask(hsv_image, config['hsv_ranges'])
            processed_mask = self._apply_morphology(mask)
            circles = self._find_circles(processed_mask)

            if circles is not None:
                detected_state = config['state']
                if self.debug_mode:
                    self._draw_detections(debug_image, circles, config['draw_color'])
                break
        
        # --- Lógica de Registro de Cambio de Estado ---
        # Solo se enviará un log si el estado actual es diferente al del fotograma anterior.
        if detected_state != self.last_detected_state:
            self.get_logger().info(
                f"Traffic light state changed: {self.last_detected_state.name} -> {detected_state.name}"
            )
            # Actualizar el estado para la siguiente comparación.
            self.last_detected_state = detected_state

        # Publicar el estado actual (siempre, no solo en cambios)
        status_msg = Int32()
        status_msg.data = detected_state.value
        self.status_publisher.publish(status_msg)

        # Publicar la imagen de depuración si el modo está activado
        if self.debug_mode and debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_publisher.publish(debug_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Failed to convert debug image: {e}')

    def _create_color_mask(self, hsv_image: np.ndarray, hsv_ranges: list) -> np.ndarray:
        """Crea una máscara binaria para un color a partir de rangos HSV."""
        combined_mask = np.zeros_like(hsv_image[:, :, 0])
        for lower, upper in hsv_ranges:
            mask_part = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            combined_mask = cv2.bitwise_or(combined_mask, mask_part)
        return combined_mask

    def _apply_morphology(self, mask: np.ndarray) -> np.ndarray:
        """Aplica operaciones morfológicas para limpiar la máscara."""
        eroded_mask = cv2.erode(mask, self.kernel_erode, iterations=1)
        dilated_mask = cv2.dilate(eroded_mask, self.kernel_dilate, iterations=2)
        return dilated_mask

    def _find_circles(self, mask: np.ndarray) -> np.ndarray | None:
        """Ejecuta la Transformada de Círculos de Hough para encontrar círculos."""
        blurred_mask = cv2.GaussianBlur(mask, (9, 9), 2, 2)
        return cv2.HoughCircles(blurred_mask, cv2.HOUGH_GRADIENT, **self.hough_params)

    def _draw_detections(self, image: np.ndarray, circles: np.ndarray, color: tuple):
        """Dibuja los círculos detectados en una imagen de depuración."""
        circles_to_draw = np.uint16(np.around(circles))
        for i in circles_to_draw[0, :]:
            center = (i[0], i[1])
            radius = i[2]
            cv2.circle(image, center, radius, color, 2)
            cv2.circle(image, center, 2, BGR_RED, 3)

    def destroy_node(self):
        """Limpia los recursos al apagar el nodo."""
        self.get_logger().info('Shutting down the node.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shutting down due to KeyboardInterrupt.')
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()