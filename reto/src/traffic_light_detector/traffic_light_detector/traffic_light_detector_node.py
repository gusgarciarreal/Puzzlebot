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
        self.declare_parameter('hough_dp', 1.5)
        self.declare_parameter('hough_min_dist', 150)
        self.declare_parameter('hough_param1', 40)
        self.declare_parameter('hough_param2', 20)
        self.declare_parameter('hough_min_radius', 10)
        self.declare_parameter('hough_max_radius', 25)

        self.hough_dp = self.get_parameter('hough_dp').get_parameter_value().double_value
        self.hough_min_dist = self.get_parameter('hough_min_dist').get_parameter_value().integer_value
        self.hough_param1 = self.get_parameter('hough_param1').get_parameter_value().integer_value
        self.hough_param2 = self.get_parameter('hough_param2').get_parameter_value().integer_value
        self.hough_min_radius = self.get_parameter('hough_min_radius').get_parameter_value().integer_value
        self.hough_max_radius = self.get_parameter('hough_max_radius').get_parameter_value().integer_value

        # Suscriptor a la imagen
        self.image_subscriber = self.create_subscription(
            Image,
            '/pre_processed_cam',
            self.image_callback,
            10)

        # Publicador para el estado del semáforo
        self.status_publisher = self.create_publisher(Int32, '/traffic_light_status', 10)

        self.bridge = CvBridge()
        self.get_logger().info('Traffic Light Detector node has been started (Final Version).')

        self.hsv_ranges = {
            'red1': ((0, 100, 120), (7, 255, 255)),
            'red2': ((160, 100, 120), (179, 255, 255)),
            'yellow': ((12, 120, 120), (40, 255, 255)),
            'green': ((40, 100, 100), (80, 255, 255)),
        }

        # Crear ventanas para depuración
        cv2.namedWindow("Processed Yellow Mask", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Detections", cv2.WINDOW_AUTOSIZE)
        # Si quieres ver otras máscaras, crea sus ventanas aquí también
        # cv2.namedWindow("Processed Red Mask", cv2.WINDOW_AUTOSIZE)
        # cv2.namedWindow("Processed Green Mask", cv2.WINDOW_AUTOSIZE)


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            debug_image = cv_image.copy() # Copia para dibujar detecciones
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Definir processed_mask_yellow con un valor por defecto (p.ej. vacía)
        # para que exista incluso si la detección de rojo retorna antes.
        # Opcionalmente, puedes crearla del tamaño de la imagen.
        processed_mask_yellow = np.zeros((cv_image.shape[0], cv_image.shape[1]), dtype=np.uint8)


        # --- Detección de Rojo ---
        mask_red1 = cv2.inRange(hsv_image, self.hsv_ranges['red1'][0], self.hsv_ranges['red1'][1])
        mask_red2 = cv2.inRange(hsv_image, self.hsv_ranges['red2'][0], self.hsv_ranges['red2'][1])
        mask_red_combined = cv2.bitwise_or(mask_red1, mask_red2)
        processed_mask_red = self.apply_morphology(mask_red_combined)
        # Opcional: cv2.imshow("Processed Red Mask", processed_mask_red)
        if self.detect_and_publish_if_found(processed_mask_red, 3, "Red", debug_image):
            cv2.imshow("Detections", debug_image)
            cv2.imshow("Processed Yellow Mask", processed_mask_yellow) # Muestra la máscara amarilla vacía/default
            cv2.waitKey(1)
            return 

        # --- Detección de Amarillo ---
        mask_yellow_combined = cv2.inRange(hsv_image, self.hsv_ranges['yellow'][0], self.hsv_ranges['yellow'][1])
        processed_mask_yellow = self.apply_morphology(mask_yellow_combined)
        cv2.imshow("Processed Yellow Mask", processed_mask_yellow) # <-- MOSTRAR MÁSCARA AMARILLA
        if self.detect_and_publish_if_found(processed_mask_yellow, 2, "Yellow", debug_image):
            cv2.imshow("Detections", debug_image)
            cv2.waitKey(1)
            return

        # --- Detección de Verde ---
        mask_green_combined = cv2.inRange(hsv_image, self.hsv_ranges['green'][0], self.hsv_ranges['green'][1])
        processed_mask_green = self.apply_morphology(mask_green_combined)
        # Opcional: cv2.imshow("Processed Green Mask", processed_mask_green)
        if self.detect_and_publish_if_found(processed_mask_green, 1, "Green", debug_image):
            cv2.imshow("Detections", debug_image)
            cv2.waitKey(1)
            return

        # Si no se detectó ningún color específico
        cv2.imshow("Detections", debug_image) # Muestra la imagen original sin nuevas detecciones
        cv2.imshow("Processed Yellow Mask", processed_mask_yellow) # Muestra la máscara amarilla (procesada pero sin círculos)
        cv2.waitKey(1)


    def apply_morphology(self, mask):
        kernel_erode = np.ones((3,3), np.uint8)
        kernel_dilate = np.ones((7,7), np.uint8) # Puede que quieras un kernel más pequeño para dilate
        mask = cv2.erode(mask, kernel_erode, iterations=1)
        mask = cv2.dilate(mask, kernel_dilate, iterations=2) # O menos iteraciones
        return mask

    def detect_and_publish_if_found(self, mask, color_flag, color_name_for_log, image_to_draw_on):
        # Aplicar GaussianBlur a la máscara antes de HoughCircles puede ayudar
        blurred_mask = cv2.GaussianBlur(mask, (9, 9), 2, 2)
        # Opcional: cv2.imshow(f"Blurred Mask for {color_name_for_log}", blurred_mask)

        circles = cv2.HoughCircles(
            blurred_mask, # Usar la máscara suavizada
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_min_dist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.hough_min_radius,
            maxRadius=self.hough_max_radius
        )

        detected_this_run = False
        if circles is not None:
            status_msg = Int32()
            status_msg.data = color_flag
            self.status_publisher.publish(status_msg)
            self.get_logger().info(f'Traffic light state: {color_name_for_log} ({color_flag}) detected.')
            
            circles_to_draw = np.uint16(np.around(circles))
            for i in circles_to_draw[0, :]:
                center = (i[0], i[1])
                radius = i[2]
                # Color para dibujar el círculo según el color detectado
                draw_color = (255, 0, 0) # Default azul
                if color_flag == 3: # Rojo
                    draw_color = (0, 0, 255)
                elif color_flag == 2: # Amarillo
                    draw_color = (0, 255, 255)
                elif color_flag == 1: # Verde
                    draw_color = (0, 255, 0)
                
                cv2.circle(image_to_draw_on, center, radius, draw_color, 2) # Círculo exterior
                cv2.circle(image_to_draw_on, center, 2, (0, 0, 255), 3)     # Punto central
            detected_this_run = True
        
        return detected_this_run

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Traffic Light Detector node shutting down due to KeyboardInterrupt.')
    finally:
        # Asegurarse de que las ventanas de OpenCV se destruyen
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()