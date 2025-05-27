import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollowerCombined(Node):
    def __init__(self):
        super().__init__("line_follower_combined")

        # Inicializar suscriptor ROS a /pre_processed_cam, publicador para velocidades e imagen de depuración
        qos = rclpy.qos.qos_profile_sensor_data
        self.sub_img = self.create_subscription(
            Image, "/pre_processed_cam", self.img_callback, qos
        )
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_debug = self.create_publisher(Image, "/line_follower/debug", 10)

        self.bridge = CvBridge()
        self.prev_err = 0.0

        # Configurar ganancias del controlador PID (kp, kd) e intervalo del bucle de control (dt)
        self.kp, self.kd = 0.007, 0.002  # Constantes PID consistentes: 0.005 y 0.002
        self.dt = 0.1  # (10 Hz)

        # Dimensiones de imagen esperadas
        self.h, self.w = 240, 320

        # Generar una máscara de región de interés triangular erosionada para la ROI
        self.tri_mask_eroded = self.build_eroded_triangle_mask(
            self.w, self.h, kernel_size=7
        )

    def build_eroded_triangle_mask(
        self, w: int, h: int, kernel_size: int = 7
    ) -> np.ndarray:
        """
        Construye una máscara de imagen binaria (uint8) con forma de triángulo que cubre el campo de visión inferior,
        luego erosiona sus bordes para evitar falsas detecciones en los límites de la ROI.
        Vértices del triángulo: inferior-izquierda, inferior-derecha y punto medio a la mitad de la altura.
        """
        mask = np.zeros((h, w), dtype=np.uint8)
        pts = np.array([[(0, h - 1), (w - 1, h - 1), (w // 2, h // 2)]], dtype=np.int32)
        cv2.fillPoly(mask, pts, 255)

        # Aplicar erosión morfológica para encoger los límites de la ROI según el tamaño del kernel
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        eroded_mask = cv2.erode(mask, kernel, iterations=1)
        return eroded_mask

    def img_callback(self, msg: Image):
        # Convertir ROS Image entrante a OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # Crear una copia para mostrar los resultados de la detección
        dbg = frame.copy()

        # PASO 1: Aislar áreas potenciales de línea mediante umbralización de regiones oscuras
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_black = cv2.inRange(
            hsv, (0, 0, 0), (180, 255, 60)
        )  # Máscara binaria para píxeles oscuros (negros)

        # Restringir la detección a la región de interés triangular precalculada
        mask_roi = cv2.bitwise_and(mask_black, self.tri_mask_eroded)

        # Realizar dilatación para rellenar huecos dentro de la máscara de línea binaria
        kernel_dilate = np.ones((3, 3), np.uint8)
        mask_roi_dilated = cv2.dilate(mask_roi, kernel_dilate, iterations=1)

        # Suavizar los bordes de la máscara con desenfoque Gaussiano para mejorar la estabilidad de la detección de bordes
        blur_for_canny = cv2.GaussianBlur(mask_roi_dilated, (5, 5), 0)

        # Detectar bordes dentro de la máscara desenfocada usando el algoritmo Canny
        edges = cv2.Canny(blur_for_canny, 50, 150)

        # PASO 2: Identificar segmentos de línea mediante la transformada probabilística de Hough
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=25, minLineLength=40, maxLineGap=25
        )

        cmd = Twist()
        line_detected_successfully = False

        if lines is not None:
            candidate_mid_x_coords = []

            for line_arr in lines:
                x1, y1, x2, y2 = line_arr[0]
                # Calcular el punto medio horizontal del segmento de línea detectado
                line_mid_x = (x1 + x2) / 2.0
                candidate_mid_x_coords.append(line_mid_x)

                # Superponer cada segmento detectado por Hough en la imagen de depuración
                cv2.line(dbg, (x1, y1), (x2, y2), (0, 100, 0), 1)  # Líneas verde oscuro

            if candidate_mid_x_coords:  # Si se encontraron puntos medios
                # Calcular el promedio de todos los puntos medios de los segmentos como el centro estimado de la línea
                avg_line_center_x = np.mean(candidate_mid_x_coords)

                # Calcular la desviación lateral en píxeles desde el centro de la imagen
                err = avg_line_center_x - self.w / 2.0
                # Calcular el término derivativo y la velocidad angular ajustada por PID (omega)
                derr = (err - self.prev_err) / self.dt
                omega = -(self.kp * err + self.kd * derr)
                # Limitar omega a un rango seguro
                omega = np.clip(omega, -0.6, 0.6)

                # Establecer velocidad lineal hacia adelante y velocidad angular calculada
                cmd.linear.x = 0.05 if abs(err) > 17 else 0.11
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0
                cmd.angular.x = 0.0
                cmd.angular.y = 0.0
                cmd.angular.z = omega
                self.prev_err = err
                line_detected_successfully = True

                # Dibujar puntos de referencia y centro detectado en la imagen de depuración
                ref_y_for_center_dot = int(
                    self.h * 0.75
                )  # Altura Y de referencia para visualización
                cv2.circle(
                    dbg,
                    (int(avg_line_center_x), ref_y_for_center_dot),
                    6,
                    (0, 255, 0),
                    -1,
                )  # Verde para centro promedio
                cv2.circle(
                    dbg, (self.w // 2, ref_y_for_center_dot), 4, (255, 255, 255), -1
                )  # Blanco para objetivo
                cv2.line(
                    dbg,
                    (int(avg_line_center_x), ref_y_for_center_dot),
                    (self.w // 2, ref_y_for_center_dot),
                    (0, 0, 255),
                    2,
                )  # Línea de error roja

        if not line_detected_successfully:
            # Si no se detecta una línea válida, detener el robot y reiniciar el error PID
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = 0.0
            self.prev_err = 0.0  # Reiniciar error
            cv2.putText(
                dbg,
                "No line detected",
                # posicion de texto centrado abajo
                (20, self.h - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, # tamaño de fuente
                (0, 0, 255), # color rojo
                2, # grosor de la línea
                cv2.LINE_AA, # tipo de línea
            )

        # Publicar comando de velocidad calculado al controlador del robot
        self.pub_cmd.publish(cmd)

        # Dibujar el contorno original de la ROI triangular para depuración
        pts_roi_visualization = np.array(
            [[(0, self.h - 1), (self.w - 1, self.h - 1), (self.w // 2, self.h // 2)]],
            dtype=np.int32,
        )
        cv2.polylines(
            dbg, pts_roi_visualization, isClosed=True, color=(0, 255, 255), thickness=1
        )  # ROI en amarillo

        # Opcional: superponer la máscara procesada o los bordes en la imagen de depuración coloreando los píxeles detectados
        # Por ejemplo, para superponerla en rojo (si es monocromática):
        # dbg[mask_roi_dilated == 255] = [0,0,255] # Colorear en rojo los píxeles de la línea detectada

        # Convertir imagen de depuración a mensaje ROS y publicar para visualización
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
            debug_msg.header = msg.header
            self.pub_debug.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Error converting debug image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerCombined()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
