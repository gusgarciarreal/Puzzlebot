import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollower(Node):
    """
    Nodo ROS2 para seguir una línea utilizando visión por computadora y un controlador PID.
    Procesa imágenes de la cámara para detectar la línea y publica comandos de velocidad
    para que el robot la siga.
    """

    def __init__(self):
        super().__init__("line_follower")

        # Configuración de suscriptores y publicadores ROS2
        qos = rclpy.qos.qos_profile_sensor_data
        self.sub_img = self.create_subscription(
            Image, "/pre_processed_cam", self.img_callback, qos
        )
        self.pub_cmd = self.create_publisher(Twist, "/line_follower/raw_cmd_vel", 10)
        self.pub_debug = self.create_publisher(Image, "/line_follower/debug", 10)
        # Se elimina el publicador para la máscara procesada

        self.bridge = CvBridge()
        self.prev_err = (
            0.0  # Error previo para el cálculo del término derivativo del PID
        )

        # Ganancias del controlador PID
        self.kp, self.kd = 0.008, 0.001
        self.dt = 0.1  # Intervalo de tiempo para el bucle de control (10 Hz)

        # Dimensiones de imagen esperadas (alto, ancho)
        self.h, self.w = 240, 320

        # Máscara de Región de Interés (ROI)
        self.tri_mask = self.build_triangle_mask(self.w, self.h)

        # Objeto para el ajuste de contraste adaptativo (CLAHE)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def build_triangle_mask(self, w: int, h: int) -> np.ndarray:
        """
        Construye una máscara triangular para definir la Región de Interés (ROI)
        en la parte inferior de la imagen.
        """
        mask = np.zeros((h, w), dtype=np.uint8)
        # Define los vértices del triángulo para la ROI
        pts = np.array(
            [[(50, h - 1), (w - 50, h - 1), (w // 2, h // 2)]], dtype=np.int32
        )
        cv2.fillPoly(mask, pts, 255)

        return mask

    def get_adaptive_canny_thresholds(self, image: np.ndarray, sigma: float = 0.05):
        """
        Calcula umbrales adaptativos para la detección de bordes Canny
        basados en la mediana de la intensidad de la imagen.
        """
        median = np.median(image)
        lower = int(max(0, (1.0 - sigma) * median))
        upper = int(min(255, (1.0 + sigma) * median))
        return lower, upper

    def _preprocess_image(self, frame: np.ndarray) -> np.ndarray:
        """
        Realiza el preprocesamiento de la imagen para la detección de bordes.
        Incluye conversión a escala de grises, mejora de contraste, suavizado
        y detección de bordes Canny. La máscara ROI se aplica a la salida de Canny.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Aplica CLAHE para mejorar el contraste local de la imagen.
        clahe_output = self.clahe.apply(gray)

        # Suaviza la imagen antes de la detección de bordes Canny para reducir el ruido.
        blurred_image = cv2.GaussianBlur(clahe_output, (17, 17), 2)

        # Detecta bordes utilizando el algoritmo de Canny con umbrales adaptativos.
        lower_canny, upper_canny = self.get_adaptive_canny_thresholds(blurred_image)
        edges = cv2.Canny(blurred_image, lower_canny, upper_canny)

        # Aplica la máscara de la Región de Interés (ROI) directamente a los bordes detectados.
        edges_roi = cv2.bitwise_and(edges, self.tri_mask)

        # Se elimina la publicación de la imagen de bordes/máscara procesada.

        return edges_roi

    def _detect_line(self, edges: np.ndarray) -> tuple[float | None, list]:
        """
        Detecta líneas en la imagen de bordes utilizando la Transformada de Hough
        y calcula el centro promedio de las líneas detectadas que cumplen los criterios.
        """
        lines = cv2.HoughLinesP(
            edges,
            rho=1,  # Resolución de distancia en píxeles
            theta=np.pi / 180,  # Resolución angular en radianes
            threshold=30,  # Mínimo número de votos en el espacio de Hough
            minLineLength=15,  # Longitud mínima de la línea detectada
            maxLineGap=13,  # Máximo espacio entre segmentos para unirlos
        )

        avg_line_center_x = None
        valid_lines_for_drawing = []

        if lines is not None:
            candidate_mid_x_coords = []
            for line_arr in lines:
                x1, y1, x2, y2 = line_arr[0]

                # Calcula el ángulo de la línea para filtrado.
                angle_rad = np.arctan2(y2 - y1, x2 - x1)
                angle_deg = np.degrees(angle_rad)

                # Filtra líneas que son demasiado horizontales.
                if abs(angle_deg) < 25 or abs(angle_deg) > 155:
                    continue

                candidate_mid_x_coords.append((x1 + x2) / 2.0)
                valid_lines_for_drawing.append(((x1, y1), (x2, y2)))

            if candidate_mid_x_coords:
                avg_line_center_x = np.mean(candidate_mid_x_coords)

        return avg_line_center_x, valid_lines_for_drawing

    def _calculate_control(self, avg_line_center_x: float) -> Twist:
        """
        Calcula las velocidades lineal y angular del robot
        basándose en el error de posición de la línea y un controlador PID.
        """
        cmd = Twist()
        # Calcula el error: diferencia entre el centro de la línea y el centro de la imagen.
        err = avg_line_center_x - self.w / 2.0
        # Calcula el término derivativo del error.
        derr = (err - self.prev_err) / self.dt
        # Calcula la velocidad angular usando el controlador PD.
        omega = -(self.kp * err + self.kd * derr)
        omega = np.clip(omega, -0.8, 0.8)  # Limita la velocidad angular máxima

        # Ajusta la velocidad lineal dinámicamente: más lento en curvas pronunciadas.
        linear_speed_factor = max(0.2, 1.0 - 0.01 * abs(err))
        cmd.linear.x = (
            0.09 * linear_speed_factor
        )  # Producto vel lineal y factor de curvas

        cmd.angular.z = omega
        self.prev_err = err  # Almacena el error actual para la siguiente iteración
        return cmd

    def _publish_debug_image(self, dbg_image: np.ndarray, original_msg_header):
        """
        Convierte y publica la imagen de depuración en un tópico ROS2.
        """
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(dbg_image, encoding="bgr8")
            debug_msg.header = original_msg_header
            self.pub_debug.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Error converting debug image: {e}")

    def img_callback(self, msg: Image):
        """
        Función de callback principal que se ejecuta cada vez que llega una nueva imagen.
        Orquesta el preprocesamiento, la detección de línea, el cálculo de control
        y la publicación de comandos y datos de depuración.
        """
        try:
            # Convierte el mensaje de imagen ROS a un formato OpenCV.
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Redimensiona la imagen si no coincide con las dimensiones esperadas.
            if frame.shape[0] != self.h or frame.shape[1] != self.w:
                frame = cv2.resize(
                    frame, (self.w, self.h), interpolation=cv2.INTER_AREA
                )
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        dbg_image = frame.copy()  # Crea una copia para dibujar elementos de depuración

        # Llama a la función para preprocesar la imagen, obteniendo los bordes directamente.
        edges = self._preprocess_image(frame)

        # Llama a la función para detectar la línea en la imagen de bordes.
        avg_line_center_x, valid_lines_for_drawing = self._detect_line(edges)

        cmd = Twist()
        line_detected_successfully = False

        if avg_line_center_x is not None:
            # Si se detectó una línea, calcula los comandos de velocidad.
            cmd = self._calculate_control(avg_line_center_x)
            line_detected_successfully = True

            # Dibuja las líneas detectadas y el punto de referencia en la imagen de depuración.
            for p1, p2 in valid_lines_for_drawing:
                cv2.line(dbg_image, p1, p2, (0, 255, 0), 2)

            ref_y_for_center_dot = int(self.h * 0.85)
            cv2.circle(
                dbg_image,
                (int(avg_line_center_x), ref_y_for_center_dot),
                7,
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
            # Si no se detecta la línea, detiene el robot y reinicia el error.
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.prev_err = 0.0
            cv2.putText(
                dbg_image,
                "No line detected",
                (20, self.h - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        # Publica los comandos de velocidad para el robot.
        self.pub_cmd.publish(cmd)

        # Dibuja el contorno de la ROI.
        pts_roi_visualization = np.array(
            [[(50, self.h - 1), (self.w - 50, self.h - 1), (self.w // 2, self.h // 2)]],
            dtype=np.int32,
        )
        cv2.polylines(
            dbg_image,
            pts_roi_visualization,
            isClosed=True,
            color=(0, 255, 255),
            thickness=1,
        )
        # Superpone los bordes de Canny en la imagen de depuración.
        # dbg_image[edges == 255] = [255, 0, 0]  # Resalta los bordes Canny en azul

        # Publica la imagen de depuración final.
        self._publish_debug_image(dbg_image, msg.header)


def main(args=None):
    """
    Función principal que inicializa el nodo ROS2 y lo mantiene activo.
    """
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al salir, asegura que el robot se detenga.
        stop_cmd = Twist()
        node.pub_cmd.publish(stop_cmd)
        node.get_logger().info("Shutting down and stopping robot.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
