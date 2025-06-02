import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollower(Node):
    def __init__(self):
        super().__init__("line_follower")

        # Inicializar suscriptor ROS a /pre_processed_cam, publicador para velocidades e imagen de depuración
        qos = rclpy.qos.qos_profile_sensor_data
        self.sub_img = self.create_subscription(
            Image, "/pre_processed_cam", self.img_callback, qos
        ) # Debería ser /image_raw o la fuente de la cámara
        self.pub_cmd = self.create_publisher(Twist, "/line_follower/raw_cmd_vel", 10)
        self.pub_debug = self.create_publisher(Image, "/line_follower/debug", 10)
        # Publicador para la imagen procesada antes de Canny (para depuración)
        self.pub_processed_mask = self.create_publisher(Image, "/line_follower/processed_mask", 10)


        self.bridge = CvBridge()
        self.prev_err = 0.0

        # Configurar ganancias del controlador PID (kp, kd) e intervalo del bucle de control (dt)
        self.kp, self.kd = 0.01, 0.002
        self.dt = 0.1  # (10 Hz)

        # Dimensiones de imagen esperadas
        self.h, self.w = 240, 320 # Asegúrate que coincida con la entrada real

        # Generar una máscara de región de interés triangular para la ROI
        self.tri_mask = self.build_triangle_mask(
            self.w, self.h, erosion_kernel_size=5
        )

        # Crear objeto CLAHE una vez
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def build_triangle_mask(
        self, w: int, h: int, erosion_kernel_size: int = 5
    ) -> np.ndarray:
        mask = np.zeros((h, w), dtype=np.uint8)
        # Vértices ajustados para ser un poco más anchos en la base y más altos
        # pts = np.array(
        #     [[(0, h - 1), (w - 1, h - 1), (w // 2, h // 2)]], dtype=np.int32
        # )
        # Vértices originales del usuario
        pts = np.array(
             [[(50, h - 1), (w - 50, h - 1), (w // 2, h // 2)]], dtype=np.int32
        )
        cv2.fillPoly(mask, pts, 255)

        if erosion_kernel_size > 0:
            kernel = np.ones((erosion_kernel_size, erosion_kernel_size), np.uint8)
            eroded_mask = cv2.erode(mask, kernel, iterations=1)
            return eroded_mask
        return mask

    def get_adaptive_canny_thresholds(self, image, sigma=0.2):
        """ Calcula umbrales alto y bajo para Canny basados en la mediana de la imagen """
        median = np.median(image)
        lower = int(max(0, (1.0 - sigma) * median))
        upper = int(min(255, (1.0 + sigma) * median))
        return lower, upper

    def img_callback(self, msg: Image):
        try:
            # Convertir ROS Image entrante a OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Redimensionar si es necesario (asegúrate que h y w coincidan)
            if frame.shape[0] != self.h or frame.shape[1] != self.w:
                 frame = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_AREA)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        dbg_image = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- MEJORAS EN EL PREPROCESAMIENTO ---
        # 1. Aplicar CLAHE para mejorar el contraste local
        clahe_output = self.clahe.apply(gray)
        # dbg_image[0:self.h//2, 0:self.w//2] = cv2.cvtColor(clahe_output[0:self.h//2, 0:self.w//2], cv2.COLOR_GRAY2BGR) # Visualizar CLAHE

        # 2. Umbralización adaptativa para aislar la línea (asumiendo línea oscura sobre fondo claro)
        # Blocksize debe ser impar. C es una constante que se resta de la media o media ponderada.
        # Probar diferentes blockSizes y C.
        # Si la línea es más clara que el fondo, usar cv2.THRESH_BINARY
        adaptive_thresh_mask = cv2.adaptiveThreshold(
            clahe_output, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 17, 2 # blockSize=17, C=4
        )
        # dbg_image[self.h//2:, 0:self.w//2] = cv2.cvtColor(adaptive_thresh_mask[self.h//2:, 0:self.w//2], cv2.COLOR_GRAY2BGR) # Visualizar adaptive_thresh

        # 3. Aplicar la máscara ROI triangular
        mask_roi = cv2.bitwise_and(adaptive_thresh_mask, self.tri_mask)

        # 4. Operaciones morfológicas para limpiar la máscara
        # La dilatación ya estaba, considera un 'opening' (erosión seguida de dilatación) para eliminar ruido pequeño
        kernel_open = np.ones((5, 5), np.uint8)
        mask_cleaned = cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN, kernel_open, iterations=1)

        kernel_dilate = np.ones((3,3), np.uint8) # El kernel original era (3,3)
        mask_processed = cv2.dilate(mask_cleaned, kernel_dilate, iterations=2) # Aumentar iteraciones si es necesario

        # Publicar la máscara procesada para depuración
        try:
            processed_mask_msg = self.bridge.cv2_to_imgmsg(mask_processed, encoding="mono8")
            processed_mask_msg.header = msg.header
            self.pub_processed_mask.publish(processed_mask_msg)
        except Exception as e:
            self.get_logger().error(f"Error converting processed mask image: {e}")


        # 5. Suavizar antes de Canny
        blur_for_canny = cv2.GaussianBlur(mask_processed, (5, 5), 0)

        # 6. Detección de bordes Canny con umbrales adaptativos
        lower_canny, upper_canny = self.get_adaptive_canny_thresholds(blur_for_canny)
        edges = cv2.Canny(blur_for_canny, lower_canny, upper_canny)
        # edges = cv2.Canny(blur_for_canny, 50, 150) # Umbrales originales para comparación

        # --- DETECCIÓN DE LÍNEAS HOUGH (sin cambios en los parámetros, pero la entrada 'edges' es mejor) ---
        lines = cv2.HoughLinesP(
            edges,
            rho=1,              # Resolución de distancia en píxeles
            theta=np.pi / 180,  # Resolución angular en radianes
            threshold=25,       # Mínimo número de votos (intersecciones en el espacio de Hough)
            minLineLength=20,   # Longitud mínima de la línea. Reducido un poco.
            maxLineGap=30       # Máximo espacio entre segmentos para unirlos. Aumentado un poco.
        )

        cmd = Twist()
        line_detected_successfully = False

        if lines is not None:
            candidate_mid_x_coords = []
            valid_lines_for_drawing = [] # Para dibujar solo las líneas que se usan para el cálculo

            for line_arr in lines:
                x1, y1, x2, y2 = line_arr[0]

                # --- Filtrado básico de líneas (Opcional, pero recomendado) ---
                # Evitar líneas muy horizontales si no se esperan
                angle_rad = np.arctan2(y2 - y1, x2 - x1)
                angle_deg = np.degrees(angle_rad)

                # Considerar líneas que no son demasiado horizontales (e.g., entre 20-160 y 200-340 grados)
                # Esto depende de cómo la cámara ve la línea.
                # Si la línea puede ser horizontal, este filtro debe eliminarse o ajustarse.
                if abs(angle_deg) < 20 or abs(angle_deg) > 160: # Filtra líneas casi horizontales
                     continue


                # Mantener líneas dentro de una porción vertical de la ROI (ej. no muy arriba)
                # if y1 < self.h * 0.6 or y2 < self.h * 0.6: # Línea demasiado alta en la ROI (cerca del vértice del triángulo)
                #      continue

                candidate_mid_x_coords.append((x1 + x2) / 2.0)
                valid_lines_for_drawing.append(((x1,y1),(x2,y2)))


            if candidate_mid_x_coords:
                avg_line_center_x = np.mean(candidate_mid_x_coords)

                for p1, p2 in valid_lines_for_drawing:
                    cv2.line(dbg_image, p1, p2, (0, 255, 0), 2) # Líneas verdes más gruesas

                err = avg_line_center_x - self.w / 2.0
                derr = (err - self.prev_err) / self.dt
                omega = -(self.kp * err + self.kd * derr)
                omega = np.clip(omega, -0.6, 0.6) # Ajusta según la velocidad máxima de tu robot

                # Velocidad lineal dinámica: más lento en curvas pronunciadas
                # Este factor puede necesitar ajuste
                linear_speed_factor = max(0.2, 1.0 - 0.01 * abs(err)) # Reduce la velocidad si el error es grande
                cmd.linear.x = 0.08 * linear_speed_factor # Velocidad base 0.08
                # cmd.linear.x = 0.04 if abs(err) > 25 else 0.08 # Lógica original con ajuste de umbral

                cmd.angular.z = omega
                self.prev_err = err
                line_detected_successfully = True

                ref_y_for_center_dot = int(self.h * 0.85) # Mover el punto de referencia más abajo
                cv2.circle(
                    dbg_image, (int(avg_line_center_x), ref_y_for_center_dot), 7, (0, 255, 0), -1,
                )
                cv2.circle(
                    dbg_image, (self.w // 2, ref_y_for_center_dot), 5, (255, 255, 255), -1,
                )
                cv2.line(
                    dbg_image,
                    (int(avg_line_center_x), ref_y_for_center_dot),
                    (self.w // 2, ref_y_for_center_dot),
                    (0, 0, 255), 2,
                )

        if not line_detected_successfully:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0 # Podrías implementar una estrategia de búsqueda (ej. rotar lentamente)
            self.prev_err = 0.0
            cv2.putText(
                dbg_image, "No line detected", (20, self.h - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA,
            )

        self.pub_cmd.publish(cmd)

        # Dibujar contorno de la ROI y bordes de Canny
        # pts_roi_visualization = np.array(
        #     [[(0, self.h - 1), (self.w - 1, self.h - 1), (self.w // 2, self.h // 2)]], dtype=np.int32,
        # )
        pts_roi_visualization = np.array( # Usar los mismos puntos que en build_triangle_mask
            [[(50, self.h - 1), (self.w - 50, self.h - 1), (self.w // 2, self.h // 2)]], dtype=np.int32,
        )
        cv2.polylines(dbg_image, pts_roi_visualization, isClosed=True, color=(0, 255, 255), thickness=1)

        # Superponer bordes de Canny en la imagen de depuración
        dbg_image[edges == 255] = [255, 0, 0] # Bordes Canny en azul

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(dbg_image, encoding="bgr8")
            debug_msg.header = msg.header
            self.pub_debug.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Error converting debug image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el robot al salir
        stop_cmd = Twist()
        node.pub_cmd.publish(stop_cmd)
        node.get_logger().info("Shutting down and stopping robot.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()