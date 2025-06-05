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
        # Minimum area threshold for a detected sign to be considered valid.
        self.area_threshold = 5000

        # Boolean flag to indicate if the robot is currently executing a predefined sign action (e.g., a turn).
        self.executing_sign_action = False
        # Timestamp when a sign action started, used to control action duration.
        self.sign_action_start_time = None

        # NUEVA BANDERA: Para moverse recto después de señal "0_straight" hasta redetectar línea
        self.reacquiring_line_after_straight_sign = False
        self.straight_speed_on_reacquire = (
            0.15  # Velocidad lineal para avanzar recto (ajustable)
        )

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

    def yolo_inference_callback(
        self, msg: InferenceResult
    ):  # Asumiendo que es InferenceResult por detección
        if msg.area < self.area_threshold:
            # self.get_logger().info(f"Sign {msg.class_name} detected but area {msg.area} is below threshold {self.area_threshold}")
            # Si la señal es muy pequeña y era la que estaba activa, podríamos querer limpiarla si no hay otras más grandes.
            # Opcional: if msg.class_name == self.current_sign_flag: self.current_sign_flag = None
            return

        if self.executing_sign_action or self.reacquiring_line_after_straight_sign:
            # Si ya está en una acción de señal (giro o re-adquisición de línea),
            # ignorar nuevas detecciones de señales para no interrumpir la maniobra actual.
            # Podrías querer una lógica más sofisticada si una señal de STOP debe interrumpir todo.
            return

        valid_signs = ["0_straight", "3_turn_left", "4_turn_right", "1_stop"]
        if msg.class_name in valid_signs:
            # Solo actualiza si la nueva señal es diferente o si no había ninguna activa,
            # o si la nueva señal es más grande (esto es opcional y puede complicar).
            # Por simplicidad, la última señal válida detectada toma precedencia si no estamos en una acción.
            if self.current_sign_flag != msg.class_name:
                self.get_logger().info(
                    f"New sign detected: {msg.class_name} with area {msg.area}"
                )
            self.current_sign_flag = msg.class_name
            self.current_sign_area = msg.area
        # else:
        # Si se detecta una señal no válida y había una válida activa, podríamos querer limpiarla.
        # if self.current_sign_flag:
        #     self.get_logger().info(f"Previously active sign {self.current_sign_flag} no longer primary detection or invalid sign detected.")
        #     self.current_sign_flag = None

    def process_and_publish_cmd(self):
        cmd = Twist()

        # --- NUEVA PRIORIDAD MÁXIMA: Re-adquiriendo línea después de señal "0_straight" ---
        if self.reacquiring_line_after_straight_sign:
            if self.line_detected:
                self.get_logger().info(
                    "Line re-detected after 'straight' sign. Resuming normal control."
                )
                self.reacquiring_line_after_straight_sign = False
                if (
                    self.current_sign_flag == "0_straight"
                ):  # Limpiar la señal que activó este estado
                    self.current_sign_flag = None
                cmd = self.last_raw_cmd_vel  # Volver a los comandos del line_follower
                # No hacer return aquí, permitir que la lógica de semáforos (si aplica) evalúe este cmd.
            else:
                # Aún intentando re-adquirir la línea, moverse recto.
                self.get_logger().info("State: Re-acquiring line (moving straight).")
                cmd.linear.x = self.straight_speed_on_reacquire
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return  # Acción explícita, anular otra lógica para este ciclo.

        # --- Prioridad 1: Ejecutar acción de señal temporizada (Giros) ---
        if self.executing_sign_action:
            elapsed = time.time() - self.sign_action_start_time
            action_complete = False
            if self.current_sign_flag == "4_turn_right":
                if elapsed < 2.0:  # Avanzar un poco
                    cmd.linear.x = 0.15
                else:  # Girar
                    cmd.angular.z = -0.5
                    cmd.linear.x = 0.0
                    if elapsed > 3.5:
                        action_complete = True
            elif self.current_sign_flag == "3_turn_left":
                if elapsed < 2.0:  # Avanzar un poco
                    cmd.linear.x = 0.15
                else:  # Girar
                    cmd.angular.z = 0.5
                    cmd.linear.x = 0.0
                    if elapsed > 3.5:
                        action_complete = True

            if action_complete:
                self.get_logger().info(
                    f"Finished sign action for {self.current_sign_flag}."
                )
                self.executing_sign_action = False
                self.current_sign_flag = None
                # cmd se quedará con la última velocidad de la acción (probablemente (0,0) o giro)
                # Es importante publicar un Twist() vacío si queremos que se detenga completamente.
                cmd = Twist()  # Detenerse después de la acción de giro.

            self.cmd_pub.publish(cmd)
            return

        # --- Prioridad 2: Responder a señales de tráfico recién detectadas (no temporizadas o inicio de temporizadas) ---
        # Esta sección se alcanza si no estamos en reacquiring_line_after_straight_sign ni en executing_sign_action.
        if self.current_sign_flag == "1_stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # self.current_sign_flag no se limpia automáticamente para "1_stop".
            # El robot se detendrá mientras "1_stop" sea el flag activo.
            # Considerar una lógica para "reanudar" después de un "stop".

        elif self.current_sign_flag == "0_straight":
            # Si llegamos aquí, es una nueva detección de "0_straight" y no estamos re-adquiriendo línea.
            if not self.line_detected:
                self.get_logger().info(
                    "'0_straight' sign detected at intersection (no line). Initiating line re-acquisition."
                )
                self.reacquiring_line_after_straight_sign = True
                # La acción de moverse recto comenzará en el siguiente ciclo del timer
                # debido al bloque de MÁXIMA PRIORIDAD.
                # Para este ciclo, podemos detenernos o avanzar un poco.
                cmd.linear.x = (
                    self.straight_speed_on_reacquire
                )  # Empezar a moverse recto inmediatamente
                cmd.angular.z = 0.0
                # No limpiar current_sign_flag aquí; el estado de re-adquisición lo usará.
            else:
                # Señal "0_straight" detectada Y la línea está presente.
                self.get_logger().info(
                    "'0_straight' sign detected while on line. Continuing line following."
                )
                cmd = self.last_raw_cmd_vel
                self.current_sign_flag = (
                    None  # La instrucción de la señal se está cumpliendo.
                )

        elif self.current_sign_flag in ["3_turn_left", "4_turn_right"]:
            # Iniciar una acción de giro si no hay línea detectada.
            if not self.line_detected:
                self.get_logger().info(
                    f"'{self.current_sign_flag}' sign detected at intersection (no line). Initiating turn sequence."
                )
                self.executing_sign_action = True
                self.sign_action_start_time = time.time()
                # El giro comenzará en el próximo ciclo. Detenerse o avanzar un poco ahora:
                cmd.linear.x = 0.0  # Detenerse brevemente antes de la maniobra de giro.
                cmd.angular.z = 0.0
            else:
                # Señal de giro detectada, pero la línea aún está presente. Seguir línea.
                self.get_logger().info(
                    f"'{self.current_sign_flag}' sign detected, but line still present. Deferring turn."
                )
                cmd = self.last_raw_cmd_vel
                # No limpiar current_sign_flag, esperar a que la línea se pierda para iniciar el giro.

        # --- Prioridad 3: Responder al estado del semáforo ---
        # Se ejecuta si ninguna acción de señal está activa o configuró `cmd` y retornó.
        elif self.traffic_light_flag == 1:  # Verde
            # self.get_logger().info("Traffic light Green: Following line follower commands.")
            cmd = self.last_raw_cmd_vel
        elif self.traffic_light_flag == 2:  # Amarillo
            self.get_logger().info("Traffic light Yellow: Reducing speed.")
            cmd.linear.x = self.last_raw_cmd_vel.linear.x / 2.0
            cmd.angular.z = self.last_raw_cmd_vel.angular.z / 2.0
        elif self.traffic_light_flag == 3:  # Rojo
            self.get_logger().info("Traffic light Red: Stopping.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Caso por defecto: si no hay semáforo o ninguna condición anterior se cumplió
            # y cmd no fue modificado por una señal.
            if (
                self.current_sign_flag is None
                and not self.executing_sign_action
                and not self.reacquiring_line_after_straight_sign
            ):
                cmd = self.last_raw_cmd_vel

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
