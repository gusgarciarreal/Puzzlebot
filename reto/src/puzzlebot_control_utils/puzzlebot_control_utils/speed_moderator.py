#!/usr/bin/env python3
"""
Nodo Moderador de Velocidad para un Robot Móvil.

Este nodo actúa como un árbitro, tomando decisiones de alto nivel sobre la 
velocidad del robot. Se suscribe a la velocidad "cruda" del seguidor de línea,
al estado de detección de la línea, a las señales de tráfico detectadas (YOLO)
y al estado de los semáforos.

Basado en un conjunto de reglas de prioridad, publica el comando de velocidad
final en el tópico `/cmd_vel`.

Lógica de Prioridades:
1.  **Parada Total (Máxima Prioridad):** Si se detecta una señal de 'stop' o un semáforo en rojo.
2.  **Maniobras Especiales:** Si se pierde la línea, ejecuta acciones basadas en la última señal vista (giro, seguir recto, espera en 'giveway').
3.  **Reducción de Velocidad:** Si se detectan señales como 'work', 'giveway' o un semáforo en amarillo.
4.  **Operación Normal:** Si no se cumple ninguna condición anterior, simplemente pasa la velocidad del seguidor de línea.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from yolov8_msgs.msg import Yolov8Inference
import time
from enum import Enum, auto

# --- Constantes para una mejor configuración ---
FINAL_VEL_TOPIC = '/cmd_vel'
RAW_VEL_TOPIC = '/line_follower/raw_cmd_vel'
LINE_STATUS_TOPIC = '/line_follower/line_detected_status'
SIGNALS_TOPIC = '/Yolov8_Inference'
TRAFFIC_LIGHT_TOPIC = '/traffic_light_status'


class NodeState(Enum):
    """Define los posibles estados operativos del robot para un logging claro."""
    FOLLOWING_LINE = auto()
    STOPPED_RED_LIGHT = auto()
    STOPPED_SIGN = auto()
    PERFORMING_TURN = auto()
    HANDLING_LOST_LINE_STRAIGHT = auto()
    WAITING_AT_GIVEAWAY = auto()
    REDUCED_SPEED = auto()
    PASS_THROUGH = auto() # Estado por defecto o sin acción específica
    UNKNOWN = auto()


class TrafficLightState(Enum):
    """Mapea el Int32 del nodo de semáforos a un estado legible."""
    NONE = 0
    GREEN = 1
    YELLOW = 2
    RED = 3


class SpeedModeratorNode(Node):
    """
    Modera la velocidad del robot basándose en la detección de líneas, 
    señales de tráfico y semáforos.
    """
    def __init__(self):
        super().__init__('speed_moderator')

        # --- Parámetros Configurables del Nodo ---
        self.declare_parameter('turn_linear_speed', 0.07)
        self.declare_parameter('turn_angular_speed', 0.5)
        self.declare_parameter('turn_duration_seconds', 3.0)
        self.declare_parameter('giveaway_wait_seconds', 2.0)
        self.declare_parameter('speed_reduction_factor', 0.5) # Reduce la velocidad al 50%
        self.declare_parameter('update_frequency', 20.0) # Hz para el bucle principal

        # Obtener valores de los parámetros
        self._turn_linear_speed = self.get_parameter('turn_linear_speed').value
        self._turn_angular_speed = self.get_parameter('turn_angular_speed').value
        self._turn_duration = self.get_parameter('turn_duration_seconds').value
        self._giveaway_wait_duration = self.get_parameter('giveaway_wait_seconds').value
        self._speed_reduction_factor = self.get_parameter('speed_reduction_factor').value
        update_period = 1.0 / self.get_parameter('update_frequency').value

        # --- Publicadores ---
        self._cmd_vel_publisher = self.create_publisher(Twist, FINAL_VEL_TOPIC, 10)

        # --- Suscriptores ---
        self.create_subscription(Twist, RAW_VEL_TOPIC, self._raw_vel_callback, 10)
        self.create_subscription(Bool, LINE_STATUS_TOPIC, self._line_status_callback, 10)
        self.create_subscription(Yolov8Inference, SIGNALS_TOPIC, self._signals_callback, 10)
        self.create_subscription(Int32, TRAFFIC_LIGHT_TOPIC, self._traffic_light_callback, 10)

        # --- Variables de Estado Interno ---
        self._current_raw_vel = Twist()
        self._is_line_detected = False
        self._active_signals = set()
        self._last_seen_maneuver_signal = None  # 'left', 'right', 'straight'
        self._traffic_light_state = TrafficLightState.NONE
        
        # Estado para maniobras temporizadas
        self._maneuver_start_time = None

        # Estado para el logging
        self._current_node_state = NodeState.PASS_THROUGH
        self._last_logged_state = None

        # --- Bucle Principal de Control ---
        self._control_timer = self.create_timer(update_period, self._update_loop)

        self.get_logger().info('Speed Moderator node has started.')

    # --- Callbacks para actualizar el estado ---

    def _raw_vel_callback(self, msg: Twist):
        self._current_raw_vel = msg

    def _line_status_callback(self, msg: Bool):
        self._is_line_detected = msg.data

    def _signals_callback(self, msg: Yolov8Inference):
        self._active_signals = {inf.class_name for inf in msg.yolov8_inference}
        # Guardar la última señal de maniobra vista para usarla si se pierde la línea
        maneuver_signals = {'left', 'right', 'straight'}
        current_maneuver_signals = self._active_signals.intersection(maneuver_signals)
        if current_maneuver_signals:
            self._last_seen_maneuver_signal = current_maneuver_signals.pop()
    
    def _traffic_light_callback(self, msg: Int32):
        try:
            self._traffic_light_state = TrafficLightState(msg.data)
        except ValueError:
            self._traffic_light_state = TrafficLightState.NONE

    # --- Bucle Principal de Lógica ---

    def _update_loop(self):
        """
        Bucle principal que se ejecuta a una frecuencia fija.
        Determina y publica la velocidad final del robot.
        """
        final_vel = Twist()
        new_state = NodeState.PASS_THROUGH # Estado por defecto

        # --- LÓGICA DE DECISIÓN POR PRIORIDADES ---

        # 1. MÁXIMA PRIORIDAD: CONDICIONES DE PARADA TOTAL
        is_stopped_by_sign = 'stop' in self._active_signals
        is_stopped_by_light = self._traffic_light_state == TrafficLightState.RED

        if is_stopped_by_sign or is_stopped_by_light:
            final_vel.linear.x = 0.0
            final_vel.angular.z = 0.0
            new_state = NodeState.STOPPED_SIGN if is_stopped_by_sign else NodeState.STOPPED_RED_LIGHT
            # Si estamos parados, reseteamos cualquier maniobra pendiente
            self._maneuver_start_time = None
        
        # 2. SEGUNDA PRIORIDAD: MANEJO DE PÉRDIDA DE LÍNEA
        elif not self._is_line_detected:
            # Si no hay línea, la velocidad cruda será cero. Decidimos si hacer una maniobra.
            final_vel = self._handle_lost_line()
            # El estado se determina dentro de la función de manejo
            if final_vel.angular.z != 0.0:
                new_state = NodeState.PERFORMING_TURN
            elif final_vel.linear.x != 0.0:
                new_state = NodeState.HANDLING_LOST_LINE_STRAIGHT
            elif self._maneuver_start_time is not None: # Indica que estamos esperando en giveaway
                 new_state = NodeState.WAITING_AT_GIVEAWAY
            else:
                new_state = NodeState.PASS_THROUGH # Línea perdida, sin señal, se detiene
        
        # 3. TERCERA PRIORIDAD: CONDICIONES DE OPERACIÓN NORMAL (LÍNEA DETECTADA)
        else:
            # Empezamos con la velocidad del seguidor de línea
            final_vel = self._current_raw_vel
            # Reseteamos la última señal de maniobra ya que vemos la línea de nuevo
            self._last_seen_maneuver_signal = None 
            self._maneuver_start_time = None

            # Aplicar reducción de velocidad si es necesario
            is_work_signal = 'work' in self._active_signals
            is_giveway_signal = 'giveway' in self._active_signals
            is_yellow_light = self._traffic_light_state == TrafficLightState.YELLOW
            
            if is_work_signal or is_giveway_signal or is_yellow_light:
                final_vel.linear.x *= self._speed_reduction_factor
                final_vel.angular.z *= self._speed_reduction_factor
                new_state = NodeState.REDUCED_SPEED
            else:
                # Si vemos luz verde, o no hay semáforo ni señales de reducción, velocidad normal.
                new_state = NodeState.FOLLOWING_LINE

        # Publicar la velocidad final calculada
        self._cmd_vel_publisher.publish(final_vel)
        
        # Registrar el cambio de estado si ha ocurrido
        self._log_state_change(new_state)

    def _handle_lost_line(self) -> Twist:
        """
        Gestiona el comportamiento del robot cuando no se detecta la línea.
        Devuelve el comando de velocidad a ejecutar.
        """
        cmd = Twist()

        # Caso GIVEWAY: esperar 2 segundos detenido
        if 'giveway' in self._active_signals:
            if self._maneuver_start_time is None:
                self._maneuver_start_time = time.time()
                self.get_logger().info("Giveaway: Line lost, waiting for 2 seconds.")
            
            if time.time() - self._maneuver_start_time < self._giveaway_wait_duration:
                return cmd # Devuelve velocidad cero mientras espera

        # Si la espera ha terminado o no era giveaway, resetea el timer
        self._maneuver_start_time = None

        # Casos de GIRO o RECTO
        if self._last_seen_maneuver_signal in ['left', 'right']:
            if self._maneuver_start_time is None:
                self._maneuver_start_time = time.time() # Iniciar temporizador de giro

            if time.time() - self._maneuver_start_time < self._turn_duration:
                cmd.linear.x = self._turn_linear_speed
                cmd.angular.z = self._turn_angular_speed if self._last_seen_maneuver_signal == 'left' else -self._turn_angular_speed
            else:
                # El tiempo de giro terminó, detener la maniobra
                self._last_seen_maneuver_signal = None
                self._maneuver_start_time = None
        
        elif self._last_seen_maneuver_signal == 'straight':
            cmd.linear.x = self._turn_linear_speed # Usamos la misma velocidad lineal
            cmd.angular.z = 0.0

        # Si no hay ninguna señal de maniobra, la velocidad es cero (devuelve cmd por defecto)
        return cmd

    def _log_state_change(self, new_state: NodeState):
        """
        Registra un mensaje en el logger solo si el estado del nodo ha cambiado.
        """
        if new_state != self._last_logged_state:
            self.get_logger().info(f"State changed to: {new_state.name}")
            self._last_logged_state = new_state

    def _shutdown_hook(self):
        """Publica un comando de detención al apagar el nodo."""
        self.get_logger().info("Shutting down Speed Moderator. Sending stop command.")
        self._cmd_vel_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SpeedModeratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()