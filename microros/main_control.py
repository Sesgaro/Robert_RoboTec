import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from inputs import get_gamepad, devices

# Constantes de configuración
AXIS_THRESHOLD = 0.01  # Umbral para sticks (valores normalizados entre -1 y 1)
TRIGGER_THRESHOLD = 0.01  # Umbral para gatillos (valores normalizados entre 0 y 1)
CHANGE_THRESHOLD = 0.01  # Umbral para detectar cambios significativos
TRIGGER_RANGE_MAX = 60  # Rango máximo para gatillos (0 a 60)
JOYSTICK_RANGE = (-90, 90)  # Rango para joysticks (-90 a 90)

class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')

        # Verificar si hay controles conectados
        if not devices.gamepads:
            self.get_logger().error("No se detectó ningún control. Conecta un joystick/gamepad y reinicia el programa.")
            rclpy.shutdown()
            exit(1)

        self.get_logger().info("Controles detectados:")
        for device in devices.gamepads:
            self.get_logger().info(f"- {device.name}")

        # Publicadores
        self.rpm_publisher = self.create_publisher(Float32MultiArray, 'motor_rpms', 10)
        self.angle_publisher = self.create_publisher(Float32MultiArray, 'steering_angles', 10)

        # Diccionarios para almacenar el estado de los ejes y gatillos
        self.current_values = {
            "sticks": {"ABS_X": 0.0, "ABS_Y": 0.0, "ABS_RX": 0.0, "ABS_RY": 0.0},
            "triggers": {"ABS_Z": 0.0, "ABS_RZ": 0.0}
        }

        # Variables para almacenar los últimos valores enviados
        self.last_trigger_values = (0, 0)  # (trigger_left, trigger_right)
        self.last_joy_values = (0, 0)  # (joy1_x, joy2_x)

        self.get_logger().info("Leyendo joycons (sticks) y triggers para controlar motores...")

    def map_range(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def run(self):
        while rclpy.ok():
            # Leer eventos del gamepad
            events = get_gamepad()

            # Procesar solo el último evento por tipo para evitar retrasos
            latest_events = {}
            for event in events:
                if event.ev_type != "Absolute":
                    continue
                latest_events[event.code] = event

            # Actualizar valores con los eventos más recientes
            for code, event in latest_events.items():
                if code in ("ABS_X", "ABS_Y", "ABS_RX", "ABS_RY"):
                    axis_value = event.state / 32768.0  # Normalizar a [-1, 1]
                    prev_value = self.current_values["sticks"][code]
                    if abs(axis_value) > AXIS_THRESHOLD and abs(axis_value - prev_value) > CHANGE_THRESHOLD:
                        self.current_values["sticks"][code] = axis_value
                    elif abs(axis_value) <= AXIS_THRESHOLD and abs(prev_value) > AXIS_THRESHOLD:
                        self.current_values["sticks"][code] = 0.0

                elif code in ("ABS_Z", "ABS_RZ"):
                    trigger_value = event.state / 255.0 if event.state != 0 else 0.0  # Normalizar a [0, 1]
                    prev_value = self.current_values["triggers"][code]
                    if trigger_value > TRIGGER_THRESHOLD and abs(trigger_value - prev_value) > CHANGE_THRESHOLD:
                        self.current_values["triggers"][code] = trigger_value
                    elif trigger_value <= TRIGGER_THRESHOLD and prev_value > TRIGGER_THRESHOLD:
                        self.current_values["triggers"][code] = 0.0

            # Mapear valores para motores y encoders
            trigger_left = (self.map_range(self.current_values["triggers"]["ABS_Z"], 0, 1, 0, TRIGGER_RANGE_MAX) // 5) * 5
            trigger_right = (self.map_range(self.current_values["triggers"]["ABS_RZ"], 0, 1, 0, TRIGGER_RANGE_MAX) // 5) * 5
            joy1_x = (self.map_range(self.current_values["sticks"]["ABS_X"], -1, 1, JOYSTICK_RANGE[0], JOYSTICK_RANGE[1]) // 5) * 5
            joy2_x = (self.map_range(self.current_values["sticks"]["ABS_RX"], -1, 1, JOYSTICK_RANGE[0], JOYSTICK_RANGE[1]) // 5) * 5

            # Publicar RPM para los motores solo si han cambiado
            current_trigger_values = (trigger_left, trigger_right)
            if current_trigger_values != self.last_trigger_values:
                rpm_msg = Float32MultiArray()
                if trigger_left > 0 and trigger_right > 0:
                    rpm_msg.data = [0.0, 0.0, 0.0, 0.0]
                elif trigger_left > 0:
                    rpm_msg.data = [-trigger_left, -trigger_left, -trigger_left, -trigger_left]
                elif trigger_right > 0:
                    rpm_msg.data = [trigger_right, trigger_right, trigger_right, trigger_right]
                else:
                    rpm_msg.data = [0.0, 0.0, 0.0, 0.0]
                self.rpm_publisher.publish(rpm_msg)
                self.last_trigger_values = current_trigger_values

            # Publicar ángulos para los encoders solo si han cambiado
            current_joy_values = (joy1_x, joy2_x)
            if current_joy_values != self.last_joy_values:
                angle_msg = Float32MultiArray()
                angle_msg.data = [joy1_x, joy2_x, joy1_x, joy2_x]  # Mismo ángulo para todas las llantas
                self.angle_publisher.publish(angle_msg)
                self.last_joy_values = current_joy_values

            rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Programa terminado por el usuario")
        # Publicar valores de parada
        rpm_msg = Float32MultiArray()
        rpm_msg.data = [0.0, 0.0, 0.0, 0.0]
        node.rpm_publisher.publish(rpm_msg)
        angle_msg = Float32MultiArray()
        angle_msg.data = [0.0, 0.0, 0.0, 0.0]
        node.angle_publisher.publish(angle_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
