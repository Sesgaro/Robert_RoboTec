from inputs import get_gamepad, devices
import ESP32_S3_Dir as encoders
import ESP32_CAN_Motor as motors

# Constantes de configuración
SERIAL_PORT_MOTORS = 'COM6'
SERIAL_PORT_ENCODERS = 'COM8'
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1
AXIS_THRESHOLD = 0.01  # Umbral para sticks (valores normalizados entre -1 y 1)
TRIGGER_THRESHOLD = 0.01  # Umbral para gatillos (valores normalizados entre 0 y 1)
CHANGE_THRESHOLD = 0.01  # Umbral para detectar cambios significativos
TRIGGER_RANGE_MAX = 60  # Rango máximo para gatillos (0 a 60)
JOYSTICK_RANGE = (-90, 90)  # Rango para joysticks (-90 a 90)

# Verificar si hay controles conectados
if not devices.gamepads:
    print("No se detectó ningún control. Conecta un joystick/gamepad y reinicia el programa.")
    exit()

print("Controles detectados:")
for device in devices.gamepads:
    print(f"- {device.name}")

# Inicializar conexiones seriales para motores y encoders
try:
    CAN = motors.initialize_serial(SERIAL_PORT_MOTORS, BAUD_RATE, SERIAL_TIMEOUT)
    ESPDIR = encoders.initialize_serial(SERIAL_PORT_ENCODERS, BAUD_RATE, SERIAL_TIMEOUT)
except Exception as e:
    print(f"Error initializing serial connections: {e}")
    exit(1)

# Diccionarios para almacenar el estado de los ejes y gatillos
current_values = {
    "sticks": {"ABS_X": 0.0, "ABS_Y": 0.0, "ABS_RX": 0.0, "ABS_RY": 0.0},
    "triggers": {"ABS_Z": 0.0, "ABS_RZ": 0.0}
}

# Variables para almacenar los últimos valores enviados
last_trigger_values = (0, 0)  # (trigger_left, trigger_right)
last_joy_values = (0, 0)  # (joy1_x, joy2_x)

# Bucle principal
print("Leyendo joycons (sticks) y triggers para controlar motores... (presiona Ctrl+C para salir)")
try:
    while True:
        # Leer eventos del gamepad
        events = get_gamepad()

        # Procesar solo el último evento por tipo para evitar retrasos
        latest_events = {}
        for event in events:
            if event.ev_type != "Absolute":
                continue
            latest_events[event.code] = event  # Sobrescribe con el evento más reciente para cada código

        # Actualizar valores con los eventos más recientes
        for code, event in latest_events.items():
            if code in ("ABS_X", "ABS_Y", "ABS_RX", "ABS_RY"):
                axis_value = event.state / 32768.0  # Normalizar a [-1, 1]
                prev_value = current_values["sticks"][code]
                if abs(axis_value) > AXIS_THRESHOLD and abs(axis_value - prev_value) > CHANGE_THRESHOLD:
                    current_values["sticks"][code] = axis_value
                elif abs(axis_value) <= AXIS_THRESHOLD and abs(prev_value) > AXIS_THRESHOLD:
                    current_values["sticks"][code] = 0.0

            elif code in ("ABS_Z", "ABS_RZ"):
                trigger_value = event.state / 255.0 if event.state != 0 else 0.0  # Normalizar a [0, 1]
                prev_value = current_values["triggers"][code]
                if trigger_value > TRIGGER_THRESHOLD and abs(trigger_value - prev_value) > CHANGE_THRESHOLD:
                    current_values["triggers"][code] = trigger_value
                elif trigger_value <= TRIGGER_THRESHOLD and prev_value > TRIGGER_THRESHOLD:
                    current_values["triggers"][code] = 0.0

        # Mapear valores para motores y encoders
        trigger_left = (motors.map_range(current_values["triggers"]["ABS_Z"], 0, 1, 0, TRIGGER_RANGE_MAX) // 5) * 5
        trigger_right = (motors.map_range(current_values["triggers"]["ABS_RZ"], 0, 1, 0, TRIGGER_RANGE_MAX) // 5) * 5
        joy1_x = (motors.map_range(current_values["sticks"]["ABS_X"], -1, 1, JOYSTICK_RANGE[0], JOYSTICK_RANGE[1]) // 5) * 5
        joy2_x = (motors.map_range(current_values["sticks"]["ABS_RX"], -1, 1, JOYSTICK_RANGE[0], JOYSTICK_RANGE[1]) // 5) * 5

        # Enviar comandos para los gatillos solo si han cambiado
        current_trigger_values = (trigger_left, trigger_right)
        if current_trigger_values != last_trigger_values:
            try:
                if trigger_left > 0 and trigger_right > 0:
                    motors.send_rpm_command(CAN, 0, 0, 0, 0)
                elif trigger_left > 0:
                    motors.send_rpm_command(CAN, -trigger_left, -trigger_left, -trigger_left, -trigger_left)
                elif trigger_right > 0:
                    motors.send_rpm_command(CAN, trigger_right, trigger_right, trigger_right, trigger_right)
                else:
                    motors.send_rpm_command(CAN, 0, 0, 0, 0)
                last_trigger_values = current_trigger_values
            except Exception as e:
                print(f"Error controlling motors: {e}")

        # Enviar comandos para los joysticks solo si han cambiado
        current_joy_values = (joy1_x, joy2_x)
        if current_joy_values != last_joy_values:
            try:
                encoders.send_grd_command(ESPDIR, joy1_x, joy2_x, joy1_x, joy2_x)
                last_joy_values = current_joy_values
            except Exception as e:
                print(f"Error controlling encoders: {e}")

except KeyboardInterrupt:
    print("\nPrograma terminado por el usuario")
    try:
        motors.send_rpm_command(CAN, 0, 0, 0, 0)
        encoders.send_grd_command(ESPDIR, 0, 0, 0, 0)
        print("Motores y encoders detenidos")
    except Exception as e:
        print(f"Error durante la limpieza: {e}")
    exit(0)

except Exception as e:
    print(f"Error inesperado en el bucle principal: {e}")
    try:
        motors.send_rpm_command(CAN, 0, 0, 0, 0)
        encoders.send_grd_command(ESPDIR, 0, 0, 0, 0)
    except Exception as e:
        print(f"Error durante la limpieza: {e}")
    exit(1)
