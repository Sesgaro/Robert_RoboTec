import serial
import time

# Configuración del puerto serial
SERIAL_PORT = 'COM6'  # Cambia esto al puerto correcto (por ejemplo, '/dev/ttyUSB0' en Linux/Mac)
BAUD_RATE = 115200    # Igual que en el código Arduino
TIMEOUT = 1           # Timeout para la conexión serial (en segundos)

def initialize_serial(port, baud_rate, timeout):
    """Inicializa la conexión serial y retorna el objeto serial."""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=timeout
        )
        print(f"Conectado al puerto {port} a {baud_rate} baudios.")
        time.sleep(2)  # Espera para que Arduino se inicialice
        return ser
    except serial.SerialException as e:
        print(f"Error al conectar al puerto {port}: {e}")
        return None

def send_grd_command(ser, rpm1, rpm2, rpm3, rpm4):
    """Envía una cadena con las direcciones en grados de las 4 llantas en el formato"""
    if ser is None or not ser.is_open:
        print("Error: No hay conexión serial activa.")
        return

    # Formatear la cadena
    command = f"D1:{rpm1};D2:{rpm2};D3:{rpm3};D4:{rpm4}\n"
    try:
        # Enviar la cadena codificada
        ser.write(command.encode('utf-8'))
        print(f"Enviado: {command.strip()}")
    except serial.SerialException as e:
        print(f"Error al enviar el comando: {e}")
