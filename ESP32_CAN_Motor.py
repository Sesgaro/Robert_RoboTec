import serial
import time

# Configuración del puerto serial
SERIAL_PORT = 'COM6'  # Cambia esto al puerto correcto (por ejemplo, '/dev/ttyUSB0' en Linux/Mac)
BAUD_RATE = 115200    # Igual que en el código Arduino
TIMEOUT = 1           # Timeout para la conexión serial (en segundos)

def map_range(value, in_min, in_max, out_min, out_max):
    if abs(value) < 0.01:  # Consider values close to 0 as 0
        return 0
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


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

def send_rpm_command(ser, rpm1, rpm2, rpm3, rpm4):
    """Envía una cadena con los RPM de las 4 llantas en el formato M1:rpm1;M2:rpm2;M3:rpm3;M4:rpm4\n."""
    if ser is None or not ser.is_open:
        print("Error: No hay conexión serial activa.")
        return

    # Formatear la cadena
    command = f"M1:{rpm1};M2:{rpm2};M3:{rpm3};M4:{rpm4}\n"
    try:
        # Enviar la cadena codificada
        ser.write(command.encode('utf-8'))
        print(f"Enviado: {command.strip()}")
    except serial.SerialException as e:
        print(f"Error al enviar el comando: {e}")

# def main():
#     # Inicializar la conexión serial
#     ser = initialize_serial(SERIAL_PORT, BAUD_RATE, TIMEOUT)
#     if ser is None:
#         return

#     try:
#         # Ejemplo interactivo: pedir RPM al usuario
#         print("Ingresa los RPM para las llantas (M1, M2, M3, M4). Usa valores enteros (positivos o negativos).")
#         print("Presiona Ctrl+C para salir.")

#         while True:
#             try:
#                 rpm1 = int(input("RPM para M1: "))
#                 rpm2 = int(input("RPM para M2: "))
#                 rpm3 = int(input("RPM para M3: "))
#                 rpm4 = int(input("RPM para M4: "))
                
#                 # Enviar el comando
#                 send_rpm_command(ser, rpm1, rpm2, rpm3, rpm4)
                
#                 # Pequeña pausa para no saturar el puerto
#                 time.sleep(0.1)
                
#             except ValueError:
#                 print("Error: Ingresa valores enteros válidos.")
#             except KeyboardInterrupt:
#                 print("\nSaliendo...")
#                 break

#     finally:
#         # Cerrar la conexión serial al salir
#         if ser and ser.is_open:
#             ser.close()
#             print("Conexión serial cerrada.")

# if __name__ == "__main__":
#     main()
