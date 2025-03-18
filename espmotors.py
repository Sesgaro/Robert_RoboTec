import serial
import time
from encoders import ESP32Reader  # Import the ESP32Reader class from ENCODERS.pyz

# Global variables to manage connected boards and latest values
connected_boards = []
latest_values = []  # Shared list to store the latest values from encoders

# Global variable to track previous motor states
prev_DMotors = {
    "M5": {"speed": 0, "direction": 0},
    "M6": {"speed": 0, "direction": 0},
    "M7": {"speed": 0, "direction": 0},
    "M8": {"speed": 0, "direction": 0}
}

def map_range(value, in_min, in_max, out_min, out_max):
    if abs(value) < 0.01:  # Consider values close to 0 as 0
        return 0
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def esp_define(port: str):
    try:
        esp = serial.Serial(port, 115200, timeout=1)  
        time.sleep(2)  # Esperar a que el ESP inicie
        return esp
    except serial.SerialException as e:
        print(f"Error abriendo el puerto serial: {e}")
        return None

def begin_esp(ports):
    global connected_boards, latest_values
    # Initialize the list of latest values with None based on the number of ports (up to 4)
    latest_values = [None] * len(ports)
    
    # Create a list of ESP32Reader objects, assigning indices for the shared list
    boards = [ESP32Reader(port, baudrate=115200, name=f"Board_{i+1}", 
                          latest_values=latest_values, index=i) 
              for i, port in enumerate(ports)]
    
    # Filter only the boards that connected successfully
    connected_boards = [board for board in boards if board.ser is not None]
    if not connected_boards:
        print("No boards connected!")
        return
    
    print(f"Connected boards: {[board.name for board in connected_boards]}")
    # Start reading on each connected board
    for board in connected_boards:
        board.start_reading()

def stop_read():
    global connected_boards
    for board in connected_boards:
        board.stop_reading()
    connected_boards = []

def control_motors(esp, motor_factors, trigger_left, trigger_right, grades_1, grades_2, last_data):
    global prev_DMotors

    if not esp or not esp.is_open:
        print("ESP no está conectado.")
        return last_data
    
    # Determine direction and base speed for the original motors (M1-M4)
    direction = 0
    base_speed = 0

    # Get the latest values from the encoders on the connected ESP32 boards (up to 4)
    encoder1 = 90  # Default value if no data (for M5)
    encoder2 = 90  # Default value if no data (for M6)
    encoder3 = 90  # Default value if no data (for M7)
    encoder4 = 90  # Default value if no data (for M8)
    if len(connected_boards) >= 1:
        last_value1 = connected_boards[0].get_latest_value()
        if last_value1 is not None:
            encoder1 = last_value1
    if len(connected_boards) >= 2:
        last_value2 = connected_boards[1].get_latest_value()
        if last_value2 is not None:
            encoder2 = last_value2
    if len(connected_boards) >= 3:
        last_value3 = connected_boards[2].get_latest_value()
        if last_value3 is not None:
            encoder3 = last_value3
    if len(connected_boards) >= 4:
        last_value4 = connected_boards[3].get_latest_value()
        if last_value4 is not None:
            encoder4 = last_value4

    # Determine base speed and direction for M1-M4 based on triggers
    if trigger_left > 0 and trigger_right > 0:
        base_speed = 0  # If both triggers are pressed, speed 0
    elif trigger_left > 0:
        base_speed = trigger_left
        direction = 0  # Forward
    elif trigger_right > 0:
        base_speed = trigger_right
        direction = 1  # Reverse

    # Build the control string for the traction motors (M1-M4)
    data = (
        f"M1:{motor_factors['M1']},{direction};"
        f"M2:{motor_factors['M2']},{direction};"
        f"M3:{motor_factors['M3']},{direction};"
        f"M4:{motor_factors['M4']},{direction};"
        f"Speed:{base_speed};"
    )

    DMotors = {
        "M5": {"speed": 0, "direction": 0},  # Motor 5
        "M6": {"speed": 0, "direction": 0},  # Motor 6
        "M7": {"speed": 0, "direction": 0},  # Motor 7
        "M8": {"speed": 0, "direction": 0}   # Motor 8
    }

    if grades_1 == 0:
        grades = 90
    
    if grades_2 == 0:
        grades_2 = 90

    # Control M5 based on grades_1 and encoder1
    if abs(encoder1 - grades_1) <= 2:  # Stop if encoder1 is within ±2 degrees of grades_1
        DMotors["M5"] = {"speed": 0, "direction": 0}
    else:
        if grades_1 < encoder1:  # Move to decrease encoder1 towards grades_1
            DMotors["M5"] = {"speed": 50, "direction": 0}  # Assuming direction 0 decreases encoder1
        elif grades_1 > encoder1:  # Move to increase encoder1 towards grades_1
            DMotors["M5"] = {"speed": 50, "direction": 1}  # Assuming direction 1 increases encoder1

    # Control M6 based on grades_1 and encoder2
    if abs(encoder2 - grades_1) <= 2:  # Stop if encoder2 is within ±2 degrees of grades_1
        DMotors["M6"] = {"speed": 0, "direction": 0}
    else:
        if grades_1 < encoder2:  # Move to decrease encoder2 towards grades_1
            DMotors["M6"] = {"speed": 50, "direction": 0}  # Assuming direction 0 decreases encoder2
        elif grades_1 > encoder2:  # Move to increase encoder2 towards grades_1
            DMotors["M6"] = {"speed": 50, "direction": 1}  # Assuming direction 1 increases encoder2

    # Control M7 based on grades_2 and encoder3
    if abs(encoder3 - grades_2) <= 2:  # Stop if encoder3 is within ±2 degrees of grades_2
        DMotors["M7"] = {"speed": 0, "direction": 0}
    else:
        if grades_2 < encoder3:  # Move to decrease encoder3 towards grades_2
            DMotors["M7"] = {"speed": 50, "direction": 0}  # Assuming direction 0 decreases encoder3
        elif grades_2 > encoder3:  # Move to increase encoder3 towards grades_2
            DMotors["M7"] = {"speed": 50, "direction": 1}  # Assuming direction 1 increases encoder3

    # Control M8 based on grades_2 and encoder4
    if abs(encoder4 - grades_2) <= 2:  # Stop if encoder4 is within ±2 degrees of grades_2
        DMotors["M8"] = {"speed": 0, "direction": 0}
    else:
        if grades_2 < encoder4:  # Move to decrease encoder4 towards grades_2
            DMotors["M8"] = {"speed": 50, "direction": 0}  # Assuming direction 0 decreases encoder4
        elif grades_2 > encoder4:  # Move to increase encoder4 towards grades_2
            DMotors["M8"] = {"speed": 50, "direction": 1}  # Assuming direction 1 increases encoder4

    # Add controls for M5-M8 to the data string
    for i in range(5, 9): 
        data += f"M{i}:{DMotors[f'M{i}']['speed']},{DMotors[f'M{i}']['direction']};"

    # Use ETX (\x03, ASCII 3) as the end marker instead of \n
    data += "\x03"

    motor_state_changed = (
        DMotors["M5"] != prev_DMotors["M5"] or
        DMotors["M6"] != prev_DMotors["M6"] or
        DMotors["M7"] != prev_DMotors["M7"] or
        DMotors["M8"] != prev_DMotors["M8"]
    )

    # Send the control string only if it has changed or motor states changed
    if (data != last_data and (base_speed % 5 == 0 and grades_1 % 5 == 0 and grades_2 % 5 == 0)) or motor_state_changed:
        # send_control_string(data)
        esp.write(data.encode())
        last_data = data
        # Update previous motor states
        prev_DMotors["M5"] = DMotors["M5"].copy()
        prev_DMotors["M6"] = DMotors["M6"].copy()
        prev_DMotors["M7"] = DMotors["M7"].copy()
        prev_DMotors["M8"] = DMotors["M8"].copy()

    return last_data

def read_data(esp):
    while True:
        datos1 = esp.readline().decode('utf-8').strip()
        if not datos1:  # Si no hay datos, romper el bucle
            print("No hay más datos. Saliendo...")
            break
        print(f"ESP1: {datos1}")
