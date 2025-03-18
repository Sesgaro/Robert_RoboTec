import pygame
import espmotors as motors
import time

# Initialize Pygame and the joystick module
try:
    pygame.init()
    pygame.joystick.init()
except Exception as e:
    print(f"Error initializing Pygame: {e}")
    exit(1)

# Check if a joystick/controller is connected
if pygame.joystick.get_count() == 0:
    print("No Xbox controller or joystick found")
    pygame.quit()
    exit(1)

# Initialize the joystick
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected: {joystick.get_name()}")
except Exception as e:
    print(f"Error initializing joystick: {e}")
    pygame.quit()
    exit(1)

# Predefined factors for motors M1-M4 (traction)
motor_factors = {
    "M1": 100,
    "M2": 80,
    "M3": 90,
    "M4": 120,
}

# Initial variables for triggers and joystick positions
ESPvalue = motors.esp_define('/dev/ttyS0')

trigger_left = 0
trigger_right = 0
joy1_x = 90  # Initial centered value (0-180 degrees)
joy2_x = 90  # Initial centered value (0-180 degrees)
last_data = ''  # Last control string sent

# List of serial ports for ESP32 boards
esps = ['/dev/ttyACM0']  # Adjust according to the actual ports in your system

# Initialize ESP32 boards to read encoders
try:
    motors.begin_esp(esps)
except Exception as e:
    print(f"Error initializing ESP32 boards: {e}")
    pygame.quit()
    exit(1)

# Main loop
try:
    while True:
        # Process Pygame events
        pygame.event.pump()

        # Map joystick and trigger values
        new_trigger_left = motors.map_range(joystick.get_axis(4), -1, 1, 0, 100)  # Left trigger
        new_trigger_right = motors.map_range(joystick.get_axis(5), -1, 1, 0, 100)  # Right trigger
        new_joy1_x = motors.map_range(joystick.get_axis(0), -1, 1, 10, 170)        # X-axis of joystick 1
        new_joy2_x = motors.map_range(joystick.get_axis(2), -1, 1, 10, 170)        # X-axis of joystick 2

        # Round to multiples of 5 to reduce update frequency
        new_trigger_left = (new_trigger_left // 5) * 5
        new_trigger_right = (new_trigger_right // 5) * 5
        new_joy1_x = (new_joy1_x // 5) * 5
        new_joy2_x = (new_joy2_x // 5) * 5

        # Update values only if they have changed
        if (new_trigger_left != trigger_left or new_trigger_right != trigger_right or new_joy1_x != joy1_x or new_joy2_x != joy2_x):
            trigger_left = new_trigger_left
            trigger_right = new_trigger_right
            joy1_x = new_joy1_x
            joy2_x = new_joy2_x

        # Call the motor control function
        try:
            last_data = motors.control_motors(
                ESPvalue, motor_factors, trigger_left, trigger_right, joy1_x, joy2_x, last_data
            )
            
        except Exception as e:
            print(f"Error controlling motors: {e}")

        # Small delay to avoid overloading the CPU
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nProgram terminated by user")
    # Stop motors upon exit
    try:
        last_data = motors.control_motors(ESPvalue, motor_factors, 0, 0, 90, 90, last_data)
        print("Motors stopped")
    except Exception as e:
        print(f"Error stopping motors: {e}")
    # Stop reading from ESP32 boards
    try:
        motors.stop_read()
        print("ESP32 reading stopped")
    except Exception as e:
        print(f"Error stopping ESP32 reading: {e}")
    # Close Pygame
    pygame.quit()
    exit(0)

except Exception as e:
    print(f"Unexpected error in main loop: {e}")
    # Stop motors and close connections upon error exit
    try:
        last_data = motors.control_motors(ESPvalue, motor_factors, 0, 0, 90, 90, last_data)
        motors.stop_read()
    except Exception as e:
        print(f"Error during cleanup: {e}")
    pygame.quit()
    exit(1)
