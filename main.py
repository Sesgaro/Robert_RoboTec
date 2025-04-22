import pygame
import ESP32_S3_Dir as encoders
import ESP32_CAN_Motor as motors
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

# Initial variables for triggers and joystick positions
CAN = motors.initialize_serial('COM6', 115200, 1)
ESPDIR = encoders.initialize_serial('COM7', 115200, 1)

trigger_left = 0
trigger_right = 0
joy1_x = 0  # Initial centered value (0-180 degrees)
joy2_x = 0  # Initial centered value (0-180 degrees)
last_data = ''  # Last control string sent

# Main loop
try:
    while True:
        # Process Pygame events
        pygame.event.pump()

        # Map joystick and trigger values
        new_trigger_left = (motors.map_range(joystick.get_axis(4), -1, 1, 0, 100) // 5) * 5  # Left trigger
        new_trigger_right = (motors.map_range(joystick.get_axis(5), -1, 1, 0, -100) // 5) * 5  # Right trigger
        new_joy1_x = (motors.map_range(joystick.get_axis(0), -1, 1, -90, 90) // 5) * 5       # X-axis of joystick 1
        new_joy2_x = (motors.map_range(joystick.get_axis(2), -1, 1, -90, 90// 5) * 5)        # X-axis of joystick 2

        # Update values only if they have changed
        if (new_trigger_left != trigger_left or new_trigger_right != trigger_right):
            trigger_left = new_trigger_left
            trigger_right = new_trigger_right
            joy1_x = new_joy1_x
            joy2_x = new_joy2_x

        # Call the motor control function
            try:
                if trigger_left > 0 and trigger_right > 0:
                    motors.send_rpm_command(CAN, 0, 0, 0, 0)

                elif trigger_left > 0:
                    motors.send_rpm_command(CAN, trigger_left, trigger_left, trigger_left, trigger_left)

                elif trigger_right > 0:
                    motors.send_rpm_command(CAN, trigger_right, trigger_right, trigger_right, trigger_right)

            except Exception as e:
                print(f"Error controlling motors: {e}")

        if (new_joy1_x != joy1_x or new_joy2_x != joy2_x):
            try:
                encoders.send_grd_command(ESPDIR, joy1_x, joy2_x, joy1_x, joy2_x)
            except Exception as e:
                print(f"Error controlling motors: {e}")
                
        # Small delay to avoid overloading the CPU
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nProgram terminated by user")
    # Stop motors upon exit
    try:
        motors.send_rpm_command(CAN, 0, 0, 0, 0)
        encoders.send_grd_command(ESPDIR, 0, 0, 0, 0)
        print("Motors stopped")

    except Exception as e:
        print(f"Error stopping motors: {e}")

    # Close Pygame
    pygame.quit()
    exit(0)

except Exception as e:
    print(f"Unexpected error in main loop: {e}")
    # Stop motors and close connections upon error exit
    try:
        motors.send_rpm_command(CAN, 0, 0, 0, 0)
        encoders.send_grd_command(ESPDIR, 0, 0, 0, 0)

    except Exception as e:
        print(f"Error during cleanup: {e}")
    pygame.quit()
    exit(1)
