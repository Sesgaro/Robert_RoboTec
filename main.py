
import pygame
import espmotors as motors
import time

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise Exception("No se encontró un control de Xbox")

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Detectado: {joystick.get_name()}")

ESPvalue = motors.esp_define('/dev/ttyACM0')

motorFactors = {
    "M1": 100,
    "M2": 80,
    "M3": 90,
    "M4": 120,
}

trigger_left = 0
trigger_right = 0
joy1_x = 90
joy2_x = 90
last_data = ''

while True:
    pygame.event.pump()

    new_trigger_left = motors.map_range(joystick.get_axis(4), -1, 1, 0, 100)
    new_trigger_right = motors.map_range(joystick.get_axis(5), -1, 1, 0, 100)
    new_joy1_x = motors.map_range(joystick.get_axis(0), -1, 1, 0, 180)
    new_joy2_x = motors.map_range(joystick.get_axis(2), -1, 1, 0, 180)

    new_trigger_left = (new_trigger_left // 5) * 5
    new_trigger_right = (new_trigger_right // 5) * 5
    new_joy1_x = (new_joy1_x // 5) * 5
    new_joy2_x = (new_joy2_x // 5) * 5

    if (new_trigger_left != trigger_left or new_trigger_right != trigger_right or
        new_joy1_x != joy1_x or new_joy2_x != joy2_x):
        trigger_left = new_trigger_left
        trigger_right = new_trigger_right
        joy1_x = new_joy1_x
        joy2_x = new_joy2_x

        last_data = motors.esp_magic(ESPvalue, motorFactors, trigger_left, trigger_right, last_data)
        print(f"Trigger Izq: {trigger_left} | Trigger Der: {trigger_right} | Joy1 X: {joy1_x} | Joy2 X: {joy2_x}")
