import Pwm_and_control_Test1 as motors
import time
from inputs import get_gamepad

ESPvalue = motors.esp_define('COM8')

motorFactors = {
    "M1": 100,
    "M2": 80,
    "M3": 90,
    "M4": 120,
}

trigger_left = 0
trigger_right = 0
last_data = ''

while True:
    events = get_gamepad()
    for event in events:
        if event.code == "ABS_Z":
            trigger_left = event.state
        
        elif event.code == "ABS_RZ":
            trigger_right = event.state
        
        elif event.code == "ABS_X":
            joy1 = event.state
          
        elif event.code == "ABS_RX":
            joy2 = event.state
    # motors.read_data(ESPvalue)
          
    last_data = motors.esp_magic(ESPvalue, motorFactors, trigger_left, trigger_right, last_data)
