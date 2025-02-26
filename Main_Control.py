import Pwm_and_control_Test1.py as motors
import serial
import time
from inputs import get_gamepad

ESPvalue = espDefine('COM8')

motorFactors = {
    "M1": 100,
    "M2": 80,
    "M3": 90,
    "M4": 120,
}

while True:
    
    for event in events:
        
      if event.code == "ABS_Z":
          trigger_left = event.state
        
      elif event.code == "ABS_RZ":
          trigger_right = event.state
          
      elif event.code == "ABS_X":
          joy1 = event.state
          
      elif event.code == "ABS_RX":
          joy2 = event.state
          
    espMagic(ESPvalue, motorFactors, trigger_left, trigger_right)
