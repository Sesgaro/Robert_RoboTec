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
    espMagic(ESPvalue, motorFactors)
