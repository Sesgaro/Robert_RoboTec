'''
// Arduino Code

#define M1_PWM 4    // Right front
#define M1_DIR 5    // Right front
#define M2_PWM 1    // Left front
#define M2_DIR 2    // Left front
#define M3_PWM 7    // Right back
#define M3_DIR 15   // Right back
#define M4_PWM 41   // Left back
#define M4_DIR 40   // Left back

float M1_F, M2_F, M3_F, M4_F;

const int LED = 48;

void setup() {
  Serial.begin(115200);

  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT);
  pinMode(M4_DIR, OUTPUT);

  neopixelWrite(LED,0,0,0);
}

void accelerate(int targetSpeed) {
  static int currentSpeed = 0;
  while (currentSpeed != targetSpeed) {
    currentSpeed += (targetSpeed > currentSpeed) ? 5 : -5;
    
    analogWrite(M1_PWM, currentSpeed * M1_F);
    analogWrite(M2_PWM, currentSpeed * M2_F);
    analogWrite(M3_PWM, currentSpeed * M3_F);
    analogWrite(M4_PWM, currentSpeed * M4_F);
    Serial.println(currentSpeed);
    delay(20);
  }
}

void loop() {
  if (Serial.available()) {
    neopixelWrite(LED,255,128,0);

    String data = Serial.readStringUntil('\n');
    
    float M1_F = data.substring(data.indexOf("M1:") + 3, data.indexOf(",")).toFloat();
    float M2_F = data.substring(data.indexOf("M2:") + 3, data.indexOf(",")).toFloat();
    float M3_F = data.substring(data.indexOf("M3:") + 3, data.indexOf(",")).toFloat();
    float M4_F = data.substring(data.indexOf("M4:") + 3, data.indexOf(",")).toFloat();

    int   Speed = data.substring(data.indexOf("Speed:") + 3, data.indexOf(",")).toInt();
    
    int directionIndex = data.lastIndexOf(",") + 1;
    int directionM = data.substring(directionIndex).toInt();

    int aux = directionM;

    if (directionM != aux && Speed > 0 ) {
      accelerate(0);
    }
    
    digitalWrite(M1_DIR, directionM);
    digitalWrite(M2_DIR, directionM);
    digitalWrite(M3_DIR, directionM);
    digitalWrite(M4_DIR, directionM);

    Serial.println(Speed);
    
    accelerate(Speed);
    
  }
  
}

'''

import serial
import time
from inputs import get_gamepad


# Serial port
esp = serial.Serial('COM10', 115200) # Or whatever your serial port is, in Mac/Linux it's usually /dev/ttyUSB0
time.sleep(2)

motor_factors = {
    "M1": 1.0,
    "M2": 0.8,
    "M3": 0.9,
    "M4": 1.2,
}

trigger_left = 0
trigger_right = 0
last_data = ""  # Temp data

while True:
    events = get_gamepad()
    for event in events:
        if event.code == "ABS_Z":
            trigger_left = event.state
        elif event.code == "ABS_RZ":
            trigger_right = event.state

    if trigger_left > 0 and trigger_right > 0:
        base_speed = 0
        direction = 0

    elif trigger_left > 0:
        base_speed = int(trigger_left) // 2
        direction = 1

    elif trigger_right > 0:
        base_speed = int(trigger_right) // 2
        direction = 0
        
    else:
        base_speed = 0
        direction = 0

    data = f"M1:{motor_factors["M1"]},{direction};M2:{motor_factors["M2"]},{direction};M3:{motor_factors["M3"]},{direction};M4:{motor_factors["M4"]},{direction};Speed:{base_speed}\n"
    # print(f"ESP1: {datos1}")
    if data != last_data:
        esp.write(data.encode())
        time.sleep(0.05)

        last_data = data


