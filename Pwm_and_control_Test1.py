'''
#define M1_PWM 4    // Right front
#define M1_DIR 5    // Right front
#define M2_PWM 1    // Left front
#define M2_DIR 2    // Left front
#define M3_PWM 7    // Right back
#define M3_DIR 15   // Right back
#define M4_PWM 41   // Left back
#define M4_DIR 40   // Left back


int M1_DI, M2_DI, M3_DI, M4_DI;

int M1_F = 100, M2_F = 80, M3_F = 90, M4_F = 120;

int Speed;

int lastDirection1, lastDirection2, lastDirection3, lastDirection4 = -1;

#define LED 48

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

  neopixelWrite(LED, 0, 0, 0);
}

void accelerate(int targetSpeed) {
  static int currentSpeed = 0;
  while (currentSpeed != targetSpeed) {
    currentSpeed += (targetSpeed > currentSpeed) ? 5 : -5;
    
    analogWrite(M1_PWM, currentSpeed * (M1_F / 100.0));
    analogWrite(M2_PWM, currentSpeed * (M2_F / 100.0));
    analogWrite(M3_PWM, currentSpeed * (M3_F / 100.0));
    analogWrite(M4_PWM, currentSpeed * (M4_F / 100.0));

    Serial.print("Velocidad actual: ");
    Serial.println(currentSpeed);
    delay(40);
  }
}

void loop() {
  if (Serial.available()) {
    neopixelWrite(LED, 255, 128, 0);   // Lights LED indicating activity 

    String data = Serial.readStringUntil('\n');

    int m1_idx = data.indexOf("M1:") + 3;
    int m2_idx = data.indexOf("M2:") + 3;
    int m3_idx = data.indexOf("M3:") + 3;
    int m4_idx = data.indexOf("M4:") + 3;
    int speed_idx = data.indexOf("Speed:") + 6;

    M1_F = data.substring(m1_idx, data.indexOf(",", m1_idx)).toInt();
    M1_DI = data.substring(data.indexOf(",", m1_idx) + 1, data.indexOf(";", m1_idx)).toInt();

    M2_F = data.substring(m2_idx, data.indexOf(",", m2_idx)).toInt();
    M2_DI = data.substring(data.indexOf(",", m2_idx) + 1, data.indexOf(";", m2_idx)).toInt();

    M3_F = data.substring(m3_idx, data.indexOf(",", m3_idx)).toInt();
    M3_DI = data.substring(data.indexOf(",", m3_idx) + 1, data.indexOf(";", m3_idx)).toInt();

    M4_F = data.substring(m4_idx, data.indexOf(",", m4_idx)).toInt();
    M4_DI = data.substring(data.indexOf(",", m4_idx) + 1, data.indexOf(";", m4_idx)).toInt();

    Speed = data.substring(speed_idx).toInt();

   


 // Direction

    // If the direction changes, stop motors before switching 
    if ((M1_DI != lastDirection1 && Speed > 0) || (M2_DI != lastDirection2 && Speed > 0) || (M3_DI != lastDirection3 && Speed > 0) || (M4_DI != lastDirection4 && Speed > 0)) {
      accelerate(0);
      lastDirection1 = M1_DI;
      lastDirection2 = M2_DI;
      lastDirection3 = M3_DI;
      lastDirection4 = M4_DI;
    }
 // Set address individually 
    digitalWrite(M1_DIR, M1_DI);
    digitalWrite(M2_DIR, M2_DI);
    digitalWrite(M3_DIR, M3_DI);
    digitalWrite(M4_DIR, M4_DI);

    accelerate(Speed);

  }
}
'''
import serial
import time

trigger_left = 0
trigger_right = 0
last_data = ""  # Last data sent to avoid redundancies

def map_range(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def espDefine(port):
  esp = serial.Serial(port, 115200, timeout=1)  # Or whatever your serial port is, in Mac/Linux it's usually /dev/ttyUSB0
  time.sleep(2)  # Wait for ESP to start
  return esp

def espMagic(esp, motor_factors, trigger_left, trigger_right):
  # events = get_gamepad()
  # for event in events:
  #     if event.code == "ABS_Z":
  #         trigger_left = event.state
        
  #     elif event.code == "ABS_RZ":
  #         trigger_right = event.state

  if trigger_left > 0 and trigger_right > 0:
      base_speed = 0
    
  elif trigger_left > 0:
      base_speed = map_range(trigger_left, 0, 255, 0, 100)
      direction = 1  
    
  elif trigger_right > 0:
      base_speed = map_range(trigger_right, 0, 255, 0, 100)
      direction = 0  
  else:
      base_speed = 0


  data = f"M1:{motor_factors['M1']},{direction};M2:{motor_factors['M2']},{direction};"
  data += f"M3:{motor_factors['M3']},{direction};M4:{motor_factors['M4']},{direction};Speed:{base_speed}\n"

  """
  datos1 = esp.readline().decode('utf-8').strip()
  print(f"ESP1: {datos1}")
  """

  # Send data only if there are changes
  if data != last_data and base_speed % 5 == 0:
        esp.write(data.encode())
        last_data = data

  time.sleep(0.02)
