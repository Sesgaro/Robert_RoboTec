'''
#include <Wire.h>

const int I2C_ADDRESS = 0x10; // I2C Direction

struct DirectionMotor {
    int pwmPinForward;
    int pwmPinReverse;
    int speed;
    int directionM;
};

DirectionMotor DMotors[4] = {
    {16, 17, 0, 0}, 
    {21, 22, 0, 0}, 
    {23, 24, 0, 0}  
};

struct WheelMotor {
    int pwmPin;
    int dirPin;
    int factor;
    int directionM;
    int lastDirection;
};

WheelMotor WMotors[4] = {
    {4, 5, 0, 0, -1}, // M1
    {1, 2, 0, 0, -1}, // M2
    {7, 15, 0, 0, -1}, // M3
    {41, 40, 0, 0, -1} // M4
};


int Speed = 0;

void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(WMotors[i].pwmPin, OUTPUT);
    pinMode(WMotors[i].dirPin, OUTPUT);
    pinMode(DMotors[i].pwmPinForward, OUTPUT);
    pinMode(DMotors[i].pwmPinReverse, OUTPUT);
  }

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);

  Serial.begin(115200);
}

void receiveEvent(int howMany) {
    String data = "";
    while (Wire.available()) {
        char c = Wire.read();
        data += c;
    }
    TotalControl(data);
}

void accelerate(int targetSpeed) {
    static int currentSpeed = 0;
    while (currentSpeed != targetSpeed) {
        currentSpeed += (targetSpeed > currentSpeed) ? 5 : -5;
        for (int i = 0; i < 4; i++) {
            int pwmValue = currentSpeed * (WMotors[i].factor / 100.0);
            analogWrite(WMotors[i].pwmPin, pwmValue);
        }
        delay(40);
    }
}

void TotalControl(String data) {
    String parts[13];
    int index = 0;
    size_t start = 0;
    size_t end = data.indexOf(';', start);
    while (end != -1) {
        parts[index++] = data.substring(start, end);
        start = end + 1;
        end = data.indexOf(';', start);
    }
    parts[index] = data.substring(start);

    // Parse M1 to M4
    for (int i = 0; i < 4; i++) {
        String part = parts[i];
        int colonPos = part.indexOf(':');
        int commaPos = part.indexOf(',');
        WMotors[i].factor = part.substring(colonPos+1, commaPos).toInt();
        WMotors[i].directionM = part.substring(commaPos+1).toInt();
    }

    // Parse Speed
    String speedPart = parts[4];
    int speedIdx = speedPart.indexOf(':');
    Speed = speedPart.substring(speedIdx+1).toInt();

    // Parse M5 to M12 
    for (int i = 0; i < 4; i++) { // Solo 4 motores nuevos, pero 8 entradas
        String partForward = parts[5+i];
        String partReverse = parts[9+i]; // Ajustar según duplicación
        int colonPosF = partForward.indexOf(':');
        int commaPosF = partForward.indexOf(',');
        DMotors[i].speed = partForward.substring(colonPosF+1, commaPosF).toInt();
        DMotors[i].directionM = partForward.substring(commaPosF+1).toInt();

        // Set PWM based on direction
        if (DMotors[i].directionM == 0) {
            analogWrite(DMotors[i].pwmPinForward, DMotors[i].speed);
            analogWrite(DMotors[i].pwmPinReverse, 0);
        } else {
            analogWrite(DMotors[i].pwmPinForward, 0);
            analogWrite(DMotors[i].pwmPinReverse, DMotors[i].speed);
        }
    }

    // Handle direction and accelerate for original motors
    bool directionChanged = false;
    for (int i = 0; i < 4; i++) {
        if (WMotors[i].directionM != WMotors[i].lastDirection && Speed > 0) {
            directionChanged = true;
            break;
        }
    }
    if (directionChanged) {
        accelerate(0);
        for (int i = 0; i < 4; i++) {
            WMotors[i].lastDirection = WMotors[i].directionM;
            digitalWrite(WMotors[i].dirPin, WMotors[i].directionM);
        }
    } else {
        for (int i = 0; i < 4; i++) {
            digitalWrite(WMotors[i].dirPin, WMotors[i].directionM);
        }
    }
    accelerate(Speed);
}

void loop(){
  if (lastReceivedData != "") {
    Serial.print("Datos recibidos via I2C: ");
    Serial.println(lastReceivedData);
  }
  delay(10);
}

}
'''
import smbus
import time

I2C_BUS = 1  # Bus I2C 1
I2C_ADDRESS = 0x10  # Direction

bus = smbus.SMBus(I2C_BUS)

def map_range(value, in_min, in_max, out_min, out_max):
    if value == 0:
        return 0 
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


# Function to send data to the ESP32S3 via I2C
def send_control_string(data):
    try:
        # Convert string to a list of bytes (ASCII values)
        byte_data = [ord(c) for c in data]
        # send data to ESP32S3
        bus.write_i2c_block_data(I2C_ADDRESS, 0, byte_data)
        print(f"I2C: {data.strip()}")
    except IOError as e:
        print(f"ERROR I2C: {e}")
    time.sleep(0.01)  

# Main function to control motors
def control_motors(motor_factors, trigger_left, trigger_right, grades_1, grades_2, last_data):
    # Determine direction and base speed for original motors
    direction = 0
    base_speed = 0

    grades_1 = 0

    if trigger_left > 0 and trigger_right > 0:
        base_speed = 0  
    elif trigger_left > 0:
        base_speed = trigger_left
        direction = 0  
    elif trigger_right > 0:
        base_speed = trigger_right
        direction = 1  

    # Building the control chain for the 4 wheel motors
    data = (
        f"M1:{motor_factors['M1']},{direction};"
        f"M2:{motor_factors['M2']},{direction};"
        f"M3:{motor_factors['M3']},{direction};"
        f"M4:{motor_factors['M4']},{direction};"
        f"Speed:{base_speed};"
    )

    # Add control for the 4 new motors, which are steering motors.
    DMotors = {
        "M5": {"speed": 0, "direction": 0},  # Motor 5
        "M6": {"speed": 0, "direction": 0},  # Motor 6
        "M7": {"speed": 0, "direction": 0},  # Motor 7
        "M8": {"speed": 0, "direction": 0}   # Motor 8
    }

    # Control M5 and M6 based on joy_1
    if grades_1 < 90:
        DMotors["M5"] = {"speed": 50, "direction": 0}  # Right (forward)
        DMotors["M6"] = {"speed": 50, "direction": 0}  # Right (forward)
    elif grades_1 > 90:
        DMotors["M5"] = {"speed": 50, "direction": 1}  # Left (reverse)
        DMotors["M6"] = {"speed": 50, "direction": 1}  # Left (reverse)
    else:
        DMotors["M5"] = {"speed": 0, "direction": 0}
        DMotors["M6"] = {"speed": 0, "direction": 0}

    # Control M7 and M8 based on joy_2
    if grades_2 < 90:
        DMotors["M7"] = {"speed": 50, "direction": 0}  # Right (forward)
        DMotors["M8"] = {"speed": 50, "direction": 0}  # Right (forward)
    elif grades_2 > 90:
        DMotors["M7"] = {"speed": 50, "direction": 1}  # Left (reverse)
        DMotors["M8"] = {"speed": 50, "direction": 1}  # Left (reverse)
    else:
        DMotors["M7"] = {"speed": 0, "direction": 0}
        DMotors["M8"] = {"speed": 0, "direction": 0}
    

    # Append new motors to the control string
    for i in range(5, 9):  # M5 to M8
        data += f"M{i}:{DMotors[f'M{i}']['speed']},{DMotors[f'M{i}']['direction']};"


    data += "\n"  # End the string with a line break

    if data != last_data and base_speed % 5 == 0 and grades_2 % 5 == 0 and grades_1 % 5 == 0:
        send_control_string(data)
        last_data = data

    return last_data
