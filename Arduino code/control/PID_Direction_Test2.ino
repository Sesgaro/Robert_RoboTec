#include <Wire.h>
#include <HardwareSerial.h>

// Estructura para almacenar los pines de cada motor
struct MotorPins {
  int rpwmPin;
  int lpwmPin;
};

// Lista de pines para los 4 motores
const MotorPins motorPins[] = {
  {13, 12},  // Motor 1: RPWM 13, LPWM 12
  {21, 22},  // Motor 2: RPWM 21, LPWM 22
  {18, 19},  // Motor 3: RPWM 18, LPWM 19
  {10, 11}   // Motor 4: RPWM 10, LPWM 11
};

// Configuración I2C para C3_1 y C3_2
#define SDA_PIN_BUS1  8
#define SCL_PIN_BUS1  9
#define SDA_PIN_BUS2  4
#define SCL_PIN_BUS2  5
#define SLAVE_ADDR_1  0x10  // Bus 1: C3_1 (Motor 1)
#define SLAVE_ADDR_2  0x11  // Bus 2: C3_2 (Motor 2)

// Configuración UART para recibir C3_3 y C3_4
#define UART_TX_PIN  20
#define UART_RX_PIN  19

volatile bool newDataC3_1 = false, newDataC3_2 = false;
volatile uint16_t angleC3_1 = 0, angleC3_2 = 0;

// Clase PIDController
class PIDController {
private:
  int rpwmPin, lpwmPin;
  static constexpr float MAX_ABSOLUTE_ANGLE = 300.0;
  static constexpr float SAFETY_MARGIN = 70.0;
  float Kp, Ki, Kd;
  float setpoint = 0.0, adjustedSetpoint = 0.0, input = 0.0;
  float absoluteAngle = 0.0, lastInput = 0.0, output = 0.0;
  float error = 0.0, lastError = 0.0, integral = 0.0;
  unsigned long lastTime = 0;
  float filteredDerivative = 0.0, alpha = 0.1;
  float filteredDelta = 0.0, deltaAlpha = 0.2;

public:
  PIDController(int motorIndex, float kp, float ki, float kd) 
    : Kp(kp), Ki(ki), Kd(kd) {
    rpwmPin = motorPins[motorIndex].rpwmPin;
    lpwmPin = motorPins[motorIndex].lpwmPin;
    pinMode(rpwmPin, OUTPUT);
    pinMode(lpwmPin, OUTPUT);
    ledcAttach(rpwmPin, 20000, 8);
    ledcAttach(lpwmPin, 20000, 8);
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, 0);
  }

  float normalizeAngle(float angle) {
    while (angle >= 360.0) angle -= 360.0;
    while (angle < 0.0) angle += 360.0;
    return angle;
  }

  float calculateAdjustedSetpoint(float target, float current, float absAngle) {
    target = normalizeAngle(target);
    current = normalizeAngle(current);
    if (absAngle > (MAX_ABSOLUTE_ANGLE - SAFETY_MARGIN)) {
      float safeAngle = current - 180.0;
      return normalizeAngle(safeAngle);
    } else if (absAngle < -(MAX_ABSOLUTE_ANGLE - SAFETY_MARGIN)) {
      float safeAngle = current + 180.0;
      return normalizeAngle(safeAngle);
    }
    return target;
  }

  float calculateError(float target, float current) {
    float error = target - current;
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    return error;
  }

  void update(float newInput, float newSetpoint) {
    setpoint = normalizeAngle(newSetpoint);
    input = normalizeAngle(newInput);
    float delta = input - lastInput;
    if (delta > 180.0) delta -= 360.0;
    if (delta < -180.0) delta += 360.0;
    filteredDelta = deltaAlpha * delta + (1 - deltaAlpha) * filteredDelta;
    absoluteAngle += filteredDelta;
    lastInput = input;
    adjustedSetpoint = calculateAdjustedSetpoint(setpoint, input, absoluteAngle);
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt == 0) dt = 0.1;
    lastTime = now;
    error = calculateError(adjustedSetpoint, input);
    float absError = abs(error);
    alpha = map(absError, 0, 180, 0.05, 0.5);
    alpha = constrain(alpha, 0.05, 0.5);
    float derivative = (error - lastError) / dt;
    filteredDerivative = alpha * derivative + (1 - alpha) * filteredDerivative;
    float unconstrainedOutput = Kp * error + Ki * integral + Kd * filteredDerivative;
    output = constrain(unconstrainedOutput, -255, 255);
    if ((unconstrainedOutput == output) || (error * unconstrainedOutput <= 0)) {
      integral += error * dt;
    }
    integral = constrain(integral, -1000, 1000);
    lastError = error;
    if (output > 0) {
      ledcWrite(rpwmPin, 0);
      ledcWrite(lpwmPin, output);
    } else if (output < 0) {
      ledcWrite(rpwmPin, -output);
      ledcWrite(lpwmPin, 0);
    } else {
      ledcWrite(rpwmPin, 0);
      ledcWrite(lpwmPin, 0);
    }
  }

  float getInput() { return input; }
  float getAbsoluteAngle() { return absoluteAngle; }
  float getAdjustedSetpoint() { return adjustedSetpoint; }
  float getError() { return error; }
  float getAlpha() { return alpha; }
  float getOutput() { return output; }
  float getIntegral() { return integral; }
};

// Instancias de PID para cuatro motores
PIDController pid1(0, 10.0, 10.0, 2.5);  // Motor 1
PIDController pid2(1, 8.0, 8.0, 2.0);    // Motor 2
PIDController pid3(2, 12.0, 12.0, 3.0);  // Motor 3
PIDController pid4(3, 9.0, 9.0, 2.2);    // Motor 4

// Buffer para comandos seriales y UART
String commandBuffer = "";
String uartBuffer = "";
float angle1 = 0.0, angle2 = 0.0, angle3 = 0.0, angle4 = 0.0;

// Funciones I2C para C3_1 y C3_2
void onReceiveBus1(int howMany) {
  if (howMany >= 3) {
    uint8_t id = Wire.read();
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    if (id == 0x01) {  // C3_1 (Motor 1)
      angleC3_1 = (hi << 8) | lo;
      newDataC3_1 = true;
    }
  }
}

void onReceiveBus2(int howMany) {
  if (howMany >= 3) {
    uint8_t id = Wire1.read();
    uint8_t lo = Wire1.read();
    uint8_t hi = Wire1.read();
    if (id == 0x02) {  // C3_2 (Motor 2)
      angleC3_2 = (hi << 8) | lo;
      newDataC3_2 = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Iniciando ESP32-S3 para control de motores (I2C para C3_1 y C3_2, UART para C3_3 y C3_4)");

  // Configurar I2C
  pinMode(SDA_PIN_BUS1, INPUT_PULLUP);
  pinMode(SCL_PIN_BUS1, INPUT_PULLUP);
  pinMode(SDA_PIN_BUS2, INPUT_PULLUP);
  pinMode(SCL_PIN_BUS2, INPUT_PULLUP);

  Wire.setPins(SDA_PIN_BUS1, SCL_PIN_BUS1);
  Wire.begin(SLAVE_ADDR_1);
  Wire.onReceive(onReceiveBus1);

  Wire1.setPins(SDA_PIN_BUS2, SCL_PIN_BUS2);
  Wire1.begin(SLAVE_ADDR_2);
  Wire1.onReceive(onReceiveBus2);

  // Configurar UART
  Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.printf("I2C esclavo listo: Bus 1 en 0x%02X (C3_1), Bus 2 en 0x%02X (C3_2)\n", SLAVE_ADDR_1, SLAVE_ADDR_2);
  Serial.println("Introduce comandos como 'D1:ang1;' para Motor 1, 'D2:ang2;' para Motor 2, etc.");
}

void loop() {
  // Procesar comandos seriales (setpoints)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      commandBuffer.trim();
      if (commandBuffer.endsWith(";")) {
        float newSetpoint = 0.0;
        int colonPos = commandBuffer.indexOf(':');
        if (colonPos != -1) {
          String id = commandBuffer.substring(0, colonPos);
          newSetpoint = commandBuffer.substring(colonPos + 1, commandBuffer.length() - 1).toFloat();
          if (id == "D1") {
            Serial.print("Setpoint Motor 1: ");
            Serial.println(newSetpoint);
            pid1.update(pid1.getInput(), newSetpoint);
          } else if (id == "D2") {
            Serial.print("Setpoint Motor 2: ");
            Serial.println(newSetpoint);
            pid2.update(pid2.getInput(), newSetpoint);
          } else if (id == "D3") {
            Serial.print("Setpoint Motor 3: ");
            Serial.println(newSetpoint);
            pid3.update(pid3.getInput(), newSetpoint);
          } else if (id == "D4") {
            Serial.print("Setpoint Motor 4: ");
            Serial.println(newSetpoint);
            pid4.update(pid4.getInput(), newSetpoint);
          }
        }
      }
      commandBuffer = "";
    } else {
      commandBuffer += c;
    }
  }

  // Actualizar ángulos desde I2C (C3_1 y C3_2)
  if (newDataC3_1) {
    angle1 = angleC3_1 / 10.0f;
    Serial.printf("Ángulo C3_1 (Motor 1): %.1f°\n", angle1);
    newDataC3_1 = false;
  }
  if (newDataC3_2) {
    angle2 = angleC3_2 / 10.0f;
    Serial.printf("Ángulo C3_2 (Motor 2): %.1f°\n", angle2);
    newDataC3_2 = false;
  }

  // Leer datos de UART (C3_3 y C3_4)
  while (Serial1.available()) {
    char c = Serial1.read();
    uartBuffer += c;
    if (c == ';') {
      // Parsear mensaje UART: A3:angle3:A4:angle4;
      int pos3 = uartBuffer.indexOf("A3:");
      int pos4 = uartBuffer.indexOf("A4:");
      int endPos = uartBuffer.indexOf(';');
      if (pos3 != -1 && pos4 != -1 && endPos != -1) {
        angle3 = uartBuffer.substring(pos3 + 3, pos4).toFloat();
        angle4 = uartBuffer.substring(pos4 + 3, endPos).toFloat();
        Serial.printf("Ángulos recibidos por UART - Motor 3: %.1f°, Motor 4: %.1f°\n", angle3, angle4);
      }
      uartBuffer = "";
    }
  }

  // Actualizar PIDs con los ángulos
  pid1.update(angle1, pid1.getAdjustedSetpoint());
  pid2.update(angle2, pid2.getAdjustedSetpoint());
  pid3.update(angle3, pid3.getAdjustedSetpoint());
  pid4.update(angle4, pid4.getAdjustedSetpoint());

  // Depuración
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    Serial.println("--- Estado de los motores ---");
    Serial.print("Motor 1");
    Serial.print(" | Input: ");
    Serial.print(angle1, 1);
    Serial.print(" | AbsAngle: ");
    Serial.print(pid1.getAbsoluteAngle(), 1);
    Serial.print(" | Setpoint: ");
    Serial.print(pid1.getAdjustedSetpoint(), 1);
    Serial.print(" | Error: ");
    Serial.print(pid1.getError(), 1);
    Serial.print(" | Alpha: ");
    Serial.print(pid1.getAlpha(), 4);
    Serial.print(" | Output: ");
    Serial.print(pid1.getOutput(), 1);
    Serial.print(" | Integral: ");
    Serial.println(pid1.getIntegral(), 1);

    Serial.print("Motor 2");
    Serial.print(" | Input: ");
    Serial.print(angle2, 1);
    Serial.print(" | AbsAngle: ");
    Serial.print(pid2.getAbsoluteAngle(), 1);
    Serial.print(" | Setpoint: ");
    Serial.print(pid2.getAdjustedSetpoint(), 1);
    Serial.print(" | Error: ");
    Serial.print(pid2.getError(), 1);
    Serial.print(" | Alpha: ");
    Serial.print(pid2.getAlpha(), 4);
    Serial.print(" | Output: ");
    Serial.print(pid2.getOutput(), 1);
    Serial.print(" | Integral: ");
    Serial.println(pid2.getIntegral(), 1);

    Serial.print("Motor 3");
    Serial.print(" | Input: ");
    Serial.print(angle3, 1);
    Serial.print(" | AbsAngle: ");
    Serial.print(pid3.getAbsoluteAngle(), 1);
    Serial.print(" | Setpoint: ");
    Serial.print(pid3.getAdjustedSetpoint(), 1);
    Serial.print(" | Error: ");
    Serial.print(pid3.getError(), 1);
    Serial.print(" | Alpha: ");
    Serial.print(pid3.getAlpha(), 4);
    Serial.print(" | Output: ");
    Serial.print(pid3.getOutput(), 1);
    Serial.print(" | Integral: ");
    Serial.println(pid3.getIntegral(), 1);

    Serial.print("Motor 4");
    Serial.print(" | Input: ");
    Serial.print(angle4, 1);
    Serial.print(" | AbsAngle: ");
    Serial.print(pid4.getAbsoluteAngle(), 1);
    Serial.print(" | Setpoint: ");
    Serial.print(pid4.getAdjustedSetpoint(), 1);
    Serial.print(" | Error: ");
    Serial.print(pid4.getError(), 1);
    Serial.print(" | Alpha: ");
    Serial.print(pid4.getAlpha(), 4);
    Serial.print(" | Output: ");
    Serial.print(pid4.getOutput(), 1);
    Serial.print(" | Integral: ");
    Serial.println(pid4.getIntegral(), 1);

    lastDebug = millis();
  }
}
