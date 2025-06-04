// ESP32S3

#include <Wire.h>
#include <AS5600.h>
#include <EEPROM.h>

#define EN1 7
#define EN2 6

#define ledpin 48

// Estructura para almacenar los pines del motor
struct MotorPins {
  int rpwmPin;
  int lpwmPin;
};

// Pines para dos motores en ESP32 (Placa 1)
const MotorPins motorPins1 = {13, 12}; 
const MotorPins motorPins2 = {10, 11}; 

// Pines para el segundo bus I2C (Wire1)
#define SDA1 5 // SDA para Wire1
#define SCL1 4 // SCL para Wire1

// Configuración de la EEPROM
#define ZERO_OFFSET_ADDR_M1 0  // Dirección en la EEPROM para zeroOffset del Motor 1
#define ZERO_OFFSET_ADDR_M2 4  // Dirección en la EEPROM para zeroOffset del Motor 2 (float usa 4 bytes)

AS5600 as5600_1(&Wire);  // Encoder para Motor 1 (usará Wire)
AS5600 as5600_2(&Wire1); // Encoder para Motor 2 (usará Wire1)

float zeroOffset1 = 0.0, zeroOffset2 = 0.0;
float angle1 = 0.0, angle2 = 0.0;

// Clase PIDController adaptada para ESP32
class PIDController {
private:
  int rpwmPin, lpwmPin;
  static constexpr float MAX_ABSOLUTE_ANGLE = 300.0;
  static constexpr float SAFETY_MARGIN = 70.0;
  static constexpr float ERROR_THRESHOLD = 5.0;
  float Kp, Ki, Kd;
  float setpoint = 0.0, adjustedSetpoint = 0.0, input = 0.0;
  float absoluteAngle = 0.0, lastInput = 0.0, output = 0.0;
  float error = 0.0, lastError = 0.0, integral = 0.0;
  unsigned long lastTime = 0;
  float filteredDerivative = 0.0, alpha = 0.1;
  float filteredDelta = 0.0, deltaAlpha = 0.2;

public:
  PIDController(int rpwm, int lpwm, float kp, float ki, float kd) : rpwmPin(rpwm), lpwmPin(lpwm), Kp(kp), Ki(ki), Kd(kd) {
    pinMode(rpwmPin, OUTPUT);
    pinMode(lpwmPin, OUTPUT);
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
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
    
    if (absError < ERROR_THRESHOLD) {
      integral *= 0.9;
      if (absError < 1.0) integral = 0.0;
    } else if ((unconstrainedOutput == output) || (error * unconstrainedOutput <= 0)) {
      integral += error * dt;
    }
    integral = constrain(integral, -1000, 1000);
    lastError = error;
    
    if (output > 0) {
      analogWrite(rpwmPin, 0);
      analogWrite(lpwmPin, output);
    } else if (output < 0) {
      analogWrite(rpwmPin, -output);
      analogWrite(lpwmPin, 0);
    } else {
      analogWrite(rpwmPin, 0);
      analogWrite(lpwmPin, 0);
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

// Instancias de PID para dos motores
PIDController pid1(motorPins1.rpwmPin, motorPins1.lpwmPin, 16.5, 24.6, 2.75); // Motor 1
PIDController pid2(motorPins2.rpwmPin, motorPins2.lpwmPin, 16.5, 24.6, 2.75); // Motor 2

// Buffer para comandos seriales
String commandBuffer = "";

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17); // RX GPIO16, TX GPIO17
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);

  rgbLedWrite(RGB_BUILTIN, 0, 0, 0);    // Apagar LED

  delay(100);
  Serial.println("Iniciando ESP32 (Placa 1) para control de dos motores con AS5600");

  // Inicializar I2C para ambos buses
  Wire.begin();               // Wire (pines por defecto: SDA 21, SCL 22)
  Wire1.begin(SDA1, SCL1);    // Wire1 (pines personalizados: SDA 18, SCL 19)

  // Leer zeroOffset almacenado en la EEPROM
  EEPROM.get(ZERO_OFFSET_ADDR_M1, zeroOffset1);
  if (isnan(zeroOffset1) || zeroOffset1 < 0.0 || zeroOffset1 >= 360.0) {
    zeroOffset1 = 0.0;
    Serial.println("No se encontró un zeroOffset válido para Motor 1 en EEPROM. Usando 0.0");
  } else {
    Serial.print("zeroOffset Motor 1 cargado desde EEPROM: ");
    Serial.println(zeroOffset1, 1);
  }

  EEPROM.get(ZERO_OFFSET_ADDR_M2, zeroOffset2);
  if (isnan(zeroOffset2) || zeroOffset2 < 0.0 || zeroOffset2 >= 360.0) {
    zeroOffset2 = 0.0;
    Serial.println("No se encontró un zeroOffset válido para Motor 2 en EEPROM. Usando 0.0");
  } else {
    Serial.print("zeroOffset Motor 2 cargado desde EEPROM: ");
    Serial.println(zeroOffset2, 1);
  }

  // Inicializar AS5600 para ambos encoders
  // Verificar conexión del AS5600 Motor 1
    while (!as5600_1.isConnected()) {
      Serial.println("AS5600 Motor 1 no detectado (Wire). Revisa conexiones.");
      rgbLedWrite(RGB_BUILTIN, 255, 0, 0);  // Rojo
      delay(500);
      rgbLedWrite(RGB_BUILTIN, 0, 0, 0);    // Apagar LED
      delay(500);
    }
    Serial.println("AS5600 Motor 1 OK (Wire)");

    // Verificar conexión del AS5600 Motor 2
    while (!as5600_2.isConnected()) {
      Serial.println("AS5600 Motor 2 no detectado (Wire1). Revisa conexiones.");
      rgbLedWrite(RGB_BUILTIN, 255, 0, 0);  // Rojo
      delay(500);
      rgbLedWrite(RGB_BUILTIN, 0, 0, 0);    // Apagar LED
      delay(500);
    }
    Serial.println("AS5600 Motor 2 OK (Wire1)");


  Serial.println("Esperando comandos desde ESP32-S3 o 'zero1'/'zero2' para calibrar los encoders.");
}

void loop() {
  // Leer ángulo de ambos encoders AS5600
  uint16_t raw1 = as5600_1.readAngle();
  float deg1 = raw1 * (360.0 / 4096.0);
  angle1 = deg1 - zeroOffset1;
  if (angle1 < 0) angle1 += 360.0;

  uint16_t raw2 = as5600_2.readAngle();
  float deg2 = raw2 * (360.0 / 4096.0);
  angle2 = deg2 - zeroOffset2;
  if (angle2 < 0) angle2 += 360.0;

  // Procesar comandos desde ESP32-S3
  while (Serial.available()) {
    char c = Serial.read();
    commandBuffer += c;
    if (c == '\n') {
      commandBuffer.trim();
      // Procesar comandos de calibración
      if (commandBuffer == "zero1") {
        zeroOffset1 = as5600_1.readAngle() * (360.0 / 4096.0);
        Serial.println("Offset Motor 1 reiniciado.");
        Serial.print("Nuevo zeroOffset Motor 1: ");
        Serial.println(zeroOffset1, 1);
        EEPROM.put(ZERO_OFFSET_ADDR_M1, zeroOffset1);
        Serial.println("zeroOffset Motor 1 guardado en EEPROM.");
      } else if (commandBuffer == "zero2") {
        zeroOffset2 = as5600_2.readAngle() * (360.0 / 4096.0);
        Serial.println("Offset Motor 2 reiniciado.");
        Serial.print("Nuevo zeroOffset Motor 2: ");
        Serial.println(zeroOffset2, 1);
        EEPROM.put(ZERO_OFFSET_ADDR_M2, zeroOffset2);
        Serial.println("zeroOffset Motor 2 guardado en EEPROM.");
      }
      // Procesar comandos de setpoints
      else if (commandBuffer.startsWith("D1:") && commandBuffer.endsWith(";")) {
        float newSetpoint1 = 0.0, newSetpoint2 = 0.0, newSetpoint3 = 0.0, newSetpoint4 = 0.0;
        bool hasAllFour = false;

        // Parsear los comandos
        int pos1 = commandBuffer.indexOf("D1:");
        int pos2 = commandBuffer.indexOf("D2:");
        int pos3 = commandBuffer.indexOf("D3:");
        int pos4 = commandBuffer.indexOf("D4:");
        int endPos = commandBuffer.length();

        // Verificar si están presentes todos los comandos (D1 a D4)
        if (pos1 != -1 && pos2 != -1 && pos3 != -1 && pos4 != -1) {
          hasAllFour = true;
          newSetpoint1 = commandBuffer.substring(pos1 + 3, commandBuffer.indexOf(';', pos1)).toFloat();
          newSetpoint2 = commandBuffer.substring(pos2 + 3, commandBuffer.indexOf(';', pos2)).toFloat();
          newSetpoint3 = commandBuffer.substring(pos3 + 3, commandBuffer.indexOf(';', pos3)).toFloat();
          newSetpoint4 = commandBuffer.substring(pos4 + 3, commandBuffer.indexOf(';', pos4)).toFloat();
        } else if (pos1 != -1 && pos2 != -1) {
          newSetpoint1 = commandBuffer.substring(pos1 + 3, commandBuffer.indexOf(';', pos1)).toFloat();
          newSetpoint2 = commandBuffer.substring(pos2 + 3, commandBuffer.indexOf(';', pos2)).toFloat();
        } else {
          Serial.println("Formato de comando inválido.");
          commandBuffer = "";
          continue;
        }

        // Actualizar setpoints locales (D1 y D2)
        Serial.print("Setpoints recibidos - Motor 1: ");
        Serial.print(newSetpoint1);
        Serial.print(", Motor 2: ");
        Serial.println(newSetpoint2);
        pid1.update(pid1.getInput(), newSetpoint1);
        pid2.update(pid2.getInput(), newSetpoint2);

        // Si se recibieron D3 y D4, enviarlos a Placa 2
        if (hasAllFour) {
          String commandToSend = "D3:" + String(newSetpoint3) + ";D4:" + String(newSetpoint4) + ";";
          Serial1.println(commandToSend); // Enviar a Placa 2
          Serial.print("Enviado a Placa 2: ");
          Serial.println(commandToSend);
        }
      }
      commandBuffer = "";
    }
  }

  // Actualizar PIDs con los ángulos
  pid1.update(angle1, pid1.getAdjustedSetpoint());
  pid2.update(angle2, pid2.getAdjustedSetpoint());

  // Depuración
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    Serial.println("--- Estado de los motores (Placa 1) ---");
    // Motor 1
    Serial.print("Motor 1 | Input: ");
    Serial.print(pid1.getInput(), 1);
    Serial.print(" | Angle: ");
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
    // Motor 2
    Serial.print("Motor 2 | Input: ");
    Serial.print(pid2.getInput(), 1);
    Serial.print(" | Angle: ");
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

    lastDebug = millis();
  }

  delay(50);
}
