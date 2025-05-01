#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

// Estructura para almacenar los pines de cada motor
struct MotorPins {
  int rpwmPin;
  int lpwmPin;
};

// Lista de pines para los 4 motores
const MotorPins motorPins[] = {
  {12, 13},  // Motor 1: RPWM 12, LPWM 13
  {11, 10},  // Motor 2: RPWM 11, LPWM 10
  {18, 15},  // Motor 3: RPWM 18, LPWM 15
  {7, 1}     // Motor 4: RPWM 7, LPWM 1
};

// Configuración I2C para C3_1 y C3_2
#define SDA_PIN_BUS1  8
#define SCL_PIN_BUS1  9
#define SDA_PIN_BUS2  4
#define SCL_PIN_BUS2  5
#define SLAVE_ADDR_1  0x10  // Bus 1: C3_1 (Motor 1)
#define SLAVE_ADDR_2  0x11  // Bus 2: C3_2 (Motor 2)

// Configuración UART para recibir C3_3 y C3_4
#define UART_TX_PIN  17
#define UART_RX_PIN  16

volatile bool newDataC3_1 = false, newDataC3_2 = false;
volatile uint16_t angleC3_1 = 0, angleC3_2 = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1) {delay(100);}}}
#define EXECUTE(fn) { if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) {error_loop();}}

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscription;
std_msgs__msg__Float32MultiArray angles_msg;
std_msgs__msg__Float32MultiArray positions_msg;
rclc_executor_t executor;
float target_positions[4] = {0.0, 0.0, 0.0, 0.0};

// Clase PIDController
class PIDController {
private:
  int rpwmPin, lpwmPin;
  int rpwmChannel, lpwmChannel;
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
  PIDController(int motorIndex, float kp, float ki, float kd, int rpwmChan, int lpwmChan) 
    : Kp(kp), Ki(ki), Kd(kd), rpwmChannel(rpwmChan), lpwmChannel(lpwmChan) {
    rpwmPin = motorPins[motorIndex].rpwmPin;
    lpwmPin = motorPins[motorIndex].lpwmPin;
    pinMode(rpwmPin, OUTPUT);
    pinMode(lpwmPin, OUTPUT);
    ledcSetup(rpwmChannel, 18000, 8);
    ledcSetup(lpwmChannel, 18000, 8);
    ledcAttachPin(rpwmPin, rpwmChannel);
    ledcAttachPin(lpwmPin, lpwmChannel);
    ledcWrite(rpwmChannel, 0);
    ledcWrite(lpwmChannel, 0);
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
      ledcWrite(rpwmChannel, 0);
      ledcWrite(lpwmChannel, output);
    } else if (output < 0) {
      ledcWrite(rpwmChannel, -output);
      ledcWrite(lpwmChannel, 0);
    } else {
      ledcWrite(rpwmChannel, 0);
      ledcWrite(lpwmChannel, 0);
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
PIDController pid1(0, 14.1, 7.83, 6.35, 0, 1);  // Motor 1
PIDController pid2(1, 16.2, 5.16, 12.73, 2, 3); // Motor 2
PIDController pid3(2, 12.0, 12.0, 3.0, 4, 5);   // Motor 3
PIDController pid4(3, 16.2, 5.16, 12.73, 6, 7); // Motor 4

// Buffer para comandos seriales y UART
String commandBuffer = "";
String uartBuffer = "";
float angle1 = 0.0, angle2 = 0.0, angle3 = 0.0, angle4 = 0.0;

// Micro-ROS callback para recibir posiciones
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size == 4) {
    target_positions[0] = msg->data.data[0]; // Posición Motor 1
    target_positions[1] = msg->data.data[1]; // Posición Motor 2
    target_positions[2] = msg->data.data[2]; // Posición Motor 3
    target_positions[3] = msg->data.data[3]; // Posición Motor 4
    Serial.printf("Recibidas posiciones objetivo: %.1f, %.1f, %.1f, %.1f\n", 
                  target_positions[0], target_positions[1], target_positions[2], target_positions[3]);
  }
}

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

void error_loop() {
  while (1) {
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Iniciando ESP32-S3 con Micro-ROS");

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

  // Inicializar Micro-ROS
  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_s3_node", "", &support));
  
  // Configurar publicador para los 4 ángulos
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "angles"));
  angles_msg.data.capacity = 4;
  angles_msg.data.size = 4;
  angles_msg.data.data = (float*)malloc(4 * sizeof(float));
  
  // Configurar suscripción para las posiciones
  RCCHECK(rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "target_positions"));
  
  // Configurar executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscription, &positions_msg, &subscription_callback, ON_NEW_DATA));

  Serial.println("Micro-ROS inicializado. Conecta el agente en /dev/ttyUSB0.");
}

void loop() {
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

  // Publicar ángulos
  angles_msg.data.data[0] = angle1;
  angles_msg.data.data[1] = angle2;
  angles_msg.data.data[2] = angle3;
  angles_msg.data.data[3] = angle4;
  RCCHECK(rcl_publish(&publisher, &angles_msg, NULL));

  // Actualizar PIDs con los ángulos y posiciones objetivo
  pid1.update(angle1, target_positions[0]);
  pid2.update(angle2, target_positions[1]);
  pid3.update(angle3, target_positions[2]);
  pid4.update(angle4, target_positions[3]);

  // Ejecutar Micro-ROS
  EXECUTE(rclc_executor_spin_some);

  // Depuración
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 1000) {
    Serial.println("--- Estado de los motores ---");
    Serial.print("Motor 1 | Input: ");
    Serial.print(pid1.getInput(), 1);
    Serial.print(" | Angle: ");
    Serial.print(angle1, 1);
    Serial.print(" | AbsAngle: ");
    Serial.print(pid1.getAbsoluteAngle(), 1);
    Serial.print(" | Setpoint: ");
    Serial.print(pid1.getAdjustedSetpoint(), 1);
    Serial.print(" | Output: ");
    Serial.println(pid1.getOutput(), 1);

    Serial.print("Motor 2 | Input: ");
    Serial.print(pid2.getInput(), 1);
    Serial.print(" | Angle: ");
    Serial.print(angle2, 1);
    Serial.print(" | AbsAngle: ");
    Serial.print(pid2.getAbsoluteAngle(), 1);
    Serial.print(" | Setpoint: ");
    Serial.print(pid2.getAdjustedSetpoint(), 1);
    Serial.print(" | Output: ");
    Serial.println(pid2.getOutput(), 1);

    Serial.print("Motor 3 | Input: ");
    Serial.print(pid3.getInput(), 1);
    Serial.print(" | Angle: ");
    Serial.print(angle3, 1);
    Serial.print(" | AbsAngle: ");
    Serial.print(pid3.getAbsoluteAngle(), 1);
    Serial.print(" | Setpoint: ");
    Serial.print(pid3.getAdjustedSetpoint(), 1);
    Serial.print(" | Output: ");
    Serial.println(pid3.getOutput(), 1);

    Serial.print("Motor 4 | Input: ");
    Serial.print(pid4.getInput(), 1);
    Serial.print(" | Angle: ");
    Serial.print(angle4, 1);
    Serial.print(" | AbsAngle: ");
    Serial.print(pid4.getAbsoluteAngle(), 1);
    Serial.print(" | Setpoint: ");
    Serial.print(pid4.getAdjustedSetpoint(), 1);
    Serial.print(" | Output: ");
    Serial.println(pid4.getOutput(), 1);

    lastDebug = millis();
  }
}
