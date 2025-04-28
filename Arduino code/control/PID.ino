#include <HardwareSerial.h>

// Pines del BST7960 (ajustados para ESP32-S3)
#define RPWM_PIN  13  // PWM para rotación "derecha"
#define LPWM_PIN  12  // PWM para rotación "izquierda"

// Límite de ángulo absoluto para evitar enredos
#define MAX_ABSOLUTE_ANGLE 300.0  // Máximo ángulo acumulado permitido (±350° desde el inicio)
#define SAFETY_MARGIN 50.0        // Margen de seguridad antes de alcanzar el límite

// Variables PID
float Kp = 2.0;    // Ganancia proporcional (reducido)
float Ki = 0.05;  // Ganancia integral (reducido aún más)
float Kd = 0.1;    // Ganancia derivativa (reducido)
float setpoint = 0.0; // Ángulo deseado (0-360)
float adjustedSetpoint = 0.0; // Setpoint ajustado para desenrollar
float input = 0.0;    // Ángulo actual (0-360)
float absoluteAngle = 0.0; // Ángulo absoluto (rastreando vueltas)
float lastInput = 0.0; // Último ángulo leído para calcular deltas
float output = 0.0;   // Salida PID (-255 a 255)
float error = 0.0, lastError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;

// Variables para el filtro pasa-bajos del término derivativo
float filteredDerivative = 0.0; // Derivativo filtrado
float alpha = 0.1; // Factor de suavizado (ajusta según necesidad)

// Filtro para el cálculo de delta
float filteredDelta = 0.0;
float deltaAlpha = 0.2; // Factor de suavizado para delta

// Buffer para recepción del setpoint
String setpointBuffer = "";

// Normalizar un ángulo al rango [0, 360)
float normalizeAngle(float angle) {
  while (angle >= 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  return angle;
}

// Calcular el setpoint ajustado para evitar acumular vueltas excesivas
float calculateAdjustedSetpoint(float target, float current, float absAngle) {
  target = normalizeAngle(target);
  current = normalizeAngle(current);

  // Si estamos cerca del límite, ajustamos el setpoint para "desenrollar"
  if (absAngle > (MAX_ABSOLUTE_ANGLE - SAFETY_MARGIN)) {
    // Estamos demasiado "enrollados" en sentido positivo
    // Ajustamos el setpoint para que el motor gire en sentido negativo
    float safeAngle = current - 180.0; // Girar 180° en sentido negativo
    return normalizeAngle(safeAngle);
  } else if (absAngle < -(MAX_ABSOLUTE_ANGLE - SAFETY_MARGIN)) {
    // Estamos demasiado "enrollados" en sentido negativo
    // Ajustamos el setpoint para que el motor gire en sentido positivo
    float safeAngle = current + 180.0; // Girar 180° en sentido positivo
    return normalizeAngle(safeAngle);
  }

  // Si no estamos cerca del límite, usamos el setpoint original
  return target;
}

// Calcular el error angular
float calculateError(float target, float current) {
  float error = target - current;
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;
  return error;
}

void setup() {
  Serial.begin(115200); // Para depuración
  Serial.println("Iniciando setup...");

  // Inicializar UART1
  Serial1.begin(115200, SERIAL_8N1, 16, 17); // UART1: RX=GPIO16, TX=GPIO17
  Serial.println("UART1 inicializado");

  // Configurar pines del BST7960
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  Serial.println("Pines configurados");

  // Configurar PWM para ambos pines (ESP32 usa ledc)
  ledcAttach(RPWM_PIN, 20000, 8); // Asignar PWM a RPWM
  ledcAttach(LPWM_PIN, 20000, 8); // Asignar PWM a LPWM
  Serial.println("PWM configurado");

  // Inicializar en apagado
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);
  Serial.println("Motor apagado");

  Serial.println("Introduce el ángulo deseado (0-360):");
  Serial.println("Setup completado");
}

void loop() {
  // Recibir ángulo actual por UART1
  if (Serial1.available()) {
    input = Serial1.parseFloat();
    while (Serial1.available()) Serial1.read(); // Limpiar buffer

    // Calcular el cambio en el ángulo para actualizar el ángulo absoluto
    float delta = input - lastInput;
    if (delta > 180.0) delta -= 360.0;  // Cruce de 0° a 360°
    if (delta < -180.0) delta += 360.0; // Cruce de 360° a 0°
    
    // Filtrar el delta para reducir el impacto del ruido
    filteredDelta = deltaAlpha * delta + (1 - deltaAlpha) * filteredDelta;
    absoluteAngle += filteredDelta;
    lastInput = input;

    // Normalizar el input al rango [0, 360)
    input = normalizeAngle(input);
  }

  // Recibir setpoint por Serial (USB)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      float newSetpoint = setpointBuffer.toFloat();
      // Normalizar el setpoint al rango [0, 360)
      newSetpoint = normalizeAngle(newSetpoint);
      setpoint = newSetpoint;
      integral = 0; // Reiniciar el término integral al cambiar el setpoint
      Serial.print("Setpoint: ");
      Serial.println(setpoint);
      setpointBuffer = ""; // Limpiar el buffer
    } else {
      setpointBuffer += c; // Acumular caracteres
    }
  }

  // Calcular el setpoint ajustado para evitar acumular vueltas excesivas
  adjustedSetpoint = calculateAdjustedSetpoint(setpoint, input, absoluteAngle);

  // Calcular PID
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // Tiempo en segundos
  if (dt == 0) dt = 0.1; // Evitar división por cero
  lastTime = now;

  // Calcular el error
  error = calculateError(adjustedSetpoint, input);
  float absError = abs(error);

  // Calcular alpha dinámico
  alpha = map(absError, 0, 180, 0.05, 0.5);
  alpha = constrain(alpha, 0.05, 0.5);
  
  // Calcular el término derivativo crudo
  float derivative = (error - lastError) / dt;

  // Filtro de 1° orden
  filteredDerivative = alpha * derivative + (1 - alpha) * filteredDerivative;

  // Calcular salida sin limitar
  float unconstrainedOutput = Kp * error + Ki * integral + Kd * filteredDerivative;

  // Limitar la salida
  output = constrain(unconstrainedOutput, -255, 255);

  // Antiwindup
  if ((unconstrainedOutput == output) || (error * unconstrainedOutput <= 0)) {
    integral += error * dt; 
  }
  integral = constrain(integral, -1000, 1000); // Limitar el término integral

  lastError = error; // Actualizar lastError con el error normalizado

  // Controlar motor con BST7960 (direcciones ajustadas)
  if (output > 0) {
    // Rotación "izquierda" (disminuir ángulo)
    ledcWrite(RPWM_PIN, 0);       // RPWM apagado
    ledcWrite(LPWM_PIN, output);  // LPWM
  } else if (output < 0) {
    // Rotación "derecha" (aumentar ángulo)
    ledcWrite(RPWM_PIN, -output); // RPWM
    ledcWrite(LPWM_PIN, 0);       // LPWM apagado
  } else {
    // Apagar motor
    ledcWrite(RPWM_PIN, 0);
    ledcWrite(LPWM_PIN, 0);
  }

  // Depuración
  Serial.print("Input: ");
  Serial.print(input, 1);
  Serial.print(" | Absolute Angle: ");
  Serial.print(absoluteAngle, 1);
  Serial.print(" | Adjusted Setpoint: ");
  Serial.print(adjustedSetpoint, 1);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | Alpha: ");
  Serial.print(alpha);
  Serial.print(" | Derivative: ");
  Serial.print(derivative);
  Serial.print(" | Filtered Derivative: ");
  Serial.print(filteredDerivative);
  Serial.print(" | Output: ");
  Serial.print(output);
  Serial.print(" | Integral: ");
  Serial.print(integral);
  Serial.print(" | RPWM: ");
  Serial.print(output > 0 ? 0 : -output);
  Serial.print(" | LPWM: ");
  Serial.println(output > 0 ? output : 0);

  delay(100); // Sincronizar con el envío (100 ms)
}
