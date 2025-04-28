#include <Arduino.h>
#include <mcp_can.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "vesc_can_bus_arduino.h"

MPU6050 mpu;
//---------------------------------------------------------------------//
//MPU VARIABLES
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

float ax, ay, az;
float gx, gy, gz;
float yaw, pitch, roll;

// Buffers para desviación estándar
const int windowSize = 50;
float ax_buffer[windowSize], ay_buffer[windowSize], az_buffer[windowSize];
float gx_buffer[windowSize], gy_buffer[windowSize], gz_buffer[windowSize];
float yaw_buffer[windowSize], pitch_buffer[windowSize], roll_buffer[windowSize];
int bufferIndex = 0;
bool bufferFilled = false;

// Desviaciones estándar y suavizado
float std_ax = 0, std_ay = 0, std_az = 0;
float std_gx = 0, std_gy = 0, std_gz = 0;
float std_yaw = 0, std_pitch = 0, std_roll = 0;

float std_ax_f = 0, std_ay_f = 0, std_az_f = 0;
float std_gx_f = 0, std_gy_f = 0, std_gz_f = 0;
float std_yaw_f = 0, std_pitch_f = 0, std_roll_f = 0;

float alpha = 0.1;
//_____________________________________________________________________________________________//
MCP_CAN CAN0(5); // CS pin
long last_print_data;
unsigned long last_can_update = 0; // Para enviar comandos CAN periódicamente
const unsigned long CAN_UPDATE_INTERVAL = 50; // Intervalo en ms para actualizar VESC

VescCAN llanta1(CAN0, 10);    // ID 10
VescCAN llanta2(CAN0, 11);    // ID 11
VescCAN llanta3(CAN0, 12);    // ID 12
VescCAN llanta4(CAN0, 13);    // ID 13
VescCAN* llantas[] = {&llanta1, &llanta2, &llanta3, &llanta4};
const int numLlantas = 4;

String receivedData = "";
int targetRpms[4] = {0, 0, 0, 0}; // Almacena los RPM objetivo para cada llanta


float calculateSTD(float *buffer, int size) {
  float sum = 0, mean, std = 0;
  for (int i = 0; i < size; i++) sum += buffer[i];
  mean = sum / size;
  for (int i = 0; i < size; i++) std += pow(buffer[i] - mean, 2);
  return sqrt(std / size);
}

void applyEMA(float &filtered, float raw) {
  filtered = alpha * raw + (1 - alpha) * filtered;
}


void setup() {
  Serial.begin(115200);
  //_______________________________//
  Wire.begin();

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP listo.");
  } else {
    Serial.print("Error al iniciar el DMP. Código: ");
    Serial.println(devStatus);
  }
  //__________________________//
  Serial.println("ESP32 UART inicializado");
  CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ); // Set the CAN bus speed and clock frequency
  CAN0.setMode(MCP_NORMAL);
}

void loop() {
  // Leer datos completos desde UART
  
  if (Serial.available()) {
    receivedData = Serial.readStringUntil('\n');
    if (receivedData.length() > 0) {
      Serial.print("UART: '");
      Serial.print(receivedData);
      Serial.println("'");
      TotalControl(receivedData);
    }
  }

  // Actualizar estado de los VESC
  llanta1.spin();
  llanta2.spin();
  llanta3.spin();
  llanta4.spin();


  // Enviar comandos de velocidad a los VESC periódicamente
  if (millis() - last_can_update >= CAN_UPDATE_INTERVAL) {
    for (int i = 0; i < numLlantas; i++) {
      llantas[i]->vesc_set_erpm(targetRpms[i]);
    }
    last_can_update = millis();
  }

  // Monitoreo periódico de datos
  if (millis() - last_print_data > 100) {
    for (int i = 0; i < numLlantas; i++) {
      Serial.print("Llanta ");
      Serial.print(i + 1);
      Serial.println(" data:");
      Serial.print("eRPM: ");
      Serial.println(llantas[i]->erpm);
      Serial.print("voltage: ");
      Serial.println(llantas[i]->inpVoltage);
      Serial.print("dutyCycleNow: ");
      Serial.println(llantas[i]->dutyCycleNow);
      Serial.print("avgInputCurrent: ");
      Serial.println(llantas[i]->avgInputCurrent);
      Serial.print("avgMotorCurrent: ");
      Serial.println(llantas[i]->avgMotorCurrent);
      Serial.print("tempFET: ");
      Serial.println(llantas[i]->tempFET);
      Serial.print("tempMotor: ");
      Serial.println(llantas[i]->tempMotor);
      Serial.print("WattHours: ");
      Serial.println(llantas[i]->WattHours);
      Serial.println("..............................");
    }
    last_print_data = millis();
    Serial.println();
  }
  delay(5); // Pequeño retraso para evitar saturar el loop

  //________________________________________________________//
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw   = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll  = ypr[2] * 180 / M_PI;

    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    ax = (ax_raw / 16384.0) * 9.81;
    ay = (ay_raw / 16384.0) * 9.81;
    az = (az_raw / 16384.0) * 9.81;
    gx = gx_raw / 131.0;
    gy = gy_raw / 131.0;
    gz = gz_raw / 131.0;

    // Guardar en buffers
    ax_buffer[bufferIndex] = ax;
    ay_buffer[bufferIndex] = ay;
    az_buffer[bufferIndex] = az;
    gx_buffer[bufferIndex] = gx;
    gy_buffer[bufferIndex] = gy;
    gz_buffer[bufferIndex] = gz;
    yaw_buffer[bufferIndex] = yaw;
    pitch_buffer[bufferIndex] = pitch;
    roll_buffer[bufferIndex] = roll;

    bufferIndex++;
    if (bufferIndex >= windowSize) {
      bufferIndex = 0;
      bufferFilled = true;
    }

    if (bufferFilled) {
      std_ax = calculateSTD(ax_buffer, windowSize);
      std_ay = calculateSTD(ay_buffer, windowSize);
      std_az = calculateSTD(az_buffer, windowSize);
      std_gx = calculateSTD(gx_buffer, windowSize);
      std_gy = calculateSTD(gy_buffer, windowSize);
      std_gz = calculateSTD(gz_buffer, windowSize);
      std_yaw = calculateSTD(yaw_buffer, windowSize);
      std_pitch = calculateSTD(pitch_buffer, windowSize);
      std_roll = calculateSTD(roll_buffer, windowSize);

      applyEMA(std_ax_f, std_ax);
      applyEMA(std_ay_f, std_ay);
      applyEMA(std_az_f, std_az);
      applyEMA(std_gx_f, std_gx);
      applyEMA(std_gy_f, std_gy);
      applyEMA(std_gz_f, std_gz);
      applyEMA(std_yaw_f, std_yaw);
      applyEMA(std_pitch_f, std_pitch);
      applyEMA(std_roll_f, std_roll);
    }

    // Imprimir datos
    Serial.print("YPR,"); Serial.print(yaw); Serial.print(",");
    Serial.print(pitch); Serial.print(","); Serial.println(roll);

    Serial.print("ACC,"); Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(","); Serial.println(az);

    Serial.print("GYRO,"); Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(","); Serial.println(gz);

    Serial.print("STD_ACC,"); Serial.print(std_ax_f); Serial.print(",");
    Serial.print(std_ay_f); Serial.print(","); Serial.println(std_az_f);

    Serial.print("STD_GYRO,"); Serial.print(std_gx_f); Serial.print(",");
    Serial.print(std_gy_f); Serial.print(","); Serial.println(std_gz_f);

    Serial.print("STD_YPR,"); Serial.print(std_yaw_f); Serial.print(",");
    Serial.print(std_pitch_f); Serial.print(","); Serial.println(std_roll_f);
    Serial.println("....................................................\n");

    delay(5);

  //________________________________________________________//
}
}


// Función para acelerar/desacelerar suavemente una llanta
void accelerate(int motorIndex, int targetRpm) {
  static int currentRpm[4] = {0, 0, 0, 0}; // Almacena el RPM actual de cada llanta
  int step = (targetRpm > currentRpm[motorIndex]) ? 1 : -1; // Incremento/decremento

  // Continuar hasta alcanzar el RPM objetivo
  while (currentRpm[motorIndex] != targetRpm) {
    currentRpm[motorIndex] += step;
    // Asegurar que no sobrepasemos el objetivo
    if ((step > 0 && currentRpm[motorIndex] > targetRpm) || (step < 0 && currentRpm[motorIndex] < targetRpm)) {
      currentRpm[motorIndex] = targetRpm;
    }
    llantas[motorIndex]->vesc_set_erpm(currentRpm[motorIndex]);
    delay(20); // Retraso para suavizar la transición
  }
  // Aplicar el valor final exacto
  currentRpm[motorIndex] = targetRpm;
  llantas[motorIndex]->vesc_set_erpm(currentRpm[motorIndex]);
  targetRpms[motorIndex] = targetRpm; // Actualizar el RPM objetivo
}

// Función para procesar los datos recibidos por UART
void TotalControl(String data) {
  String parts[4]; // Esperamos datos para las 4 llantas
  int index = 0;
  int start = 0;
  int end = data.indexOf(';', start);

  // Dividir la cadena en partes
  while (end != -1 && index < 4) {
    parts[index++] = data.substring(start, end);
    start = end + 1;
    end = data.indexOf(';', start);
  }
  if (start < data.length()) {
    parts[index++] = data.substring(start);
  }

  // Parsear RPM para cada llanta (M1 a M4)
  int newRpms[4] = {0, 0, 0, 0}; // Almacena los nuevos RPM
  for (int i = 0; i < min(index, 4); i++) {
    String part = parts[i];
    int colonPos = part.indexOf(':');
    if (colonPos != -1) {
      newRpms[i] = part.substring(colonPos + 1).toInt(); // Extraer RPM
    }
  }

  // Aplicar RPM a cada llanta con aceleración suave
  for (int i = 0; i < 4; i++) {
    accelerate(i, newRpms[i]);
  }
}
