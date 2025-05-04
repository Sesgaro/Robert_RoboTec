#include <Arduino.h>
#include <mcp_can.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "vesc_can_bus_arduino.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

// Definiciones para multitarea
#define CORE_0 0
#define CORE_1 1

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

int targetRpms[4] = {0, 0, 0, 0}; // Almacena los RPM objetivo para cada llanta

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1) {delay(100);}}}
#define EXECUTE(fn) { if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) {error_loop();}}

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscription;
std_msgs__msg__Float32MultiArray sensor_data_msg;
std_msgs__msg__Float32MultiArray target_rpms_msg;
rclc_executor_t executor;

TaskHandle_t microRosTaskHandle;
TaskHandle_t arduinoTaskHandle;

// Prototipos de funciones de tareas
void microRosTask(void *pvParameters);
void arduinoTask(void *pvParameters);

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

void error_loop() {
  while (1) {
    delay(100);
  }
}

// Callback para recibir RPM objetivo
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size == 4) {
    for (int i = 0; i < 4; i++) {
      targetRpms[i] = (int)msg->data.data[i];
      Serial.print("RPM objetivo Llanta ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(targetRpms[i]);
    }
  }
}

// Tarea para Micro-ROS (Núcleo 0)
void microRosTask(void *pvParameters) {
  while (1) {
    EXECUTE(rclc_executor_spin_some);
    delay(10); // Pequeño retraso para evitar saturación
  }
}

// Tarea para Arduino (Núcleo 1)
void arduinoTask(void *pvParameters) {
  while (1) {
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

    if (!dmpReady) {
      delay(10);
      continue;
    }

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      yaw = ypr[0] * 180 / M_PI;
      pitch = ypr[1] * 180 / M_PI;
      roll = ypr[2] * 180 / M_PI;

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

      // Publicar datos del IMU y RPM
      sensor_data_msg.data.data[0] = yaw;
      sensor_data_msg.data.data[1] = pitch;
      sensor_data_msg.data.data[2] = roll;
      sensor_data_msg.data.data[3] = ax;
      sensor_data_msg.data.data[4] = ay;
      sensor_data_msg.data.data[5] = az;
      sensor_data_msg.data.data[6] = gx;
      sensor_data_msg.data.data[7] = gy;
      sensor_data_msg.data.data[8] = gz;
      sensor_data_msg.data.data[9] = std_yaw_f;
      sensor_data_msg.data.data[10] = std_pitch_f;
      sensor_data_msg.data.data[11] = std_roll_f;
      sensor_data_msg.data.data[12] = std_ax_f;
      sensor_data_msg.data.data[13] = std_ay_f;
      sensor_data_msg.data.data[14] = std_az_f;
      sensor_data_msg.data.data[15] = std_gx_f;
      sensor_data_msg.data.data[16] = std_gy_f;
      sensor_data_msg.data.data[17] = std_gz_f;
      sensor_data_msg.data.data[18] = llanta1.erpm;
      sensor_data_msg.data.data[19] = llanta2.erpm;
      sensor_data_msg.data.data[20] = llanta3.erpm;
      sensor_data_msg.data.data[21] = llanta4.erpm;
      RCCHECK(rcl_publish(&publisher, &sensor_data_msg, NULL));

      // Monitoreo periódico de datos
      if (millis() - last_print_data > 100) {
        Serial.printf("Running on core %d\n", xPortGetCoreID());
        Serial.print("YPR: ");
        Serial.print(yaw); Serial.print(", ");
        Serial.print(pitch); Serial.print(", ");
        Serial.println(roll);

        Serial.print("ACC: ");
        Serial.print(ax); Serial.print(", ");
        Serial.print(ay); Serial.print(", ");
        Serial.println(az);

        Serial.print("GYRO: ");
        Serial.print(gx); Serial.print(", ");
        Serial.print(gy); Serial.print(", ");
        Serial.println(gz);

        Serial.print("STD_YPR: ");
        Serial.print(std_yaw_f); Serial.print(", ");
        Serial.print(std_pitch_f); Serial.print(", ");
        Serial.println(std_roll_f);

        Serial.print("STD_ACC: ");
        Serial.print(std_ax_f); Serial.print(", ");
        Serial.print(std_ay_f); Serial.print(", ");
        Serial.println(std_az_f);

        Serial.print("STD_GYRO: ");
        Serial.print(std_gx_f); Serial.print(", ");
        Serial.print(std_gy_f); Serial.print(", ");
        Serial.println(std_gz_f);

        Serial.print("RPM: ");
        Serial.print(llanta1.erpm); Serial.print(", ");
        Serial.print(llanta2.erpm); Serial.print(", ");
        Serial.print(llanta3.erpm); Serial.print(", ");
        Serial.println(llanta4.erpm);

        for (int i = 0; i < numLlantas; i++) {
          Serial.print("Llanta ");
          Serial.print(i + 1);
          Serial.print(" data: ");
          Serial.print("eRPM: ");
          Serial.print(llantas[i]->erpm);
          Serial.print(" | voltage: ");
          Serial.print(llantas[i]->inpVoltage);
          Serial.print(" | dutyCycleNow: ");
          Serial.println(llantas[i]->dutyCycleNow);
        }
        Serial.println("..............................");

        last_print_data = millis();
      }
      delay(5);
    }
  }
}

void setup() {
  Serial.begin(115200);
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

  Serial.println("ESP32 UART inicializado");
  CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);

  // Inicializar Micro-ROS
  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_s3_imu_node", "", &support));
  
  // Configurar publicador para datos del IMU y RPM
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "sensor_data"));
  sensor_data_msg.data.capacity = 18; // 12 IMU (yaw, pitch, roll, ax, ay, az, gx, gy, gz, 6 stds) + 4 RPM
  sensor_data_msg.data.size = 18;
  sensor_data_msg.data.data = (float*)malloc(18 * sizeof(float));
  
  // Configurar suscripción para RPM objetivo
  RCCHECK(rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "target_rpms"));
  
  // Configurar executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscription, &target_rpms_msg, &subscription_callback, ON_NEW_DATA));

  // Crear tareas para los núcleos
  xTaskCreatePinnedToCore(
    microRosTask,
    "MicroRosTask",
    10000,
    NULL,
    1,
    &microRosTaskHandle,
    CORE_0);

  xTaskCreatePinnedToCore(
    arduinoTask,
    "ArduinoTask",
    10000,
    NULL,
    1,
    &arduinoTaskHandle,
    CORE_1);

  Serial.println("Micro-ROS inicializado. Conecta el agente en /dev/ttyUSB0.");
}

void loop() {
  // El loop principal no se usa, las tareas manejan todo
// delay(1000);
}
