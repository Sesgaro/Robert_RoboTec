#include <Arduino.h>
#include <mcp_can.h>
#include "vesc_can_bus_arduino.h"

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

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 UART inicializado");
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

  delay(10); // Pequeño retraso para evitar saturar el loop
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
