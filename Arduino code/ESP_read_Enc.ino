#include <Wire.h>

// Configuración I2C
#define SDA_PIN_BUS1  21
#define SCL_PIN_BUS1  22
#define SDA_PIN_BUS2  18
#define SCL_PIN_BUS2  19
#define SLAVE_ADDR_1  0x10  // Bus 1: C3_3 (Motor 3)
#define SLAVE_ADDR_2  0x11  // Bus 2: C3_4 (Motor 4)

// Configuración UART
#define UART_TX_PIN  17
#define UART_RX_PIN  16

volatile bool newDataC3_3 = false, newDataC3_4 = false;
volatile uint16_t angleC3_3 = 0, angleC3_4 = 0;
float angle3 = 0.0, angle4 = 0.0;

// Funciones I2C
void onReceiveBus1(int howMany) {
  if (howMany >= 3) {
    uint8_t id = Wire.read();
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    if (id == 0x03) {  // C3_3 (Motor 3)
      angleC3_3 = (hi << 8) | lo;
      newDataC3_3 = true;
    }
  }
}

void onReceiveBus2(int howMany) {
  if (howMany >= 3) {
    uint8_t id = Wire1.read();
    uint8_t lo = Wire1.read();
    uint8_t hi = Wire1.read();
    if (id == 0x04) {  // C3_4 (Motor 4)
      angleC3_4 = (hi << 8) | lo;
      newDataC3_4 = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Iniciando ESP32 para leer encoders C3_3 y C3_4 (I2C) y enviar por UART");

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

  Serial.printf("I2C esclavo listo: Bus 1 en 0x%02X (C3_3), Bus 2 en 0x%02X (C3_4)\n", SLAVE_ADDR_1, SLAVE_ADDR_2);
}

void loop() {
  // Actualizar ángulos desde I2C
  if (newDataC3_3) {
    angle3 = angleC3_3 / 10.0f;
    Serial.printf("Ángulo C3_3 (Motor 3): %.1f°\n", angle3);
    newDataC3_3 = false;
  }
  if (newDataC3_4) {
    angle4 = angleC3_4 / 10.0f;
    Serial.printf("Ángulo C3_4 (Motor 4): %.1f°\n", angle4);
    newDataC3_4 = false;
  }

  // Enviar ángulos por UART
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 100) {  // Enviar cada 100ms
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "A3:%.1f:A4:%.1f;", angle3, angle4);
    Serial1.print(buffer);
    Serial.printf("Enviado por UART: %s\n", buffer);
    lastSend = millis();
  }

  delay(10);
}
