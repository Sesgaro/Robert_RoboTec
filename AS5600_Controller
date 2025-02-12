#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("68:6c:e6:49:74:ed");

const int pin = 48;

// SET WIFI CREDENTIALS
const char* ssid = "robot_1";
const char* password = "robot123";

// Definición de la clase llanta
class llanta {
  private:
    int pwmPin;
    int dirPin;
    int pwmValue;

  public:
    llanta() : pwmPin(0), dirPin(0), pwmValue(0) {}

    llanta(int pwmp, int dirp, int value) {
      this->pwmPin = pwmp;
      this->dirPin = dirp;
      this->pwmValue = value;
      pinMode(pwmPin, OUTPUT);
      pinMode(dirPin, OUTPUT);
    }

    void setPwmValue(int nuevo_value) {
      this->pwmValue = nuevo_value;
    }

    void adelante() {
      pwmValue = 0;
      digitalWrite(dirPin, HIGH);
      for(int i=0; i<250;i++){ 
        pwmValue = i;
        analogWrite(pwmPin, pwmValue);
        delay(50);
      }
      Serial.println("ADELANTE");
    }

    void atras() {
      pwmValue = 0;
      digitalWrite(dirPin, LOW);
      for(int i=0; i<250;i++){ 
        pwmValue = i;
        analogWrite(pwmPin, pwmValue);
        delay(50);
      }
      Serial.println("ATRAS");
    }

    void stop() {
      for(int i=250; i >= 0;i--){ 
        pwmValue = i;
        analogWrite(pwmPin, pwmValue);
        delay(50);
      }
      Serial.println("STOP");
    }
};

// Instancias globales de las llantas
llanta llanta1(4, 5, 0);  // Adelante Izquierda
llanta llanta2(7, 15, 0);  // Atras Izquierda
llanta llanta3(41, 40, 0);  // Adelante Derecha
llanta llanta4(37, 36, 0);    // Atras Derecha

char test = '0';

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  delay(50);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  ArduinoOTA.begin(); //BEGGIN OTA
  neopixelWrite(pin,0,0,0);
  Serial.println("Starting NimBLE Client");
  xboxController.begin();
}

void loop() {
  ArduinoOTA.handle(); //HANDLE OTA
  xboxController.onLoop();
  if (xboxController.isConnected()) {
    if (xboxController.isWaitingForFirstNotification()) {
      Serial.println("waiting for first notification");
    } 
  
    else {
      neopixelWrite(pin,255,128,0);
      Serial.println("Address: " + xboxController.buildDeviceAddressStr());
      Serial.print(xboxController.xboxNotif.toString());
      unsigned long receivedAt = xboxController.getReceiveNotificationAt();
      uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
      Serial.print("joyLHori rate: ");
      Serial.println((float)xboxController.xboxNotif.joyLHori / joystickMax);
      Serial.print("joyLVert rate: ");
      Serial.println((float)xboxController.xboxNotif.joyLVert / joystickMax);
      Serial.println("battery " + String(xboxController.battery) + "%");
      Serial.println("received at " + String(receivedAt));
    }
  } else {
    Serial.println("not connected");
    neopixelWrite(pin,0,0,0);
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }
  }
  delay(500);
  //switch (test) {
  //  case '1':
  //    llanta1.adelante();
  //    llanta2.adelante();
  //    llanta3.adelante();
  //    llanta4.adelante();
  //    break;
  //  case '2':
  //    llanta1.atras();
  //    llanta2.atras();
  //    llanta3.atras();
  //    llanta4.atras();
  //    break;
  //  case '3':
  //    llanta1.atras();
  //    llanta2.atras();
  //    llanta3.adelante();
  //    llanta4.adelante();
  //    break;
  //  case '4':
  //    llanta3.atras();
  //    llanta4.atras();
  //    llanta1.adelante();
  //    llanta2.adelante();
  //    break;
  //  case '5':
  //    llanta1.stop();
  //    llanta2.stop();
  //    llanta3.stop();
  //    llanta4.stop();
  //    break;
  //  default:
  //    Serial.println("Esperando orden válida...");
  //    delay(500);
  //    break;
  //}
}
