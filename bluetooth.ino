#include <BluetoothSerial.h>

// Definición de la clase llanta
class llanta {
  private:
    int pwmPin;
    int dirPin;
    int pwmValue;
    int posValue;

  public:
    llanta() : pwmPin(0), dirPin(0), pwmValue(0), posValue(0) {}

    llanta(int pwmp, int dirp, int value, int pos) { //pos es izquierda o derecha
      this->pwmPin = pwmp;
      this->dirPin = dirp;
      this->pwmValue = value;
      this->posValue = pos;
      pinMode(pwmPin, OUTPUT);
      pinMode(dirPin, OUTPUT);
    }

    void setPwmValue(int nuevo_value) {
      this->pwmValue = nuevo_value;
    }

    void adelante() {
      pwmValue = 0;
      int aux = pwmValue;
      if (! pwmValue <= 0){
        for(int i=aux; 0 <= i;i--){ 
          pwmValue = i;
          analogWrite(pwmPin, pwmValue);
          delay(10);
        }
      }
      if(posValue == 0){  
        digitalWrite(dirPin, HIGH);
      }
      else{
        digitalWrite(dirPin, LOW);
      }
      
      for(int i=0; i<100;i++){ 
        pwmValue = i;
        analogWrite(pwmPin, pwmValue);
        delay(10);
      }
    }

    void atras() {
      int aux = pwmValue;
      if (! pwmValue <= 0){
        for(int i=aux; 0 <= i;i--){ 
          pwmValue = i;
          analogWrite(pwmPin, pwmValue);
          delay(10);
        }
      }
      if(posValue == 0){  
        digitalWrite(dirPin, LOW);
      }
      else{
        digitalWrite(dirPin, HIGH);
      }
      for(int i=0; i<100;i++){ 
        pwmValue = i;
        analogWrite(pwmPin, pwmValue);
        delay(10);
      }
    }

    void stop() {
      int aux = pwmValue;
      for(int i=aux; 0 <= i;i--){ 
        pwmValue = i;
        analogWrite(pwmPin, pwmValue);
        delay(10);
      }
    }
};

// Instancias globales de las llantas
llanta llanta1(23, 22, 0, 0);  // Adelante Izquierda
llanta llanta2(26, 27, 0, 0);  // Atras Izquierda
llanta llanta3(32, 33, 0, 1);  // Adelante Derecha
llanta llanta4(21, 19, 0, 1);    // Atras Derecha

BluetoothSerial SerialBT;
int test = 0;

void setup() {
  delay(500);
  Serial.begin(115200);
  SerialBT.begin("Twilight Roberto Sparkle"); // JAJAJAJAJ

  Serial.println("Bluetooth listo, esperando comandos...");
}

void loop() {
  if (Serial.available()) SerialBT.write(Serial.read());
  if (SerialBT.available()) {
    test = SerialBT.read();
    Serial.println(test);
  }

  switch (test) {
    case '1':
      llanta1.adelante();
      llanta2.adelante();
      llanta3.adelante();
      llanta4.adelante();
      break;
    case '2':
      llanta1.atras();
      llanta2.atras();
      llanta3.atras();
      llanta4.atras();
      break;
    case '3':
      llanta1.atras();
      llanta2.atras();
      llanta3.adelante();
      llanta4.adelante();
      break;
    case '4':
      llanta3.atras();
      llanta4.atras();
      llanta1.adelante();
      llanta2.adelante();
      break;
    case '5':
      llanta1.stop();
      llanta2.stop();
      llanta3.stop();
      llanta4.stop();
      break;
    default:
      Serial.println("Esperando orden válida...");
      delay(500);
      break;
  }
}
