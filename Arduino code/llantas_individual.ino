#define pwmPin 32      // Pin para salida PWM,  
#define pinDir 33
#define pwmPin1 26
#define pinDir1 27
#define pwmPin2 23
#define pinDir2 22
#define pwmPin3 21
#define pinDir3 19     //pin para dirección
int test = 0;
int pwmValue = 80;         // Valor constante de PWM, ajusta según necesidad (0-255)

void setup() {
  Serial.begin(115200);
  pinMode(pwmPin, OUTPUT);
  pinMode(pinDir, OUTPUT);
  
}

void loop() {

      digitalWrite(pinDir, HIGH);  // Establecer dirección
      analogWrite(pwmPin, pwmValue); // Aplicar valor PWM al motor
      Serial.println("Iniciando");
      for(int i=0; i<80;i++){ 
        pwmValue = i;
        analogWrite(pwmPin, pwmValue);
        delay(10);
      }
      delay(3000);
      Serial.println("Movimiento hacia adelante finalizado 1");
      for(int i=100; 0<i ;i--){ 
        pwmValue = i;
        analogWrite(pwmPin, pwmValue);
        delay(10);
      }
      delay(100);
      analogWrite(pwmPin, 0);

      //----------------------------------------------------------//
      
      digitalWrite(pinDir1,HIGH);  // Establecer dirección 
      Serial.println("Iniciando");
      for(int i=0; i<80;i++){ 
        pwmValue = i;
        analogWrite(pwmPin1, pwmValue);
        delay(10);
      }
      delay(3000);
      Serial.println("Movimiento hacia adelante finalizado 1");
      for(int i=100; 0<i ;i--){ 
        pwmValue = i;
        analogWrite(pwmPin1, pwmValue);
        delay(10);
      }
      delay(100);
      analogWrite(pwmPin1, 0);

  //----------------------------------------------------------//
      
      digitalWrite(pinDir2, HIGH);   
      Serial.println("Iniciando");
      for(int i=0; i<80;i++){ 
        pwmValue = i;
        analogWrite(pwmPin2, pwmValue);
        delay(10);
      }
      delay(3000);
      Serial.println("Movimiento hacia adelante finalizado 1");
      for(int i=100; 0<i ;i--){ 
        pwmValue = i;
        analogWrite(pwmPin2, pwmValue);
        delay(10);
      }
      delay(100);
      analogWrite(pwmPin2, 0);

  //----------------------------------------------------------//

      digitalWrite(pinDir3, HIGH);   // Aplicar valor PWM al motor
      Serial.println("Iniciando");
      for(int i=0; i<100;i++){ 
        pwmValue = i;
        analogWrite(pwmPin3, pwmValue);
        delay(10);
      }
      delay(3000);
      Serial.println("Movimiento hacia adelante finalizado 1");
      for(int i=100; 0<i ;i--){ 
        pwmValue = i;
        analogWrite(pwmPin3, pwmValue);
        delay(10);
      }
      delay(100);
      analogWrite(pwmPin3, 0);
}
