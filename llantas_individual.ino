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

// 1=Adelante
// 2=Atras
// 'r'=Derecha
// 'l'=Izquierda

void setup() {
  Serial.begin(115200);
  pinMode(pwmPin, OUTPUT);
  pinMode(pinDir, OUTPUT);
  
}

void loop() {
  
  // Aquí podrías añadir lógica adicional si necesitas manejar otros aspectos
  // Por ahora, solo se mantiene constante el PWM

  if (Serial.available() > 0) {
      char input = Serial.read(); // Leer un carácter del serial
      if (isdigit(input)) {       // Verificar si es un número
        test = input - '0';       // Convertir el carácter a un número entero
      } else {
        test = -1;
      } 
    }
      Serial.println(test);
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
      Serial.println(test);
      
      digitalWrite(pinDir1,HIGH);  // Establecer dirección // Aplicar valor PWM al motor
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
      Serial.println(test);

      
      digitalWrite(pinDir2, HIGH);   // Aplicar valor PWM al motor
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
