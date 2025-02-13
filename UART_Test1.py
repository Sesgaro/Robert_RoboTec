'''
TEST 1
This code is a test for read a diferent ESP in the Jetson
'''

'''
//Code 1

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(112500);
}
void loop() {
 
  Serial.println("Hola!");
  delay(1000); 
  Serial.println("Soy la una ESP32S3!");
  delay(1000);                        
  Serial.println("Esta es la prueba 1");
  delay(1000);            
  Serial.println("La otra placa tiene texto diferente");
  delay(1000);
  Serial.println("Ahora, prueba de numeros");
  delay(1000);
  for(int i=0; i<1000 ; i++){
    Serial.println(i);
  }
  delay(1000);
  Serial.println("Bueno, gusbay");
  delay(1000);
}
'''
'''
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(112500);
}
void loop() {
 
  Serial.println("Hola!");
  delay(1000); 
  Serial.println("Soy la una ESP32-S3!");
  delay(1000);                        
  Serial.println("Esta es la prueba 1, hmmmm");
  delay(1000);            
  Serial.println("La placa a lado tiene otros textos");
  delay(1000);
  Serial.println("Ahora, prueba de digitos");
  delay(1000);
  for(int i=0; i<1000 ; i++){
    Serial.println(i);
  }
  delay(1000);
  Serial.println("Bueno, adios");
  delay(1000);
}
'''

import serial

def read_esp(puerto1: str, puerto2: str, baudrate=115200):
    try:
        esp1 = serial.Serial(puerto1, baudrate, timeout=1)
        esp2 = serial.Serial(puerto2, baudrate, timeout=1)
        print(f"Conecting to {puerto1} and {puerto2}")
        
        while True:
            if esp1.in_waiting:
                datos1 = esp1.readline().decode('utf-8').strip()
                print(f"ESP1: {datos1}")
            
            if esp2.in_waiting:
                datos2 = esp2.readline().decode('utf-8').strip()
                print(f"ESP2: {datos2}")
    
    except serial.SerialException as e:
        print(f"Error: {e}")
    finally:
        esp1.close()
        esp2.close()

# Linux/Mac: leer_placas('/dev/ttyUSB0', '/dev/ttyUSB1'

read_esp('COM8','COM10')