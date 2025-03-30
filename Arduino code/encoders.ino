#include "AS5600.h"
#include "Wire.h"
#include "Arduino.h"

// Known raw values and their corresponding degrees
// 180°=98, 90°=1074, 0°=1884, 270°=-755

AS5600 as5600;
int enc, grades, aux, cont;

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  Wire.begin();

  as5600.begin(2);
  as5600.setDirection(AS5600_CLOCK_WISE);

  Serial.println(as5600.getAddress());
  delay(100);
}

int rawToDegrees(int raw) {
  // Define known points
  const int points[4] = {98, 1074, 1884, -755}; 
  const int angles[4] = {0, 90, 180, 270};       

  if (raw < -162) {
    raw += 4096;
  }
  raw = raw % 4096;

  // Map raw values to degrees based on known points
  if (raw >= points[0] && raw < points[1]) {
    return map(raw, points[0], points[1], angles[0], angles[1]);
  }
  else if (raw >= points[1] && raw < points[2]) {
    return map(raw, points[1], points[2], angles[1], angles[2]);
  }
  else if (raw >= points[2] || raw < points[3]) {
    if (raw >= points[2]) { 
      return map(raw, points[2], 4095, angles[2], 360);
    }
    else { 
      return map(raw, 0, points[3], 360, angles[3]);
    }
  }
  else { 
    return map(raw, points[3], points[0], angles[3], 360);
  }
}

// Helper function to map raw values to integer degrees (Arduino's map)
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop()
{
  static uint32_t lastTime = 0;

  enc = as5600.getCumulativePosition();

  grades = rawToDegrees(enc);
    
  if (grades != aux || cont == 100) {
    Serial.println(grades);
    aux = grades;
    cont = 0;
  }
  else {
    cont++;
  }
  delay(10);
}
