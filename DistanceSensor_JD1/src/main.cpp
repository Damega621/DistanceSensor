#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>


void setup() {
  int sda = 22;
  int scl = 21;
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(sda, scl);
}

void loop() {

  VL53L0X_RangingMeasurementData_t measurements;

  if (measurements.RangeStatus == 0) {
    Serial.print("Distance: ");
    Serial.print(measurements.RangeMilliMeter);
    Serial.print(" mm");
  }
  else
    Serial.print("Out of range.");
  
  
  delay (500);
}
