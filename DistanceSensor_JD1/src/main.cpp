#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(22, 21);
}

void loop() {
  // put your main code here, to run repeatedly:
}
