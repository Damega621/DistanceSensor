#include <Arduino.h>
//#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <LiquidCrystal_I2C.h> // If using I2C LC
#include <string>


// VL53L0X Configuration
#define VL53L0X_XSHUT_PIN 5  // ESP32 pin connected to VL53L0X XSHUT
//#define VL53L0X_GPIO1_PIN 7  // ESP32 pin connected to VL53L0X GPIO1 (optional)

Adafruit_VL53L0X sensor = Adafruit_VL53L0X();



// LCD Configuration (I2C)
#define LCD_ADDRESS 0x27 //Test Using 
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS); //I2C LCD



// Previos LCD Configuration (Parallel), Scratched after switching to I2C
// const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
// LiquidCrystal lcd(rs, en, d4, d5, d6, d7); //Parallel LCD


void setup() {

  
  Serial.begin(115200);
  Wire.begin(21,22);  // Initialize I2C

  // LCD Initialization
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.println("Initializing... "); // Display a message on the first row";
  delay(1000);
  lcd.clear();

  // VL53L0X Initialization
  
  pinMode(VL53L0X_XSHUT_PIN, OUTPUT);
  digitalWrite(VL53L0X_XSHUT_PIN, LOW); // Reset Sensor
  delay(10);
  digitalWrite(VL53L0X_XSHUT_PIN, HIGH); // Enable Sensor
  delay(10);

  // Initialize VL53L0X
  if (!sensor.begin()) {
    lcd.clear();
    lcd.print("Sensor Error!");
    while (1);
  }

  sensor.setMeasurementTimingBudgetMicroSeconds(20000); // Adjust as needed (in microseconds)
  //sensor.setGpioConfig(VL53L0X_GPIO1_PIN, LOW, false); // Set GPIO1 pin to LOW, adn disable interrupt
  
  sensor.startRangeContinuous(0); // Start continuous mode

  // Calibrate the sensor (optional, but recommended)
  // You might need to perform offset and crosstalk calibration
  // as described in the VL53L0X datasheet and API user manual.

}

void loop() {

  VL53L0X_RangingMeasurementData_t measure;
  sensor.startRange(); // Perform a single measurement

  
//uint16_t distance = sensor.getRangingMeasurement(&measure);
uint16_t distance = sensor.readRange();


if (distance <= 1200 || distance >= 150) {
  // Valid measurement
  Serial.print("Distance: ");
  Serial.print(measure.RangeMilliMeter);
  Serial.println(" mm");
  lcd.setCursor(0, 0);
  lcd.println("Distance: ");
  
  lcd.setCursor(0,10);
  //lcd.print(measure.RangeMilliMeter);
  
  lcd.print(distance);
  lcd.print("    mm");
} else {
  Serial.println("OUT OF RANGE   ");
  lcd.setCursor(0, 1);
  lcd.println("OUT OF RANGE    ");
}

  delay(100); // Adjust delay as needed
}


// void setup() {
//   Wire.begin();
//   Serial.begin(115200);
//   Serial.println("\nI2C Scanner");
// }
 
// void loop() {
//   byte error, address;
//   int nDevices;
//   Serial.println("Scanning...");
//   nDevices = 0;
//   for(address = 1; address < 127; address++ ) {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();
//     if (error == 0) {
//       Serial.print("I2C device found at address 0x");
//       if (address<16) {
//         Serial.print("0");
//       }
//       Serial.println(address,HEX);
//       nDevices++;
//     }
//     else if (error==4) {
//       Serial.print("Unknow error at address 0x");
//       if (address<16) {
//         Serial.print("0");
//       }
//       Serial.println(address,HEX);
//     }    
//   }
//   if (nDevices == 0) {
//     Serial.println("No I2C devices found\n");
//   }
//   else {
//     Serial.println("done\n");
//   }
//   delay(5000);          
// }