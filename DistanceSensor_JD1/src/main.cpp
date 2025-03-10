#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <string>


/***********  MACROS  ******************/

// VL53L0X CONFIG MACROS
#define VL53L0X_XSHUT_PIN 5  // ESP32 pin connected to VL53L0X XSHUT
#define VL53L0X_GPIO1_PIN 7  // ESP32 pin connected to VL53L0X GPIO1 (optional)
#define VL53L0X_ADDRESS 0x29 // Default I2C address for VL53L0X

// VL53L0X USER CONFIG
#define USING_INTERRUPT 0    // Set to 1 if you want to use interrupt
#define USING_GPIO1 0        // Set to 1 if you want to use GPIO1

// LCD USER CONFIG
#define I2C_LCD 1
#define PARALLEL_LCD 0


#if I2C_LCD
#include <LiquidCrystal_I2C.h> // If using I2C LCD
// LCD Configuration (I2C)
#define LCD_ADDRESS 0x27 // I2C Address of the LCD from I2C Scanner
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS); //I2C LCD
#endif

#if PARALLEL_LCD
// LCD Configuration (Parallel)
#include <LiquidCrystal.h> // If using Parallel LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); //Parallel LCD
#endif
/************************END OF MACROS*******************************/

Adafruit_VL53L0X sensor = Adafruit_VL53L0X();

void setup() {
  
  Serial.begin(115200); // Standard Baud Rate for I2C
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

  if (!sensor.begin()) { //Initialize the sensor
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

#define I2C_SCANNER 0
#define VL53L0X_RANGE_STATUS_CHECK 0
void loop() {

  //sensor.startMeasurement(); // Start a new measurement
VL53L0X_RangingMeasurementData_t measure;
sensor.startRange(); // Perform a single measurement

  
//uint16_t distance = sensor.getRangingMeasurement(&measure);
uint16_t distance = sensor.readRange();
measure.RangeStatus;

#if VL53L0X_RANGE_STATUS_CHECK
if (measure.RangeStatus != 0) {
  Serial.print("Range Status: ");
  Serial.println(measure.RangeStatus);
  lcd.setCursor(0, 0);
  lcd.println("Range Status:  ");
  lcd.setCursor(14, 0);
  lcd.print(measure.RangeStatus);
  lcd.setCursor(0, 1);
  lcd.println("                ");
  delay(100);
  return;
}
#else
  if (distance < 1320) {
    // Valid measurement
    Serial.print("Distance:      ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
    lcd.setCursor(0, 0);
    lcd.println("Distance:       ");
    
    lcd.setCursor(10,0);
    //lcd.print(measure.RangeMilliMeter);
    
    lcd.print(distance);
    lcd.setCursor(14, 0);
    lcd.print("mm");
    lcd.setCursor(0, 1);
    lcd.println("                ");
  } else {
    Serial.println("OUT OF RANGE   ");
    lcd.setCursor(0, 0);
    lcd.println("  INVALID READ  ");
    lcd.setCursor(0, 1);
    lcd.println("  OUT OF RANGE  ");
  }
#endif

  delay(100); // Adjust delay as needed
}


#if I2C_SCANNER
void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}
 
void loop() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}
#endif