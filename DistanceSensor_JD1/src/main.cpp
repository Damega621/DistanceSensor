#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>


// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}


//#include <LiquidCrystal_I2C.h> // If using I2C LC
//#include <LiquidCrystal.h>     // If using parallel LCD

// VL53L0X Configuration
#define VL53L0X_XSHUT_PIN 5  // ESP32 pin connected to VL53L0X XSHUT
#define VL53L0X_GPIO1_PIN 4  // ESP32 pin connected to VL53L0X GPIO1 (optional)

 Adafruit_VL53L0X sensor;

// LCD Configuration (I2C)
#define LCD_ADDRESS 0x27 // Check your LCD's I2C address (usually 0x27 or 0x3F)
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS); //I2C LCD

//Parallel LCD config
//const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7); //Parallel LCD


void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C

  // LCD Initialization
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Distance: ");

  // VL53L0X Initialization
  pinMode(VL53L0X_XSHUT_PIN, OUTPUT);
  digitalWrite(VL53L0X_XSHUT_PIN, LOW); // Reset the sensor
  delay(10);
  digitalWrite(VL53L0X_XSHUT_PIN, HIGH); // Enable the sensor
  delay(10);

  sensor.init();
  sensor.setMeasurementTimingBudget(20000); // Adjust as needed (in microseconds)

  // Calibrate the sensor (optional, but recommended)
  // You might need to perform offset and crosstalk calibration
  // as described in the VL53L0X datasheet and API user manual.

}

void loop() {
  sensor.startRange(); // Perform a single measurement
  unsigned int distance = sensor.readRangeResultMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.println("Timeout occurred!");
    lcd.setCursor(0, 1);
    lcd.print("Timeout!        ");  //Clear row
  } else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");

    // Display on LCD
    lcd.setCursor(0, 1); // Set cursor to the second row
    lcd.print(distance);
    lcd.print(" mm        "); // Pad with spaces to clear previous digits
  }

  delay(100); // Adjust delay as needed
}
