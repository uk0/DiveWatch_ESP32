// I2C Scanner for ESP32-S3
// Use this to verify MS5837 (0x76) and OLED SH1106 (0x3C) are wired correctly
// Open Serial Monitor at 115200 baud after flashing.

#include <Wire.h>

// Arduino Nano ESP32 board labels: A4 = GPIO 11 (SDA), A5 = GPIO 12 (SCL)
#define I2C_SDA 11   // wire to board pin labeled "A4"
#define I2C_SCL 12   // wire to board pin labeled "A5"

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println();
  Serial.println("=== I2C Scanner (ESP32-S3) ===");
  Serial.printf("SDA=GPIO%d  SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  Found device at 0x%02X", address);
      if (address == 0x3C || address == 0x3D) Serial.print("  <- OLED (SH1106/SSD1306)");
      if (address == 0x76 || address == 0x77) Serial.print("  <- MS5837 / BMP280-class");
      Serial.println();
      nDevices++;
    } else if (error == 4) {
      Serial.printf("  Unknown error at 0x%02X\n", address);
    }
  }
  if (nDevices == 0) {
    Serial.println("  !!! No I2C devices found.");
    Serial.println("  Check: SDA/SCL swapped? VCC=3.3V? Pull-ups present? Common GND?");
  } else {
    Serial.printf("  Done. %d device(s).\n", nDevices);
  }
  Serial.println();
  delay(3000);
}
