
/*
  GY-271 Compass
  modified on 3/24/2025
  Library sources: https://github.com/mprograms/QMC5883LCompass
*/

// I2C Library
#include <Wire.h>
// QMC5883L Compass Library
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  // Initialize the serial port.
  Serial.begin(115200);
  // Initialize I2C.
  Wire.begin();
  // Initialize the Compass.
  compass.init();
  compass.setCalibrationOffsets(-5.00, 72.00, -142.00);
  compass.setCalibrationScales(1.02, 72.00, 1.03);
  compass.setMode("0x00", "0x00", "0x10", "0x00");
}

void loop() {
  int x, y, z, a;
  char myArray[3];

  // Read compass values
  compass.read();

  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  a = compass.getAzimuth();

  compass.getDirection(myArray, a);

  Serial.print("X: ");
  Serial.print(x);
  Serial.print("   Y: ");
  Serial.print(y);
  Serial.print("   Z: ");
  Serial.print(z);
  Serial.print(" Azimuth ");
  Serial.print(a);
  Serial.print("  Direction:");
  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.println(myArray[2]);

  delay(300);
}

