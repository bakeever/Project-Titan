/*
===============================================================================================================
QMC5883LCompass.h Library Azimuth Example Sketch
Learn more at [https://github.com/mprograms/QMC5883Compas]
===============================================================================================================
v0.3 - June 12, 2019
Written by MRPrograms 
Github: [https://github.com/mprograms/]

Release under the GNU General Public License v3
[https://www.gnu.org/licenses/gpl-3.0.en.html]
===============================================================================================================
*/
#include <QMC5883LCompass.h>
#include <Wire.h>
QMC5883LCompass compass;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("about to compass");
  compass.init();
  Serial.println("compassed");
  
  //compass.setCalibrationOffsets(-310.00, 2154.00, -3118.00);
  //compass.setCalibrationScales(1.75, 0.86, 0.79);
  
  //compass.setCalibrationOffsets(156.00, 74.00, -667.00);
  //compass.setCalibrationScales(0.95, 1.11, 0.96);
  
  //compass.setCalibrationOffsets(397.00, -344.00, -973.00);
  //compass.setCalibrationScales(1.13, 0.79, 1.18);
  
  //compass.setCalibrationOffsets(11.00, 1465.00, -87.00);
  //compass.setCalibrationScales(1.27, 0.69, 1.30);
  //compass.setMode("0x00", "0x00", "0x10", "0x00");
}

void loop() {
  int a, x,y,z;
  
  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  
  //Serial.print("X: ");
  //Serial.print(x);
  //Serial.print(" Y: ");
  //Serial.print(y);
  //Serial.print(" Z: ");
  //Serial.print(z);
  //Serial.print(" ");

  // Return Azimuth reading
  a = compass.getAzimuth();
  Serial.print("RAW:");
  Serial.print(a);
  Serial.print("     ");
  a = a+270;
  if(a>360){a=a-360;}
  Serial.print("A: ");
  Serial.print(a);
  Serial.print(" ");

  byte b = compass.getAzimuth();

  char myArray[3];
  compass.getDirection(myArray, a);
  
  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.print(myArray[2]);
  Serial.println();
  
  delay(250);
}
