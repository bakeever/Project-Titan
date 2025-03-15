/*
    Arduino and ADXL345 Accelerometer Tutorial
    by Dejan, https://howtomechatronics.com

    Another source to use: https://makersportal.com/blog/2017/9/25/accelerometer-on-an-elevator
*/

#include <Wire.h> // Wire library - used for I2C communication

int ADXL345 = 0x53; // The ADXL345 sensor I2C address

float X_out, Y_out, Z_out;  // Outputs
float X_accel[3] = {0,0,0};
float Y_accel[3] = {0,0,0};
float Z_accel[3] = {0,0,0};

float X_velocity[2] = {0,0};
float Y_velocity[2] = {0,0};
float Z_velocity[2] = {0,0};

float X_Displace = 0;
float Y_Displace = 0;
float Z_Displace = 0;

float gravity_const = 32.174; //ft/s^2 
int last_time = millis();

void setup() {
  Serial.begin(9600); // Initiate serial communication for printing the results on the Serial monitor
  Wire.begin(); // Initiate the Wire library
  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); // Start communicating with the device 
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable 
  Wire.endTransmission();
  delay(10);

  int last_time = millis();

  Serial.println("");
  Serial.println("Time (millis), Xa, Ya, Za");
}

void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X_out = ( Wire.read()| Wire.read() << 8); // X-axis value
  X_out = X_out/256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  Y_out = ( Wire.read()| Wire.read() << 8); // Y-axis value
  Y_out = Y_out/256;
  Z_out = ( Wire.read()| Wire.read() << 8); // Z-axis value
  Z_out = Z_out/256;

  int current_time = millis();
  float sum = 0.5 * X_out * (current_time - last_time)*(current_time - last_time)/1e6;
  last_time = current_time;

  Serial.println(sum);
  X_accel[2] = X_out*gravity_const;
  Y_accel[2] = Y_out*gravity_const;
  Z_accel[2] = Z_out*gravity_const;
  /*
  Serial.print(X_velocity[0]);
  Serial.print(X_velocity[1]);
  Serial.print(X_velocity[2]);
  Serial.print(",");
  Serial.print(Y_velocity[0]);
  Serial.print(Y_velocity[1]);
  Serial.print(Y_velocity[2]);
  Serial.print(",");
  Serial.print(Z_velocity[0]);
  Serial.print(Z_velocity[1]);
  Serial.println(Z_velocity[2]);
  */

 // === determine displacement === //
  array_shift(X_accel);
  array_shift(Y_accel);
  array_shift(Z_accel);

  determine_velocity(X_accel, X_velocity);
  determine_velocity(Y_accel, Y_velocity);
  determine_velocity(Z_accel, Z_velocity);

  X_Displace = X_Displace + determine_displacement(X_velocity);
  Y_Displace = Y_Displace + determine_displacement(Y_velocity);
  Z_Displace = Z_Displace + determine_displacement(Z_velocity);
  /*
  Serial.print(millis());
  Serial.print(",");
  Serial.print(X_Displace);
  Serial.print(",");
  Serial.print(Y_Displace);
  Serial.print(",");
  Serial.println(Z_Displace);
  */
  delay(1);
}

void array_shift(float list[]) {
  //size of arrays MUST be hardcoded since you can't pass an array into a function
  for (int i = 0; i < 2; i++){
    list[i] = list[i+1];
  }
  return;
}

void determine_velocity(float accel_list[], float velocity_list[]){
  //size of arrays MUST be hardcoded since you can't pass an array into a function
  for (int i = 0; i < 2; i++){
    velocity_list[i] = accel_list[i+1]-accel_list[i];
  }
  return;
}

float determine_displacement(float velocity_list[]){
  float displacement = velocity_list[1]-velocity_list[0];
  return displacement;
}