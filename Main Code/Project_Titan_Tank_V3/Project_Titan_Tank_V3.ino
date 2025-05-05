/*
 * ==========================================================
 * Project Name: Project Titan Hyperion
 * Author: Bryce Keever
 * Date: April 9, 2025
 * Version: 3.0
 * Description: Using Arduino Motor Shield Rev3 Clone
 * ==========================================================
 */

// ==========================================================
//                      Library Includes
// ==========================================================
#include <AFMotor.h>
#include <SPI.h>
// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
#include <Servo.h>  // Include the Servo library
// I2C Library
#include <Wire.h>
// QMC5883L Compass Library
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
// ==========================================================
//                      Program Variables
// ==========================================================
// === Serial and GPS Settings ===
#define GPS_SERIAL Serial3
#define GPS_BAUD 38400
#define PC_BAUD 115200

// === Ultrasonic Variables ===
#define NUM_SAMPLES 5  // Number of readings to average
#define PULSE_TIMEOUT 30000  // Timeout for pulseIn() in microseconds
bool sensorsEnabled = true;  // Boolean flag to enable or disable sensors
const long sensorInterval = 1000;  // 1-second interval

unsigned long previousMillis = 0;
const long interval = 3000; // 3 seconds

int pos = 0; // Initial position for servo

uint16_t tread_right = 250; // Initial speed for 
uint16_t tread_left = 250;
uint8_t i; // For accel and deccel
// === RF Messaging ===
static uint8_t buf[12] = {0};  
uint8_t buflen = sizeof(buf);

// === Mission 4B Variables ===
float startLat = 0;
float startLon = 0;
float targetLat = 0;
float targetLon = 0;

bool waypointReceived = false;
unsigned long lastMessageTime = 0;

// ==========================================================
//                      Pin Declaration
// ==========================================================
/* Ultrasonic Sensor Pins */
#define TRIG_PIN_LEFT 22  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN_LEFT 23 // Pin connected to the Echo pin of the sensor
#define TRIG_PIN_RIGHT 24  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN_RIGHT 25 // Pin connected to the Echo pin of the sensor
#define STARTUP_ERROR 28
#define MAIN_ERROR 29

/* Motor Hat Pins */
const int directionPinR = 12; // Direction Pins
const int directionPinL = 13;

const int pwmPinR = 3; // PWM Pins
const int pwmPinL = 11;

const int brakePinR = 9; // Brake Pins
const int brakePinL = 8;

const bool directionState; //boolean to switch direction (not currently used)

// ==========================================================
//                   Object Declaration
// ========================================================== 

/* Create ASK objects for RF Communication */
RH_ASK rf_driver(2000,19,18,10); // Reciever [Pin 18]
RH_ASK rf_driver1(2000,18,19,10,true); // Transmitter [Pin 19]

/* Servo object */
Servo myServo; 
/* Compass Object */
QMC5883LCompass compass;
/* GPS Object */
TinyGPSPlus gps;
// ==========================================================
//               Program Variable Initialization
// ==========================================================


void checkDist(){/* Ultrasonic Sensor Check */
  long duration, dist_mm, distL, distR;

  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN_LEFT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_LEFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_LEFT, LOW);

  // Measure the echo time
  duration = pulseIn(ECHO_PIN_LEFT, HIGH);

  // Convert time to distance
  distL = (duration * 0.34) / 2;  // Distance in mm

  // Resetting duration variable for reuse
  duration = 0;
  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_RIGHT, LOW);

  distR = (duration * 0.34) / 2;

}
/* 
* Function to return bearing from compass as int.
*/
int getBearing(){
  compass.read();
  int bearing = compass.getAzimuth();
  //adjust for factory calibration being off by 90 degrees
  bearing = bearing + 293+45;
  //Adjust for values over 360
  if(bearing>360){bearing=bearing-360;}
  if(bearing<0){bearing = bearing+360;}
  return 360-bearing;
}
/*
* Function to return measured distance as long int.
*/
long getDistance(int trigPin, int echoPin) {
    long totalDistance, duration, distance;
    int validReadings = 0;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT);

    distance = (duration * 0.34) / 2;
    return distance;
    //     if (distance > 1) continue;  // Ignore out-of-range values
    // for (int i = 0; i < NUM_SAMPLES; i++) {
    //     digitalWrite(trigPin, LOW);
    //     delayMicroseconds(2);
    //     digitalWrite(trigPin, HIGH);
    //     delayMicroseconds(10);
    //     digitalWrite(trigPin, LOW);

    //     duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT);

    //     if (duration == 0) continue;  // Skip invalid readings

    //     distance = (duration * 0.34) / 2;

    //     if (distance > 1) continue;  // Ignore out-of-range values

    //     totalDistance += distance;
    //     validReadings++;
    //     delay(50);
    // }

    //return (validReadings > 0) ? totalDistance / validReadings : -1;  // Return average or error
}
void setDutyRight(int duty){
  analogWrite(pwmPinR,duty);
}
void setDutyLeft(int duty){
  analogWrite(pwmPinL,duty);
}
// Function: Sets Rover Direction Forward
void setDirFor(){
  digitalWrite(directionPinR, HIGH); 
  digitalWrite(directionPinL, LOW);  
}
// Function: Sets Rover Direction backward_debugs
void setDirBack(){
  digitalWrite(directionPinR, LOW); 
  digitalWrite(directionPinL, HIGH);  
}
// Function: Sets Rover Direction Right
void setDirRight(){
  digitalWrite(directionPinR, LOW); 
  digitalWrite(directionPinL, LOW);  
}
// Function: Sets Rover Direction Left
void setDirLeft(){
  digitalWrite(directionPinR, HIGH); 
  digitalWrite(directionPinL, HIGH);  
}

/*
* Function: Sets rover brakes based on passed input (T/F).
*/
void setBrakes(bool state){
  if(state == true){
    //activate brakes
    digitalWrite(brakePinR, HIGH);
    digitalWrite(brakePinL, HIGH);
  }
  else if(state == false){
    //deactivate brakes
    digitalWrite(brakePinR, LOW);
    digitalWrite(brakePinL, LOW);
  }
}
// void accel() {
//   static int i = 0;
//   static unsigned long lastUpdate = 0;

//   if (i < 255 && millis() - lastUpdate > 250) { // Slower acceleration
//     // setDutyRight(i);
//     // setDutyLeft(i);
//     i += 0.1;  // Change to i += 2 for a faster increase
//     lastUpdate = millis();
//   }
// }
// void decel(){
//   static int i = 255;
//   static unsigned long lastUpdate = 0;
  
//   if (i > 0 && millis() - lastUpdate > 5) {
//     // setDutyRight(i);
//     // setDutyLeft(i);
//     i--;
//     lastUpdate = millis();
//   }
// }

/*
* Function to command rover to move forward for hardcoded time delay.
*/
void forward_debug(){
    Serial.print("Forward debug");

    // Set Direction Forward
    digitalWrite(12, LOW); //Establishes forward direction of Channel A
    digitalWrite(13, HIGH);  //Establishes forward direction of Channel B

    // 
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B

    int tread = 255;
    analogWrite(3, tread);   //Spins the motor on Channel A at full speed
    analogWrite(11, tread);    //Spins the motor on Channel B at full speed

    delay(1000);

    digitalWrite(9, HIGH);  //Engage the Brake for Channel A
    digitalWrite(8, HIGH);  //Engage the Brake for Channel B

    analogWrite(3, 0);  
    analogWrite(11, 0);   

}
/*
* Function to command rover to move forward for given time delay.
* Also includes straight line adjustment based on heading value.
*/
void forward(int wait){
    int ForwardHeading = getBearing();
    int startTime = millis();
    Serial.print("Forward debug");
    // Set Direction Forward
    digitalWrite(12, HIGH); //Establishes forward direction of Channel A
    digitalWrite(13, LOW);  //Establishes backward_debug direction of Channel B

    // Disengagese Brakes
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  
    analogWrite(3, 255);   //Spins the motor on Channel A at full speed
    analogWrite(11, 255);    //Spins the motor on Channel B at half speed

    if (getBearing() < ForwardHeading){
        tread_right = tread_right - 5;
        // setDutyRight(tread_right);
        analogWrite(3, tread_right);   // Changes the speed of Channel A
        tread_left = tread_left +5;
        // setDutyLeft(tread_left);
        analogWrite(11, tread_left);    // Changes the speed of Channel B
    }
    else if (getBearing() > ForwardHeading){
        tread_right = tread_right + 5;
        // setDutyRight(tread_right);
        analogWrite(3, tread_right);   // Changes the speed of Channel A
        tread_left = tread_left - 5;
        // setDutyLeft(tread_left);
        analogWrite(11, tread_left);    // Changes the speed of Channel B
    }
    if (millis() > startTime + wait){
      digitalWrite(9, HIGH);  //Engage the Brake for Channel A
      digitalWrite(8, HIGH);  //Engage the Brake for Channel B
    }
}
/*
* Function to command rover to move forward for given delay time and num of loops.
*/
void forward(int wait, int loops){
  /*
    New version of "forward" function, modifying by controlling each motor 
    on sinusodal pattern to cause the rover to drive in a wobbly
    fashion, thus increasing the chance of seeing a wall before 
    running into it.
  */
  int i = 0; // Variable for number of loops
  float angle = 0; //this angle is in RADIANS
  // setDirFor();
  // setBrakes(false); //release breaks
  // setDutyRight(255);
  // setDutyLeft(255);

  // Set Direction Forward
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(13, LOW);  //Establishes backward_debug direction of Channel B
  // Disengagese Brakes
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B

  while (i<loops){
    float tread_right = (sin(angle+3.14159)+1)*255;
    float tread_left = (sin(angle)+1)*255;
    Serial.print("Forward debug");
    // setDutyRight(tread_right);
    // setDutyLeft(tread_left);
    analogWrite(3, tread_right);   // Changes the speed of Channel A
    analogWrite(11, tread_left);    // Changes the speed of Channel B
    delay(wait);

    angle = angle + 3.14159;
    i++;
  }
  //activate breaks
  // setBrakes(true);
  // //set work duty for the motor to 0 (off)
  // setDutyRight(0);
  // setDutyLeft(0);
  digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  digitalWrite(8, HIGH);  //Engage the Brake for Channel B
}
/*
* Mission Function: Move rover forward.
*/
void Forward(){
    // Set Direction Forward
    digitalWrite(12, LOW); //Establishes forward direction of Channel A
    digitalWrite(13, HIGH);  //Establishes forward direction of Channel B

    // 
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B

    int tread = 255;
    analogWrite(3, tread);   //Spins the motor on Channel A at full speed
    analogWrite(11, tread);    //Spins the motor on Channel B at full speed
    delay(1000);
}
/*
* Function to command rover to move backward for hardcoded time delay.
*/
void backward_debug(){
    Serial.print("backward debug");

    // Set Direction Forward
    digitalWrite(12, HIGH); //Establishes forward direction of Channel A
    digitalWrite(13, LOW);  //Establishes forward direction of Channel B

    // 
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B

    int tread = 255;
    analogWrite(3, tread);   //Spins the motor on Channel A at full speed
    analogWrite(11, tread);    //Spins the motor on Channel B at full speed

    delay(1000);

    digitalWrite(9, HIGH);  //Engage the Brake for Channel A
    digitalWrite(8, HIGH);  //Engage the Brake for Channel B

    analogWrite(3, 0);  
    analogWrite(11, 0);  
}
/*
* Mission Function: Move rover backward.
*/
void Backward(){

}
/*
* Function to command rover to turn to the right 90 degrees.
*/
void right(){
    Serial.println("RIGHT TURN");
      //Motor A forward @ full speed
    digitalWrite(12, LOW); //Establishes forward direction of Channel A
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    analogWrite(3, 255);   //Spins the motor on Channel A at full speed

    //Motor B backward_debug @ half speed
    digitalWrite(13, LOW);  //Establishes backward_debug direction of Channel B
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B
    analogWrite(11, 255);    //Spins the motor on Channel B at half speed

    delay(150);

    digitalWrite(9, HIGH);  //Engage the Brake for Channel A
    digitalWrite(8, HIGH);  //Engage the Brake for Channel B

    analogWrite(3, 0);  
    analogWrite(11, 0);  
    // setDirRight();        // Set motors to turn right (left motor forward, right motor backward_debug)
    // setBrakes(false);     // Release brakes
    // setDutyRight(200);    // Power for right motor (backward_debug)
    // setDutyLeft(200);     // Power for left motor (forward)
    // delay(300);           // Duration of the turn
    // setBrakes(true);      // Apply brakes
    // setDutyRight(0);      // Stop motors
    // setDutyLeft(0);
}
/*
* Function to command rover to turn to the right 90 degrees.
* Includes heading value to turn right with increaed accuracy.
*/
void right3(){
    Serial.println("RIGHT TURN");

    int ForwardHeading = getBearing(); // Store the current heading
    digitalWrite(12, LOW); //Establishes forward direction of Channel A
    digitalWrite(13, LOW);
  

    // Set Duty of both motors
    analogWrite(3, 255);  
    analogWrite(11, 255);    
    // Turn until heading is approx. 90° less than starting heading
    while (getBearing() > ForwardHeading - 90){
        Serial.print("turning");
        delay(5);
    }
    digitalWrite(9, HIGH);  //Engage the Brake for Channel A
    digitalWrite(8, HIGH);  //Engage the Brake for Channel B

    setDutyRight(0);
    setDutyLeft(0);
}
/*
* Function to command rover to turn to the left 90 degrees.
*/
void left(){
    Serial.println("LEFT TURN");
    //Motor A forward @ full speed
    digitalWrite(12, HIGH); //Establishes forward direction of Channel A
    digitalWrite(9, LOW);   //Disengage the Brake for Channel A
    analogWrite(3, 255);   //Spins the motor on Channel A at full speed

    //Motor B backward_debug @ half speed
    digitalWrite(13, HIGH);  //Establishes backward_debug direction of Channel B
    digitalWrite(8, LOW);   //Disengage the Brake for Channel B
    analogWrite(11, 255);    //Spins the motor on Channel B at half speed

    delay(150);

    digitalWrite(9, HIGH);  //Engage the Brake for Channel A
    digitalWrite(8, HIGH);  //Engage the Brake for Channel B

    analogWrite(3, 0);  
    analogWrite(11, 0);  
    // setDirLeft();         // Set motors to turn left (right motor forward, left motor backward_debug)
    // setBrakes(false);     // Release brakes
    // setDutyRight(200);    // Power for right motor (forward)
    // setDutyLeft(200);     // Power for left motor (backward_debug)
    // delay(300);           // Duration of the turn
    // setBrakes(true);      // Apply brakes
    // setDutyRight(0);      // Stop motors
    // setDutyLeft(0);
}
/*
* Function: Left turn until rover has rotated approximately 90 degrees.
* Includes heading value to turn right with increaed accuracy.
*/
void left3(){
    Serial.println("LEFT TURN");

    int ForwardHeading = getBearing(); // Store the current heading
    setDirLeft();                      // Set motors for left turn (right forward, left backward_debug)
    setBrakes(false);                 // Release brakes

    setDutyRight(200);                // Power both motors
    setDutyLeft(200);

    // Turn until heading
    while (getBearing() < ForwardHeading + 90){
        Serial.print("turning");
        delay(5);
    }
    setBrakes(true);                 // Stop the rover
    setDutyRight(0);
    setDutyLeft(0);
}
/*
* Payload Function: Moves swing arm with servo to ejecto the payload.
*/
void payload(){
  int pos = 0;    // variable to store the servo position
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(2);                       // waits 15ms for the servo to reach the position
  }
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
    myServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15ms for the servo to reach the position
  }
}
/*
* Function to return distance. Used in previous missions to calculate distance per time.
*/
void travelDistance(int distance){
  // Given that 10 feet takes 4000ms, calculate delay as 400ms per foot - changed to 4.77/ft
  int delayTime = distance * 380;
  Serial.print("Traveling ");
  Serial.print(distance);
  Serial.print(" feet (");
  Serial.print(delayTime);
  Serial.println(" ms).");
  forward(delayTime);
  delay(1000);
  payload();
  delay(500);
  forward(500);
}
/*
* Handshake Function: Verifies RF communication is working.
*/
bool handshakeRF(){
  if (rf_driver.recv(buf, &buflen)) {
    buf[buflen] = '\0';  // Null-terminate the received message
    Serial.print("Received: ");
    Serial.println((char*)buf);

    // If handshake message received, respond with "ACK"
    if (strcmp((char*)buf, "HANDSHAKE") == 0) {
        Serial.println("Sending ACK...");
        char ackMsg[] = "ACK";
        rf_driver1.send((uint8_t *)ackMsg, strlen(ackMsg));
        rf_driver1.waitPacketSent();
        return true;
    }
  }
}
/* 
* Handshake function: Verifies functionality of both motors, direction setting and brakes. 
*/
bool handshakeMotor(){
  // Set Direction Forward
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(13, LOW);  //Establishes backward_debug direction of Channel B
  //release breaks
  digitalWrite(brakePinR, LOW);
  digitalWrite(brakePinL, LOW);
  //set work duty for the motor
  analogWrite(pwmPinR, 255);
  analogWrite(pwmPinL, 255);
  delay(500);
  //activate breaks
  digitalWrite(brakePinR, HIGH);
  digitalWrite(brakePinL, HIGH);
  // Set Direction backward_debug
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(13, LOW);  //Establishes backward_debug direction of Channel B
  //set work duty for the motor
  analogWrite(pwmPinR, 255);
  analogWrite(pwmPinL, 255);
  delay(500);
  //activate breaks
  digitalWrite(brakePinR, HIGH);
  digitalWrite(brakePinL, HIGH);
  //set work duty for the motor to 0 (off)
  analogWrite(pwmPinR, 0);
  analogWrite(pwmPinL, 0);
  delay(1000);
  return true;
}
/* 
* Handshake function: Verifies functionality of GPS and GPS-Compass.
*/

/* 
* Handshake function: Verifies functionality of both Ultrasonic distance sensors.
*/
bool handshakeUltra(){
  long duration, distL, distR;

  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN_LEFT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_LEFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_LEFT, LOW);

  // Measure the echo time
  duration = pulseIn(ECHO_PIN_LEFT, HIGH);

  // Added delay for reading accuracy
  delay(100);
  
  // Convert time to distance
  distL = (duration * 0.34) / 2;  // Distance in mm
  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_RIGHT, LOW);
  // Measure the echo time
  duration = pulseIn(ECHO_PIN_RIGHT, HIGH);
  distR = (duration * 0.343) / 2;
  Serial.print("Distance Left:"); Serial.println(distL);
  Serial.print("Distance Right:"); Serial.println(distR);
  return true;
}
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    lat1 = radians(lat1);
    lon1 = radians(lon1);
    lat2 = radians(lat2);
    lon2 = radians(lon2);

    float dLon = lon2 - lon1;
    float x = sin(dLon) * cos(lat2);
    float y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    float bearing = atan2(x, y);
    bearing = degrees(bearing);
    bearing = fmod((bearing + 360.0), 360.0); // Normalize to 0-360
    return bearing;
}

float headingError(float currentHeading, float targetBearing) {
    float error = targetBearing - currentHeading;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    return error;
}
void driveTowardsTarget(float currLat, float currLon, float destLat, float destLon) {
  Serial.print("Driving towards target");
    compass.read();
    float bearing = getBearing();
    // float currentHeading = compass.getAzimuth();  // Compass heading in degrees
    float currentHeading = bearing;  // Compass heading in degrees
    float targetBearing = calculateBearing(currLat, currLon, destLat, destLon);
    float error = headingError(currentHeading, targetBearing);

    // Serial.print("Current Heading: ");
    // Serial.println(currentHeading);
    // Serial.print("Target Bearing: ");
    // Serial.println(targetBearing);
    // Serial.print("Heading Error: ");
    // Serial.println(error);

    if (abs(error) < 20) {
        Serial.println("Moving Forward");
        Forward();
    } else if (error > 0) {
        Serial.println("Turning Right");
        right();
        // left();
    } else {
        Serial.println("Turning Left");
        left();
        // right();
    }
}
float distanceToTarget(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000; // Earth's radius in meters
    lat1 = radians(lat1);
    lon1 = radians(lon1);
    lat2 = radians(lat2);
    lon2 = radians(lon2);

    float dLat = lat2 - lat1;
    float dLon = lon2 - lon1;

    float a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1) * cos(lat2) *
              sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));

    return R * c;
}

/*
* Mission 1A Function: Drives in a 10'x10' square path.
*/
void mission_11(){
  int counter = 0;  // Initialize counter
  while (counter < 3) {  // Run while counter is less than 4
    Serial.print("Counter: ");
    Serial.println(counter);
    forward(3800);
    delay(500);
    //right();
    counter++;  // Increment counter
    delay(1000);
  }
  forward(3800);
  delay(500);
  payload();
  forward(500);
}
/*
* Misson 1B Function: Waits for distance command and then travels distance 
*/
void mission_12(){
  uint8_t buf[13] = {0};
  uint8_t buflen = sizeof(buf);
  Serial.print("Starting Mission 1B");
  // Loop continuously until a valid message is received
  while (true) {
    // Check if a message has been received
    if (rf_driver.recv(buf, &buflen)) {
      // Check if the message starts with 'X'
      if (buf[0] == 'X') {
        // Convert the subsequent characters into an integer
        int receivedNum = atoi((char*)&buf[1]);
        Serial.print("Message Received: ");
        Serial.println((char*)buf);
        // Pass the parsed number to the travelDistance function
        travelDistance(receivedNum);
        break; // Exit the loop once a valid message is processed
      } else {
        Serial.println("Invalid message format received.");
      }
      buflen = sizeof(buf);      // Reset the buffer length for the next attempt
    }
    delay(100); // Small delay to avoid busy waiting
  }
}
/* 
* Mission 2A Function: Drive in an enclosed area (approx 6’x6’) without contacting walls or obstacles. 
* Mission Time Limit: 60 seconds.
*/
void mission_21(){
  Serial.println("RUNNING MISSION 2A");
  long L_dist, R_dist;
  while (true){
    //check both distance sensors to see if there's a wall.
    L_dist = getDistance(TRIG_PIN_LEFT,ECHO_PIN_LEFT);
    R_dist = getDistance(TRIG_PIN_RIGHT,ECHO_PIN_RIGHT);
    Serial.print("Left: ");
    Serial.print(L_dist);
    Serial.print("     Right: ");
    Serial.println(R_dist);

    //If both sensors detect that we're close to a wall, U-turn
    if (L_dist < 400 and R_dist < 400){ //5 inches is 200 mm
      Serial.println("U-turn");
      digitalWrite(9, HIGH);  //Engage the Brake for Channel A
      digitalWrite(8, HIGH);  //Engage the Brake for Channel B
      right();
    }
    //If left sensor sees something, turn right 90 degrees
    else if(L_dist < 400 and R_dist > 400){
      Serial.println("Turn Right");
      digitalWrite(9, HIGH);  //Engage the Brake for Channel A
      digitalWrite(8, HIGH);  //Engage the Brake for Channel B
      right();
    }
    //If Right sensor sees something, rutn left 90 degrees.
    else if(L_dist > 400 and R_dist < 400){
      Serial.println("Turn Left");
      digitalWrite(9, HIGH);  //Engage the Brake for Channel A
      digitalWrite(8, HIGH);  //Engage the Brake for Channel B
      left();
    }
    //If neither sensor sees something, wobble forward
    else if(L_dist > 400 and R_dist > 400){
      Serial.println("Forward");
      digitalWrite(9, HIGH);  //Engage the Brake for Channel A
      digitalWrite(8, HIGH);  //Engage the Brake for Channel B
      forward_debug();//This is the alternate forward for wobbling.
    }
  }
}
/* 
* Mission 2B Function: Drive 30ft with obstacle avoidance. 
*/
void mission_22(){
  /* 
  Drive forward ~30 feet directly ahead. 
  Demonstrate obstacle avoidance by resuming path to the original waypoint 
  after detouring around a single obstacle. 
  Stop within 5 ft of the waypoint, all under 120 seconds (user requirement).
*/
  static const unsigned long TRAVEL_TIME_FOR_30_FEET = 5000UL;  // 
  compass.read();  // Read initial heading
  int startHeading = compass.getAzimuth();

  // Setup motor speeds
  setDirFor();      
  setBrakes(false);                 // Release brakes

  setDutyRight(200);                // Power both motors
  setDutyLeft(200);

  // Track times
  unsigned long startTime     = millis();
  unsigned long obstacleTime  = 0; 
  unsigned long secondStart   = 0; 
  // We'll use 'flag' to indicate if we've already performed obstacle avoidance
  int flag = 0;

  while (true)
  {
    // Continuously read compass and distance sensors
    compass.read();
    int currentHeading = compass.getAzimuth();
    long L_dist = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    long R_dist = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

    // Basic heading correction: If off course, adjust motor speeds
    if (currentHeading < startHeading) {
        tread_left += 5;
        setDutyLeft(tread_left);
    }
    else if (currentHeading > startHeading) {
        tread_left -= 5;
        setDutyLeft(tread_left);
    }

    // Detect obstacle if either side sensor < 5 inches (200mm) 
    if ((L_dist < 200) || (R_dist < 200)) {
      // If we haven't yet performed avoidance
      if (flag == 0) {
        obstacleTime = millis(); 
        unsigned long traveledTime = (obstacleTime - startTime);

        // Stop motors while we do avoidance maneuvers
        setBrakes(true);

        // Example obstacle-avoidance sequence:
        left();
        forward(500);   // Move forward (500 ms) after turning left
        right();

        // Mark that we’ve already done our single obstacle detour
        flag = 1;

        // Optionally re-read heading if it changed significantly
        compass.read();
        startHeading = compass.getAzimuth(); 
      }
    }

    // Begin movement toward the final waypoint
    secondStart = millis();        // Record the current time as the new movement start
    setDirFor();                   // Set both motors to move forward
    setBrakes(false);              // Release the brakes to allow movement
    setDutyRight(200);             // Apply power to the right motor
    setDutyLeft(200);              // Apply power to the left motor

    // Calculate how long we traveled before the obstacle 
    // (so we only complete the "remaining" distance)
    unsigned long traveledSoFar = (obstacleTime > 0) ? (obstacleTime - startTime) : 0;

    // Continue driving until we’ve covered total “30 ft” time
    while (traveledSoFar + (millis() - secondStart) < TRAVEL_TIME_FOR_30_FEET) 
    {
      // Continuously correct heading inside this inner loop
      compass.read();
      currentHeading = compass.getAzimuth();

    if (currentHeading < startHeading) {
        tread_left += 5;
        setDutyLeft(tread_left);
    }
    else if (currentHeading > startHeading) {
        tread_left -= 5;
        setDutyLeft(tread_left);
    }

      // Optionally check sensors again in here 
    }

    setBrakes(true);                 // Stop the rover
    setDutyRight(0);
    setDutyLeft(0);
    // End this mission
    break;
  }
}
/*
* Mission 3 Function: Turn rover until pointed North.
*/
void mission_3(){
    int i = 10;

    while (true) {
        compass.read();                 // Update compass data
        i = compass.getAzimuth();       // Get current azimuth (heading)

        Serial.print("m3: ");
        Serial.println(i);

        // Check if rover is pointing approximately North (within 35–55 degrees)
        if (i > 35 && i < 55) {
            Serial.println("Mission 3 complete");
            break;
        }

        // Begin turning right in place to reorient
        setDirRight();                  // Set motors: right backward_debug, left forward
        setBrakes(false);              // Release brakes

        setDutyRight(250);             // Set speed for right motor
        setDutyLeft(250);              // Set speed for left motor

        delay(200);                    // Turn for 200ms before checking heading again
    }

    // Stop motors and perform final tasks
    setBrakes(true);                   // Engage brakes
    setDutyRight(0);                   // Stop right motor
    setDutyLeft(0);                    // Stop left motor
    payload();                         // Custom function: possibly for deploying or logging payload?
}
/*
* Mission 4A Function: .
*/
void mission_41() {
  Serial.println("Starting Mission 4A");

  bool missionComplete = false;

  while (!missionComplete) {
    // Continuously read GPS serial data!
    while (GPS_SERIAL.available() > 0) {
      gps.encode(GPS_SERIAL.read());
    }

    // Now we can check if GPS has an updated fix
    if (gps.location.isUpdated()) {
      float currLat = gps.location.lat();
      float currLon = gps.location.lng();

      float destLat = 39.08111;   // Example destination
      float destLon = -108.55949; // Example destination

      driveTowardsTarget(currLat, currLon, destLat, destLon);

      // OPTIONAL: you can add a distance check here!
      float distance = distanceToTarget(currLat, currLon, destLat, destLon);
      Serial.println(distance);
      if (distance < 2.0) { // Example: 2 meters
        Serial.println("Destination Reached!");
        //activate breaks
        digitalWrite(brakePinR, HIGH);
        digitalWrite(brakePinL, HIGH);
        //set work duty for the motor to 0 (off)
        analogWrite(pwmPinR, 0);
        analogWrite(pwmPinL, 0);
        missionComplete = true;
      }
    }
  }
}

/*
* Mission 4B Function: .
*/
bool parseWaypointMessage(char* msg, float& lat, float& lon) {
  if (strlen(msg) != 12 || msg[0] != 'N' || msg[6] != 'W') return false;

  char latPart[6] = {0};
  char lonPart[6] = {0};

  strncpy(latPart, msg + 1, 5);  // e.g. "09495"
  strncpy(lonPart, msg + 7, 5);  // e.g. "58784"

  lat = 39.0 + atof(latPart) / 100000.0;
  lon = -108.0 - atof(lonPart) / 100000.0;

  return true;
}

void mission_42() {
  Serial.println("Starting Mission 4B - Waypoint Navigation & Return");
  float currLat = gps.location.lat();
  float currLon = gps.location.lng();
  // Step 1: Wait for valid RF message
  while (!waypointReceived) {
    if (rf_driver.recv(buf, &buflen)) {
      buf[buflen] = '\0';
      Serial.print("Received RF: ");
      Serial.println((char*)buf);

      if (parseWaypointMessage((char*)buf, targetLat, targetLon)) {
        waypointReceived = true;
        Serial.print("Parsed Target: ");
        Serial.print(targetLat, 6); Serial.print(", ");
        Serial.println(targetLon, 6);
      } else {
        Serial.println("Invalid RF format");
      }
    }
    delay(100);
  }
  // Step 2: Wait for valid GPS fix
  while (!gps.location.isValid()) {
    while (GPS_SERIAL.available() > 0) {
      gps.encode(GPS_SERIAL.read());
    }
    delay(100);
  }

  // Save starting position
  startLat = gps.location.lat();
  startLon = gps.location.lng();
  Serial.print("Start Position: ");
  Serial.print(startLat, 6); Serial.print(", ");
  Serial.println(startLon, 6);

  // Step 3: Drive to waypoint
  bool Step3 = false;
  while(Step3 == false){
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
      }
    if (gps.location.isUpdated()) {
      float currLat = gps.location.lat();
      float currLon = gps.location.lng();

      driveTowardsTarget(currLat, currLon, targetLat, targetLon);
    }
    // OPTIONAL: you can add a distance check here!
    float distance = distanceToTarget(currLat, currLon, targetLat, targetLon);
    Serial.println(distance);
    if (distance < 2.0) { // Example: 2 meters
      Serial.println("Destination Reached!");
      //activate breaks
      digitalWrite(brakePinR, HIGH);
      digitalWrite(brakePinL, HIGH);
      //set work duty for the motor to 0 (off)
      analogWrite(pwmPinR, 0);
      analogWrite(pwmPinL, 0);
    }
  }

  // Step 4: Drop payload/marker
  Serial.println("Arrived at waypoint. Deploying payload...");
  payload();
  delay(500);

  // Step 5: Return to start
  Serial.println("Returning to start location...");
  while (distanceToTarget(gps.location.lat(), gps.location.lng(), startLat, startLon) > 4.5) {
    while (GPS_SERIAL.available() > 0) {
      gps.encode(GPS_SERIAL.read());
    }
    driveTowardsTarget(gps.location.lat(), gps.location.lng(), startLat, startLon);
    delay(100);
  }

  Serial.println("Returned to start position. Mission Complete.");

  // Final stop
  setBrakes(true);
  setDutyRight(0);
  setDutyLeft(0);
}

/*
* Parses a string command and executes the corresponding movement or mission routine.
*/
void parseCommand(char *message) {
    if (strcmp(message, "FORW") == 0) {
      forward_debug();
    }
    else if (strcmp(message, "BACK") == 0) {
      backward_debug();
    }
    else if (strcmp(message, "RIGH") == 0) {
      right();
    }
    else if (strcmp(message, "LEFT") == 0) {
      left();
    }
    else if (strcmp(message, "STOP") == 0) {
      setBrakes(true);
    }
    else if (strcmp(message, "Mission 1A") == 0) {
      mission_11();
    }  
    else if (strcmp(message, "Mission 1B") == 0) {
      mission_12();
    }
    else if (strcmp(message, "Mission 2A") == 0) {
      mission_21();
    }
    else if (strcmp(message, "Mission 2B") == 0) {
      mission_22();
    }
    else if (strcmp(message, "Mission 3B") == 0) {
      mission_3();
    }
    else if (strcmp(message, "Mission 4A") == 0) {
      mission_41();
    }
    else if (strcmp(message, "Mission 4B") == 0) {
      mission_42();
    }
    else{
      return;
    }
}
bool handshakeGPS() {
    bool gpsReady = false;

    // Continuously read GPS serial data
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }

    // Only act when we get a valid GPS update
    if (gps.location.isUpdated()) {
        gpsReady = true; // handshake success!

        Serial.println("===============================");

        // --- GPS Data ---
        if (gps.date.isValid() && gps.time.isValid()) {
            Serial.print("Date: ");
            Serial.print(gps.date.year());
            Serial.print("-");
            Serial.print(gps.date.month());
            Serial.print("-");
            Serial.print(gps.date.day());
            Serial.print("  Time: ");
            Serial.print(gps.time.hour());
            Serial.print(":");
            Serial.print(gps.time.minute());
            Serial.print(":");
            Serial.println(gps.time.second());
        } else {
            Serial.println("Date/Time: Not Available");
        }

        if (gps.location.isValid()) {
            Serial.print("Latitude: ");
            Serial.print(gps.location.lat(), 6);
            Serial.println(gps.location.rawLat().negative ? " S" : " N");

            Serial.print("Longitude: ");
            Serial.print(gps.location.lng(), 6);
            Serial.println(gps.location.rawLng().negative ? " W" : " E");
        } else {
            Serial.println("Location: Not Available");
        }

        if (gps.altitude.isValid()) {
            Serial.print("Altitude: ");
            Serial.print(gps.altitude.meters());
            Serial.println(" m");
        } else {
            Serial.println("Altitude: Not Available");
        }

        if (gps.speed.isValid()) {
            Serial.print("Speed: ");
            Serial.print(gps.speed.kmph(), 2);
            Serial.println(" km/h");
        } else {
            Serial.println("Speed: Not Available");
        }

        if (gps.course.isValid()) {
            Serial.print("GPS Course: ");
            Serial.print(gps.course.deg(), 2);
            Serial.println("°");
        } else {
            Serial.println("GPS Course: Not Available");
        }

        if (gps.satellites.isValid()) {
            Serial.print("Satellites: ");
            Serial.println(gps.satellites.value());
        } else {
            Serial.println("Satellites: Not Available");
        }

        // --- Compass Data ---
        compass.read();
        int heading = compass.getAzimuth();
        int x = compass.getX();
        int y = compass.getY();
        int z = compass.getZ();

        Serial.println("--- Compass Data ---");
        Serial.print("Heading: ");
        Serial.print(heading);
        Serial.println("°");

        Serial.print("Raw X: ");
        Serial.print(x);
        Serial.print("  Y: ");
        Serial.print(y);
        Serial.print("  Z: ");
        Serial.println(z);

        Serial.println("===============================");
        Serial.println();
    }

    return gpsReady; // return handshake result
}

void setup() {
  // Ultrasonic Sensor Setup
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);

  // Motor Shield Setup
  pinMode(directionPinR, OUTPUT); // Right Tread
  pinMode(pwmPinR, OUTPUT);
  pinMode(brakePinR, OUTPUT);

  pinMode(directionPinL, OUTPUT); // Left Tread
  pinMode(pwmPinL, OUTPUT);
  pinMode(brakePinL, OUTPUT);

  /* Setup Serial Monitor */
  Serial.begin(PC_BAUD);
  GPS_SERIAL.begin(GPS_BAUD);
  Wire.begin(); // Start I2C for compass
  /* Initialize RF ASK Object */
  rf_driver.init();
  if (!rf_driver.init()) {
      Serial.println("RF Module Initialization Failed!");
  } else {
      Serial.println("RF Module Initialized.");
      Serial.println("Waiting for messages...");
  }
  
  /* COMPASS SETUP */
  compass.init();
  compass.setCalibrationOffsets(-289.00, -666.00, 965.00);
  compass.setCalibrationScales(1.72, 1.01, 0.70);
  // compass.setCalibrationOffsets(-193.00, 22.00, -747.00);
  // compass.setCalibrationScales(1.13, 0.77, 1.22);
  // compass.read();
  int heading = compass.getAzimuth();
  Serial.print("Heading: ");
  Serial.println(heading);
  int heading2 = getBearing();
  Serial.print("Heading2: ");
  Serial.println(heading2);
  /* RF Verfication */
  // handshakeRF();
  /* Ultrasonic Handhshake */
  handshakeUltra();
  /* GPS Verification */
  handshakeGPS();
  // Read GPS data
  // while(true){
  //   while (GPS_SERIAL.available() > 0) {
  //       gps.encode(GPS_SERIAL.read());
  //   }

  //   if (gps.location.isUpdated()) {
  //       Serial.println("===============================");

  //       // --- GPS Data ---
  //       // Date and Time
  //       if (gps.date.isValid() && gps.time.isValid()) {
  //           Serial.print("Date: ");
  //           Serial.print(gps.date.year());
  //           Serial.print("-");
  //           Serial.print(gps.date.month());
  //           Serial.print("-");
  //           Serial.print(gps.date.day());
  //           Serial.print("  Time: ");
  //           Serial.print(gps.time.hour());
  //           Serial.print(":");
  //           Serial.print(gps.time.minute());
  //           Serial.print(":");
  //           Serial.println(gps.time.second());
  //       } else {
  //           Serial.println("Date/Time: Not Available");
  //       }

  //       // Location
  //       if (gps.location.isValid()) {
  //           Serial.print("Latitude: ");
  //           Serial.print(gps.location.lat(), 6);
  //           Serial.println(gps.location.rawLat().negative ? " S" : " N");

  //           Serial.print("Longitude: ");
  //           Serial.print(gps.location.lng(), 6);
  //           Serial.println(gps.location.rawLng().negative ? " W" : " E");
  //       } else {
  //           Serial.println("Location: Not Available");
  //       }

  //       // Altitude
  //       if (gps.altitude.isValid()) {
  //           Serial.print("Altitude: ");
  //           Serial.print(gps.altitude.meters());
  //           Serial.println(" m");
  //       } else {
  //           Serial.println("Altitude: Not Available");
  //       }

  //       // Speed
  //       if (gps.speed.isValid()) {
  //           Serial.print("Speed: ");
  //           Serial.print(gps.speed.kmph(), 2);
  //           Serial.println(" km/h");
  //       } else {
  //           Serial.println("Speed: Not Available");
  //       }

  //       // Course (GPS heading)
  //       if (gps.course.isValid()) {
  //           Serial.print("GPS Course: ");
  //           Serial.print(gps.course.deg(), 2);
  //           Serial.println("°");
  //       } else {
  //           Serial.println("GPS Course: Not Available");
  //       }

  //       // Satellites
  //       if (gps.satellites.isValid()) {
  //           Serial.print("Satellites: ");
  //           Serial.println(gps.satellites.value());
  //       } else {
  //           Serial.println("Satellites: Not Available");
  //       }
  //     // --- Compass Data ---
  //       compass.read();

  //       int heading = compass.getAzimuth();
  //       int x = compass.getX();
  //       int y = compass.getY();
  //       int z = compass.getZ();

  //       Serial.println("--- Compass Data ---");
  //       Serial.print("Heading: ");
  //       Serial.print(heading);
  //       Serial.println("°");

  //       Serial.print("Raw X: ");
  //       Serial.print(x);
  //       Serial.print("  Y: ");
  //       Serial.print(y);
  //       Serial.print("  Z: ");
  //       Serial.println(z);

  //       Serial.println("===============================");
  //       Serial.println();

  //       delay(500); // Update rate
  //       Serial.println();
  //       break;
  //   }
  //}
  /* DEBUG & TESTING FUNCTIONS BELOW THIS */
  mission_42();

}
void loop() {
  // int heading = compass.getAzimuth();
  // Serial.print("Heading: ");
  // Serial.println(heading-23);
  // int heading2 = getBearing();
  // Serial.print("Heading2: ");
  // Serial.println(heading2);
  // if (rf_driver.recv(buf, &buflen)) {
  //     buf[buflen] = '\0';  // Null-terminate the received message
  //     Serial.print("Received: ");
  //     Serial.println((char*)buf);
  //     // Process movement commands
  //     parseCommand((char*)buf);
  //     Serial.print("Waiting for next command");
  //     memset(buf, 0, sizeof(buf));
  //     delay(50);  // Small delay to prevent CPU overload
  // }
  // delay(500);
}
