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
// ==========================================================
//                      Program Variables
// ==========================================================
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
int directionPinR = 12; // Direction Pins
int directionPinL = 13;

int pwmPinR = 3; // PWM Pins
int pwmPinL = 11;

int brakePinR = 9; // Brake Pins
int brakePinL = 8;

bool directionState; //boolean to switch direction (not currently used)

// ==========================================================
//                   Object Declaration
// ========================================================== 

/* Create ASK objects for RF Communication */
RH_ASK rf_driver(2000,19,18,10); // Reciever [Pin 18]
RH_ASK rf_driver1(2000,18,19,10,true); // Transmitter [Pin 19]

/* Servo object */
Servo myServo;  // Create a servo object

/* Compass Object */
QMC5883LCompass compass;

// ==========================================================
//               Program Variable Initialization
// ==========================================================



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
}
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

  duration = 0;
    // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_RIGHT, LOW);

  distR = (duration * 0.34) / 2;

}
int getBearing(){
  compass.read();
  int bearing = compass.getAzimuth();
  //adjust for factory calibration being off by 90 degrees
  bearing = bearing + 270;
  //Adjust for values over 360
  if(bearing>360){bearing=bearing-360;}
  if(bearing<0){bearing = bearing+360;}
  return bearing;
}
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
// Function: Sets Rover Direction Backwards
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

// Function: Sets Rover Direction Forward
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
  else{
    return;
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
void forward(){
    Serial.print("Forward debug");
    setDirFor(); // Set Rover Direction to forward
    setBrakes(false); //release breaks
        //set work duty for the motor (manual control - will remove)
    setDutyRight(250);
    setDutyLeft(250);
    delay(1000);
        //activate breaks
    setBrakes(true);
    //set work duty for the motor to 0 (off)
    setDutyRight(0);
    setDutyLeft(0);
}
// Function: Forward command until wait ends
void forward(int wait){
    int ForwardHeading = getBearing();
    int startTime = millis();
    Serial.print("Forward debug");
    setDirFor(); // Set Rover Direction to forward
    setBrakes(false); //release breaks

    //set work duty for the motor (manual control - will remove)
    setDutyRight(200);
    setDutyLeft(200);

    // if (getBearing() < ForwardHeading){
    //     tread_right = tread_right - 5;
    //     setDutyRight(tread_right);
    //     tread_left = tread_left +5;
    //     setDutyLeft(tread_left);
    // }
    // else if (getBearing() > ForwardHeading){
    //     tread_right = tread_right + 5;
    //     setDutyRight(tread_right);
    //     tread_left = tread_left - 5;
    //     setDutyLeft(tread_left);
    // }
    if (millis() > startTime + wait){
    //activate breaks
    setBrakes(true);
    //set work duty for the motor to 0 (off)
    setDutyRight(0);
    setDutyLeft(0);
    }
}
// Function: Forward command until wait & loops end
void forward(int wait, int loops){
  /*
    New version of "forward" function, modifying by controlling each motor 
    on sinusodal pattern to cause the rover to drive in a wobbly
    fashion, thus increasing the chance of seeing a wall before 
    running into it.
  */
  int i = 0; // Variable for number of loops
  float angle = 0; //this angle is in RADIANS
  setDirFor();
  setBrakes(false); //release breaks

  //set work duty for the motor (manual control - will remove)
  setDutyRight(255);
  setDutyLeft(255);

  while (i<loops){
    float tread_right = (sin(angle+3.14159)+1)*255;
    float tread_left = (sin(angle)+1)*255;
    Serial.print("Forward debug");
    setDutyRight(tread_right);
    setDutyLeft(tread_left);
    delay(wait);

    angle = angle + 3.14159;
    i++;
  }
  //activate breaks
  setBrakes(true);
  //set work duty for the motor to 0 (off)
  setDutyRight(0);
  setDutyLeft(0);
}
// Function: Backward command with acceleration and deceleration
void backward(int wait){
    Serial.print("Backward debug");

    // Set rover direction to backward
    setDirBack();         
    setBrakes(false);     // Release brakes

    // Set initial duty cycles
    setDutyRight(250);
    setDutyLeft(250);

    delay(wait);           // Move backward for 500ms

    // Stop the rover
    setBrakes(true);      // Apply brakes
    setDutyRight(0);      // Cut power
    setDutyLeft(0);
}
// Function: Right turn
void right(){
    Serial.println("RIGHT TURN");
    setDirRight();        // Set motors to turn right (left motor forward, right motor backward)
    setBrakes(false);     // Release brakes
    setDutyRight(200);    // Power for right motor (backward)
    setDutyLeft(200);     // Power for left motor (forward)
    delay(300);           // Duration of the turn
    setBrakes(true);      // Apply brakes
    // setDutyRight(0);      // Stop motors
    // setDutyLeft(0);
}
// Function: Right turn until rover has rotated approximately 90 degrees
void right3(){
    Serial.println("RIGHT TURN");

    int ForwardHeading = getBearing(); // Store the current heading
    setDirRight();                     // Set turning direction (left motor forward, right motor backward)
    setBrakes(false);                  // Release brakes

    setDutyRight(200);                 // Set motor speeds
    setDutyLeft(200);

    // Turn until heading is approx. 90° less than starting heading
    while (getBearing() > ForwardHeading - 90){
        Serial.print("turning");
        delay(5);
    }
    setBrakes(true);                   // Stop movement
    setDutyRight(0);
    setDutyLeft(0);
}
// Function: Left turn
void left(){
    Serial.println("LEFT TURN");
    setDirLeft();         // Set motors to turn left (right motor forward, left motor backward)
    setBrakes(false);     // Release brakes
    setDutyRight(200);    // Power for right motor (forward)
    setDutyLeft(200);     // Power for left motor (backward)
    delay(300);           // Duration of the turn
    setBrakes(true);      // Apply brakes
    setDutyRight(0);      // Stop motors
    setDutyLeft(0);
}
// Function: Left turn until rover has rotated approximately 90 degrees
void left3(){
    Serial.println("LEFT TURN");

    int ForwardHeading = getBearing(); // Store the current heading
    setDirLeft();                      // Set motors for left turn (right forward, left backward)
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
// bool handshakeRF(){
//   if (rf_driver.recv(buf, &buflen)) {
//     buf[buflen] = '\0';  // Null-terminate the received message
//     Serial.print("Received: ");
//     Serial.println((char*)buf);

//     // If handshake message received, respond with "ACK"
//     if (strcmp((char*)buf, "HANDSHAKE") == 0) {
//         Serial.println("Sending ACK...");
//         char ackMsg[] = "ACK";
//         rf_driver1.send((uint8_t *)ackMsg, strlen(ackMsg));
//         rf_driver1.waitPacketSent();
//         return true;
//     }
// }
bool handshakeMotor(){
  //   //change direction every loop()
  // directionState = !directionState;

  // //write a high state to the direction pin (13)
  // else{
    digitalWrite(directionPinR, HIGH); // Sets right tread to forward
    digitalWrite(directionPinL, LOW);  // Sets left tread to forward
  // }

  //release breaks
  digitalWrite(brakePinR, LOW);
  digitalWrite(brakePinL, LOW);

  //set work duty for the motor
  analogWrite(pwmPinR, 255);
  analogWrite(pwmPinL, 255);
  delay(2000);

  //activate breaks
  digitalWrite(brakePinR, HIGH);
  digitalWrite(brakePinL, HIGH);
  //set work duty for the motor to 0 (off)
  analogWrite(pwmPinR, 0);
  analogWrite(pwmPinL, 0);
  delay(1000);
  return true;
}
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
  //payload();
  forward(500);
}
// Function that waits until a valid RF message is received, verifies it, and passes the number to travelDistance()
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
      // Reset the buffer length for the next attempt
      buflen = sizeof(buf);
    }
    delay(100); // Small delay to avoid busy waiting
  }
}
/* Drive in an enclosed area (approx 6’x6’) without contacting walls or obstacles. Mission Time Limit: 60 seconds.*/
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
    if (L_dist < 200 and R_dist < 200){ //5 inches is 200 mm
      Serial.println("U-turn");
      right();
      right();
    }
    //If left sensor sees something, turn right 90 degrees
    else if(L_dist < 200 and R_dist > 200){
      Serial.println("Turn Right");
      right();
    }
    //If Right sensor sees something, rutn left 90 degrees.
    else if(L_dist > 200 and R_dist < 200){
      Serial.println("Turn Left");
      left();
    }
    //If neither sensor sees something, wobble forward
    else if(L_dist > 200 and R_dist > 200){
      Serial.println("Forward");
      forward(10);//This is the alternate forward for wobbling.
    }
  }
}
static const unsigned long TRAVEL_TIME_FOR_30_FEET = 5000UL;  // 
/* 
  Drive forward ~30 feet directly ahead. 
  Demonstrate obstacle avoidance by resuming path to the original waypoint 
  after detouring around a single obstacle. 
  Stop within 5 ft of the waypoint, all under 120 seconds (user requirement).
*/
void mission_22(){
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
        setDirRight();                  // Set motors: right backward, left forward
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
void mission_41(){
}
void mission_42(){
}
// Parses a string command and executes the corresponding movement or mission routine.
void parseCommand(char *message) {
    if (strcmp(message, "FORWARD") == 0) {
      forward(1000);
    }
    else if (strcmp(message, "BACKWARD") == 0) {
      backward(1000);
    }
    else if (strcmp(message, "RIGHT") == 0) {
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
    else if (strcmp(message, "Mission 3A") == 0) {
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
void setup() {
  /* Pin Declaration */
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);

  // Motor Hat Pins
  pinMode(directionPinR, OUTPUT);
  pinMode(pwmPinR, OUTPUT);
  pinMode(brakePinR, OUTPUT);
  pinMode(directionPinL, OUTPUT);
  pinMode(pwmPinL, OUTPUT);
  pinMode(brakePinL, OUTPUT);

  /* Motor handshake */
  // if(handshakeMotor() == true){
  //   Serial.println("Motor Initialize Complete");
  // }

  /* Setup Serial Monitor */
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println("Waiting for message");
  // Initialize ASK Object
  rf_driver.init();
  if (!rf_driver.init()) {
      Serial.println("RF Module Initialization Failed!");
  } else {
      Serial.println("RF Module Initialized.");
      Serial.println("Waiting for messages...");
  }
  myServo.write(pos); // put it before the attach() so it goes straight to that position
  myServo.attach(6);  // Attach the servo to pin 9
  payload();
  /* RF Handshake */
  // if(handshakeRF() == true){
  //   continue;
  //}

  /* Ultrasonic Handshake */
  handshakeUltra();
  //*****COMPASS SETUP*****
  Wire.begin();
  compass.init();
  // compass.setCalibrationOffsets(-471.00, -148.00, -991.00);
  // compass.setCalibrationScales(1.01, 1.21, 0.85);
  compass.setCalibrationOffsets(-193.00, 22.00, -747.00);
  compass.setCalibrationScales(1.13, 0.77, 1.22);
  // compass.read();
  // int heading = compass.getAzimuth();
  int heading = getBearing();
  Serial.print("Heading: ");
  Serial.println(heading);

  // mission_21();
  forward();
}
void loop() {
  /* RF Messaging */
  static uint8_t buf[10] = {0};  
  uint8_t buflen = sizeof(buf);
  
  if (rf_driver.recv(buf, &buflen)) {
      buf[buflen] = '\0';  // Null-terminate the received message
      Serial.print("Received: ");
      Serial.println((char*)buf);

      // Process movement commands
      parseCommand((char*)buf);
      Serial.print("Waiting for next command");
      memset(buf, 0, sizeof(buf));
      delay(50);  // Small delay to prevent CPU overload
  }
}