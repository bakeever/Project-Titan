/*
 * ==========================================================
 * Project Name: Project Titan Hyperion
 * Author: Bryce Keever
 * Date: January 27, 2025
 * Version: 1.0
 * Description: 
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

int pos = 0;


// ==========================================================
//                      Pin Declaration
// ==========================================================
#define TRIG_PIN_LEFT 22  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN_LEFT 23 // Pin connected to the Echo pin of the sensor
#define TRIG_PIN_RIGHT 24  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN_RIGHT 25 // Pin connected to the Echo pin of the sensor

#define STARTUP_ERROR 28
#define MAIN_ERROR 29

// ==========================================================
//                      Object Declaration
// ==========================================================
AF_DCMotor m1(2); // create motor #1, Right Side
AF_DCMotor m2(3); // create motor #2, Left Side

// Create ASK objects for RF Communication
RH_ASK rf_driver(2000,19,18,10); // Reciever
RH_ASK rf_driver1(2000,18,19,10,true); // Transmitter

Servo myServo;  // Create a servo object

QMC5883LCompass compass;
// ==========================================================
//                  Initialization Section
// ==========================================================
uint16_t outer_tread = 250;
uint16_t inner_tread = 250;
uint8_t i; // For accel and deccel

// long distanceLeft = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
// long distanceRight = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

void handshakeRF(){
  // if (rf_driver.recv(buf, &buflen)) {
  //   buf[buflen] = '\0';  // Null-terminate the received message
  //   Serial.print("Received: ");
  //   Serial.println((char*)buf);

  //   // If handshake message received, respond with "ACK"
  //   if (strcmp((char*)buf, "HANDSHAKE") == 0) {
  //       Serial.println("Sending ACK...");
  //       char ackMsg[] = "ACK";
  //       rf_driver1.send((uint8_t *)ackMsg, strlen(ackMsg));
  //       rf_driver1.waitPacketSent();
  //       return true;
  //   }
}
bool handshakeUltra(){
  bool flagLeft = false;
  bool flagRight = false;
  if (sensorsEnabled) {  // Check if sensors are enabled
    long durationL, durationR, distL, distR;

    // Trigger the ultrasonic sensor - left
    digitalWrite(TRIG_PIN_LEFT, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_LEFT, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_LEFT, LOW);

    durationL = pulseIn(ECHO_PIN_LEFT, HIGH); // Measure the echo time
    distL = (durationL * 0.34) / 2;  // Distance in mm - converted from time

    if (distL >= 0) {
        Serial.print("Stable Left Side Distance: ");
        Serial.print(distL);
        Serial.println(" mm");
        flagLeft = true;
    }
    // Trigger the ultrasonic sensor - right
    digitalWrite(TRIG_PIN_RIGHT, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_RIGHT, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_RIGHT, LOW);

    durationR = pulseIn(ECHO_PIN_RIGHT, HIGH); // Measure the echo time
    distR = (durationR * 0.34) / 2;  // Distance in mm - converted from time
    if (distR >= 0) {
        Serial.print("Stable Right Side Distance: ");
        Serial.print(distR);
        Serial.println(" mm");
        flagRight = true;
    }
    // if(flagLeft == true || flagRight == true){
    //   return true;
    // }

  } 
  else {
    Serial.println("Sensors are disabled.");
    return false;
  }
}
bool handshakeMotor(){
  // Rotate Left 
  m1.setSpeed(150);
  m2.setSpeed(150);
  m1.run(FORWARD);
  m2.run(FORWARD);
  delay(1000);
  m1.run(BACKWARD);
  m2.run(BACKWARD);
  delay(1000);
  m1.run(RELEASE);
  m2.run(RELEASE);
  return true;
}
// void mission_11(){
//   int counter = 0;  // Initialize counter

//   while (counter < 3) {  // Run while counter is less than 4
//     Serial.print("Counter: ");
//     Serial.println(counter);
//     forward(3800);
//     delay(500);
//     //right();
//     counter++;  // Increment counter
//     delay(1000);

//   }
//   forward(3800);
//   delay(500);
//   //payload();
//   forward(500);

// }
// void mission_12(){
//   // Function that waits until a valid RF message is received,
//   // verifies it, and passes the number to travelDistance()
//   uint8_t buf[13] = {0};
//   uint8_t buflen = sizeof(buf);
//   Serial.print("Starting Mission 1B");
//   // Loop continuously until a valid message is received
//   while (true) {
//     // Check if a message has been received
//     if (rf_driver.recv(buf, &buflen)) {
//       // Check if the message starts with 'X'
//       if (buf[0] == 'X') {
//         // Convert the subsequent characters into an integer
//         int receivedNum = atoi((char*)&buf[1]);
//         Serial.print("Message Received: ");
//         Serial.println((char*)buf);
//         // Pass the parsed number to the travelDistance function
//         travelDistance(receivedNum);
//         break; // Exit the loop once a valid message is processed
//       } else {
//         Serial.println("Invalid message format received.");
//       }
//       // Reset the buffer length for the next attempt
//       buflen = sizeof(buf);
//     }
//     delay(100); // Small delay to avoid busy waiting
//   }
// }
// void mission_21(){
//   /*Drive in an enclosed area (approx 6’x6’) 
//   //without contacting walls or obstacles. 
//   //Themission will last 60 seconds.
//   */

//   //Use alternate forward() to cause rover to wobble left and right.
//   //This allows the rover to see walls sooner.
//   Serial.println("RUNNING MISSION 2A");
//   long L_dist, R_dist;
//   while (true){
//     //check both distance sensors to see if there's a wall.
//     L_dist = getDistance(TRIG_PIN_LEFT,ECHO_PIN_LEFT);
//     R_dist = getDistance(TRIG_PIN_RIGHT,ECHO_PIN_RIGHT);
//     Serial.print("Left: ");
//     Serial.print(L_dist);
//     Serial.print("     Right: ");
//     Serial.println(R_dist);

//     //If both sensors detect that we're close to a wall, U-turn
//     if (L_dist < 127 and R_dist < 127){ //5 inches is 127 mm
//       Serial.println("U-turn");
//       right();
//       right();
//     }
//     //If left sensor sees something, turn right 90 degrees
//     else if(L_dist < 127 and R_dist > 127){
//       Serial.println("Turn Right");
//       right();
//     }
//     //If Right sensor sees something, rutn left 90 degrees.
//     else if(L_dist > 127 and R_dist < 127){
//       Serial.println("Turn Left");
//       left();
//     }
//     //If neither sensor sees something, wobble forward
//     else if(L_dist > 127 and R_dist > 127){
//       Serial.println("Forward");
//       forward(10,5);//This is the alternate forward for wobbling.
//     }
//   }
// }

// static const unsigned long TRAVEL_TIME_FOR_30_FEET = 5000UL;  // 

// void mission_22(){
//   /* 
//     Drive forward ~30 feet directly ahead. 
//     Demonstrate obstacle avoidance by resuming path to the original waypoint 
//     after detouring around a single obstacle. 
//     Stop within 5 ft of the waypoint, all under 120 seconds (user requirement).
//   */

//   // Read initial heading
//   compass.read();
//   int startHeading = compass.getAzimuth();

//   // Setup motor speeds
//   int M1Speed = 255;
//   int M2Speed = 255;
//   m1.setSpeed(M1Speed);
//   m2.setSpeed(M2Speed);

//   // Start motors
//   m1.run(FORWARD);
//   m2.run(FORWARD);

//   // Track times
//   unsigned long startTime     = millis();
//   unsigned long obstacleTime  = 0; 
//   unsigned long secondStart   = 0; 
//   // We'll use 'flag' to indicate if we've already performed obstacle avoidance
//   int flag = 0;

//   while (true)
//   {
//     // Continuously read compass and distance sensors
//     compass.read();
//     int currentHeading = compass.getAzimuth();
//     long L_dist = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
//     long R_dist = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

//     // Basic heading correction: If off course, adjust motor speeds
//     if (currentHeading < startHeading) {
//       M1Speed -= 5;
//       M2Speed += 5;
//       m1.setSpeed(M1Speed);
//       m2.setSpeed(M2Speed);
//     }
//     else if (currentHeading > startHeading) {
//       M1Speed += 5;
//       M2Speed -= 5;
//       m1.setSpeed(M1Speed);
//       m2.setSpeed(M2Speed);
//     }

//     // Detect obstacle if either side sensor < 5 inches (127mm) 
//     if ((L_dist < 127) || (R_dist < 127)) {
//       // If we haven't yet performed avoidance
//       if (flag == 0) {
//         obstacleTime = millis(); 
//         unsigned long traveledTime = (obstacleTime - startTime);

//         // Stop motors while we do avoidance maneuvers
//         m1.run(RELEASE);
//         m2.run(RELEASE);

//         // Example obstacle-avoidance sequence:
//         left();
//         forward(500);   // Move forward (500 ms) after turning left
//         right();

//         // Mark that we’ve already done our single obstacle detour
//         flag = 1;

//         // Optionally re-read heading if it changed significantly
//         compass.read();
//         startHeading = compass.getAzimuth(); 
//       }
//     }

//     // Now start moving again toward the final waypoint
//     secondStart = millis();
//     M1Speed = 255;
//     M2Speed = 255;
//     m1.setSpeed(M1Speed);
//     m2.setSpeed(M2Speed);
//     m1.run(FORWARD);
//     m2.run(FORWARD);

//     // Calculate how long we traveled before the obstacle 
//     // (so we only complete the "remaining" distance)
//     unsigned long traveledSoFar = (obstacleTime > 0) ? (obstacleTime - startTime) : 0;

//     // Continue driving until we’ve covered total “30 ft” time
//     while (traveledSoFar + (millis() - secondStart) < TRAVEL_TIME_FOR_30_FEET) 
//     {
//       // Continuously correct heading inside this inner loop
//       compass.read();
//       currentHeading = compass.getAzimuth();

//       if (currentHeading < startHeading) {
//         M1Speed -= 5;
//         m1.setSpeed(M1Speed);
//         M2Speed += 5;
//         m2.setSpeed(M2Speed);
//       }
//       else if (currentHeading > startHeading) {
//         M1Speed += 5;
//         m1.setSpeed(M1Speed);
//         M2Speed -= 5;
//         m2.setSpeed(M2Speed);
//       }

//       // Optionally check sensors again in here 
//     }

//     // Stop motors at the final waypoint
//     m1.run(RELEASE);
//     m2.run(RELEASE);

//     // End this mission
//     break;
//   }
// }
// void mission_31(){
// }
// void mission_32(){
// }
// void mission_41(){
// }
// void mission_42(){
// }
// void travelDistance(int distance){
//   // Given that 10 feet takes 4000ms, calculate delay as 400ms per foot - changed to 4.77/ft
//   int delayTime = distance * 380;
//   Serial.print("Traveling ");
//   Serial.print(distance);
//   Serial.print(" feet (");
//   Serial.print(delayTime);
//   Serial.println(" ms).");
//   forward(delayTime);
//   delay(1000);
//   payload();
//   delay(500);
//   forward(500);
// }

void setup() {
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  /* Motor handshake */
  // if(handshakeMotor()){
  //   Serial.println("Motor Initialize Complete");
  // }

  // Setup Serial Monitor
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
  myServo.attach(9);  // Attach the servo to pin 9
  // payload();
  // /* RF Handshake */
  // if(handshakeRF() == true){
  //   continue;
  // }

  /* Ultrasonic Handshake */
  if(handshakeUltra() == true){
    Serial.print("Ultrasonic Handshake Successful");
  }
  //*****COMPASS SETUP*****
  Wire.begin();
  compass.init();
  compass.setCalibrationOffsets(-471.00, -148.00, -991.00);
  compass.setCalibrationScales(1.01, 1.21, 0.85);
  compass.read();
  float heading = compass.getAzimuth();
  Serial.print("Heading: ");
  Serial.println(heading);

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
void mission_12(){
  // Function that waits until a valid RF message is received,
  // verifies it, and passes the number to travelDistance()
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
void mission_21(){
  /*Drive in an enclosed area (approx 6’x6’) 
  //without contacting walls or obstacles. 
  //Themission will last 60 seconds.
  */

  //Use alternate forward() to cause rover to wobble left and right.
  //This allows the rover to see walls sooner.
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
    if (L_dist < 127 and R_dist < 127){ //5 inches is 127 mm
      Serial.println("U-turn");
      right();
      right();
    }
    //If left sensor sees something, turn right 90 degrees
    else if(L_dist < 127 and R_dist > 127){
      Serial.println("Turn Right");
      right();
    }
    //If Right sensor sees something, rutn left 90 degrees.
    else if(L_dist > 127 and R_dist < 127){
      Serial.println("Turn Left");
      left();
    }
    //If neither sensor sees something, wobble forward
    else if(L_dist > 127 and R_dist > 127){
      Serial.println("Forward");
      forward(10,30);//This is the alternate forward for wobbling.
    }
  }
}

static const unsigned long TRAVEL_TIME_FOR_30_FEET = 5000UL;  // 

void mission_22(){
  /* 
    Drive forward ~30 feet directly ahead. 
    Demonstrate obstacle avoidance by resuming path to the original waypoint 
    after detouring around a single obstacle. 
    Stop within 5 ft of the waypoint, all under 120 seconds (user requirement).
  */

  // Read initial heading
  compass.read();
  int startHeading = compass.getAzimuth();

  // Setup motor speeds
  int M1Speed = 255;
  int M2Speed = 255;
  m1.setSpeed(M1Speed);
  m2.setSpeed(M2Speed);

  // Start motors
  m1.run(FORWARD);
  m2.run(FORWARD);

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
      M1Speed -= 5;
      M2Speed += 5;
      m1.setSpeed(M1Speed);
      m2.setSpeed(M2Speed);
    }
    else if (currentHeading > startHeading) {
      M1Speed += 5;
      M2Speed -= 5;
      m1.setSpeed(M1Speed);
      m2.setSpeed(M2Speed);
    }

    // Detect obstacle if either side sensor < 5 inches (127mm) 
    if ((L_dist < 127) || (R_dist < 127)) {
      // If we haven't yet performed avoidance
      if (flag == 0) {
        obstacleTime = millis(); 
        unsigned long traveledTime = (obstacleTime - startTime);

        // Stop motors while we do avoidance maneuvers
        m1.run(RELEASE);
        m2.run(RELEASE);

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

    // Now start moving again toward the final waypoint
    secondStart = millis();
    M1Speed = 255;
    M2Speed = 255;
    m1.setSpeed(M1Speed);
    m2.setSpeed(M2Speed);
    m1.run(FORWARD);
    m2.run(FORWARD);

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
        M1Speed -= 5;
        m1.setSpeed(M1Speed);
        M2Speed += 5;
        m2.setSpeed(M2Speed);
      }
      else if (currentHeading > startHeading) {
        M1Speed += 5;
        m1.setSpeed(M1Speed);
        M2Speed -= 5;
        m2.setSpeed(M2Speed);
      }

      // Optionally check sensors again in here 
    }

    // Stop motors at the final waypoint
    m1.run(RELEASE);
    m2.run(RELEASE);

    // End this mission
    break;
  }
}
void mission_31(){
}
void mission_32(){
}
void mission_41(){
}
void mission_42(){
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
void checkDist(){/* Ultrasonic Sensor Check */
  long duration, dist_mm, dist_cm, dist_m;

  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN_LEFT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_LEFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_LEFT, LOW);

  // Measure the echo time
  duration = pulseIn(ECHO_PIN_LEFT, HIGH);

  // Convert time to distance
  dist_mm = (duration * 0.34) / 2;  // Distance in mm
}
long getDistance(int trigPin, int echoPin) {
    long totalDistance = 0;
    int validReadings = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        long duration, distance;

        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT);

        if (duration == 0) continue;  // Skip invalid readings

        distance = (duration * 0.34) / 2;

        if (distance > 400 || distance < 2) continue;  // Ignore out-of-range values

        totalDistance += distance;
        validReadings++;
        delay(50);
    }

    return (validReadings > 0) ? totalDistance / validReadings : -1;  // Return average or error
}
void parseCommand(char *message) {
    if (strcmp(message, "FORWARD") == 0) {
        forward(1000);
    }
    else if (strcmp(message, "BACKWARD") == 0) {
        backward();
    }
    else if (strcmp(message, "RIGHT") == 0) {
        right();
    }
    else if (strcmp(message, "LEFT") == 0) {
        left();
    }
    else if (strcmp(message, "STOP") == 0) {
        m1.run(RELEASE);
        m2.run(RELEASE);
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
    // else if (strcmp(message, "ADJUST") == 0)
    else{
      return;
    }
}
void accel() {
  static int i = 0;
  static unsigned long lastUpdate = 0;

  if (i < 255 && millis() - lastUpdate > 250) { // Slower acceleration
    m1.setSpeed(i);
    m2.setSpeed(i);
    i += 0.1;  // Change to i += 2 for a faster increase
    lastUpdate = millis();
  }
}
void decel(){
  static int i = 255;
  static unsigned long lastUpdate = 0;
  
  if (i > 0 && millis() - lastUpdate > 5) {
    m1.setSpeed(i);
    m2.setSpeed(i);
    i--;
    lastUpdate = millis();
  }
}
void forward(int wait){
  compass.read();
  int ForwardHeading = compass.getAzimuth();
  int startTime = millis();
  int CurrentHeading = 0;
  int M1Speed = 255;
  int M2Speed = 255;
  Serial.print("Forward debug");
  m1.setSpeed(255);
  m2.setSpeed(255);
  m1.run(FORWARD);
  m2.run(FORWARD);
  while (millis() > startTime + wait){
    compass.read();
    CurrentHeading = compass.getAzimuth();
    if (CurrentHeading < ForwardHeading){
        M1Speed = M1Speed-5;
        m1.setSpeed(M1Speed);
        M2Speed = M2Speed+5;
        m2.setSpeed(M2Speed);
      }
      else if (CurrentHeading > ForwardHeading){
        M1Speed = M1Speed+5;
        m1.setSpeed(M1Speed);
        M2Speed = M2Speed-5;
        m2.setSpeed(M2Speed);
    }
  }
  m1.run(RELEASE);
  m2.run(RELEASE);
}
void forward(int wait, int loops){
  //this function modifies the standard forward function by
  //controlling each motor on a sinusoidal pattern to
  //cause the rover to drive in a wobbly fashion, 
  //thus increasing the chance of seeing a wall before 
  //running into it.
  int i = 0;
  float angle = 0; //this angle is in RADIANS
  while (i<loops){
    float m1_speed = (sin(angle+3.14159)+1)*255;
    float m2_speed = (sin(angle)+1)*255;
    Serial.print("Forward debug");
    m1.setSpeed(m1_speed);
    m2.setSpeed(m2_speed);
    m1.run(FORWARD);
    m2.run(FORWARD);
    delay(wait);

    angle = angle + 3.14159;
    i++;
  }
    m1.run(RELEASE);
    m2.run(RELEASE);
}
void backward(){
  Serial.print("Backward debug");
  m1.setSpeed(250);
  m2.setSpeed(250);
  m1.run(BACKWARD);
  m2.run(BACKWARD);
  accel();
  delay(500);
  decel();
  m1.run(RELEASE);
  m2.run(RELEASE);
}
void release(){
  m1.run(RELEASE);
  m2.run(RELEASE);
  Serial.println("System Update: Motors released");
}
void right(){
  Serial.println("RIGHT TURN");
  compass.read();
  int ForwardHeading = compass.getAzimuth();
  m1.setSpeed(200);
  m2.setSpeed(200);
  m1.run(BACKWARD);
  m2.run(FORWARD);
  // int starttime = millis();
  // int endtime = starttime+1000;
  // while (millis()<endtime){
  //   compass.read();
  //   Serial.print("Degrees right to turn: ");
  //   Serial.println(compass.getAzimuth()-ForwardHeading+90);
  //   delay(5);
  // }
  delay(500);
  m1.run(RELEASE);
  m2.run(RELEASE);
}
void left(){
  Serial.println("LEFT TURN");
  compass.read();
  int ForwardHeading = compass.getAzimuth();
  m1.setSpeed(200);
  m2.setSpeed(200);
  m1.run(FORWARD);
  m2.run(BACKWARD);
  // int starttime = millis();
  // int endtime = starttime+1000;
  // while (millis()<endtime){
  //   compass.read();
  //   Serial.print("Degrees left to turn: ");
  //   Serial.println(compass.getAzimuth()-ForwardHeading+90);
  //   delay(5);
  // }
  delay(500);
  m1.run(RELEASE);
  m2.run(RELEASE);
}
void payload(){
  int pos = 0;    // variable to store the servo position
  // myServo.write(0); // Move servo to 180 degrees
  // delay(1000);        // Wait 1 second
  // myServo.write(-180); // Move servo to 180 degrees
  // delay(1000);        // Wait 1 second
  // myServo.write(0); // Move servo to 180 degrees
  // delay(1000);        // Wait 1 second
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(2);                       // waits 15ms for the servo to reach the position
  }
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15ms for the servo to reach the position
  }
}
