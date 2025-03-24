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
#define TRIG_PIN_LEFT 24  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN_LEFT 23 // Pin connected to the Echo pin of the sensor
#define TRIG_PIN_RIGHT 26  // Pin connected to the Trig pin of the sensor
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
// ==========================================================
//                  Initialization Section
// ==========================================================
uint16_t outer_tread = 250;
uint16_t inner_tread = 250;
uint8_t i; // For accel and deccel

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

void handshakeUltra(){

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

void mission_11(){
  int counter = 0;  // Initialize counter

  while (counter < 3) {  // Run while counter is less than 4
    Serial.print("Counter: ");
    Serial.println(counter);
    forward(3800);
    delay(500);
    right();
    counter++;  // Increment counter
    delay(1000);

  }
  forward(3800);
  delay(500);
  payload();
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
void mission_21(){
/*Drive in an enclosed area (approx 6’x6’) 
//without contacting walls or obstacles. 
//Themission will last 60 seconds.
*/

//Use alternate forward() to cause rover to wobble left and right.
//This allows the rover to see walls sooner.
  while (true){
    //check both distance sensors to see if there's a wall.
    L_dist = getDistance( "pin", "pin");
    R_dist = getDistance( "pin", "pin");

    //If both sensors detect that we're close to a wall, U-turn
    if (L_dist < "5 inches" and R_dist < "5 inches"){
      Right();
      Right();
    }
    //If left sensor sees something, turn right 90 degrees
    else if(L_dist < "5 inches" and R_dist > "5 inches"){
      Right();
    }
    //If Right sensor sees something, rutn left 90 degrees.
    else if(L_dist > "5 inches" and R_dist < "5 inches"){
      Left();
    }
    //If neither sensor sees something, wobble forward
    else if(L_dist > "5 inches" and R_dist > "5 inches"){
      forward(10,5)//This is the alternate forward for wobbling.
    }



  }
}
void mission_22(){
}
void mission_3(){
}
void mission_4(){
}

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
  payload();
  // /* RF Handshake */
  // if(handshakeRF() == true){
  //   continue;
  // }
  // /* Ultrasonic Handshake */
  // if(handshakeUltra() == true){
  //   continue;
  // }

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
  /* Ultrasonic Sensor Check */

    //   if (sensorsEnabled) {  // Check if sensors are enabled
    //     long distanceLeft = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    //     long distanceRight = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

    //     if (distanceLeft >= 0) {
    //         Serial.print("Stable Left Side Distance: ");
    //         Serial.print(distanceLeft);
    //         Serial.println(" mm");
    //     }

    //     if (distanceRight >= 0) {
    //         Serial.print("Stable Right Side Distance: ");
    //         Serial.print(distanceRight);
    //         Serial.println(" mm");
    //     }
    // } else {
    //     Serial.println("Sensors are disabled.");
    // }
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
    else if (strcmp(message, "Mission 3") == 0) {
        mission_3();
    }
    else if (strcmp(message, "Mission 4") == 0) {
        mission_4();
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
  Serial.print("Forward debug");
  m1.setSpeed(255);
  m2.setSpeed(255);
  m1.run(FORWARD);
  m2.run(FORWARD);
  delay(wait);
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
    m1.run(RELEASE);
    m2.run(RELEASE);
    angle = angle + 3.14159;
    i++;
  }
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
  m1.setSpeed(200);
  m2.setSpeed(200);
  m1.run(BACKWARD);
  m2.run(FORWARD);
  delay(576);
  m1.run(RELEASE);
  m2.run(RELEASE);
}
void left(){
  m1.setSpeed(200);
  m2.setSpeed(200);
  m1.run(FORWARD);
  m2.run(BACKWARD);
  delay(750);
  m1.run(RELEASE);
  m2.run(RELEASE);
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
