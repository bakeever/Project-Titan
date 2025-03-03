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

void mission_1(){
  int counter = 0;  // Initialize counter

  while (counter < 4) {  // Run while counter is less than 4
    Serial.print("Counter: ");
    Serial.println(counter);
    forward();
    delay(500);
    right();
    counter++;  // Increment counter
    delay(1500);
  }
  payload();
}
void mission_2(){
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
  
  myServo.attach(9);  // Attach the servo to pin 9

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
        forward();
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
    else if (strcmp(message, "Mission 1") == 0) {
        mission_1();
    }  
    else if (strcmp(message, "Mission 2") == 0) {
        mission_2();
    }
    else if (strcmp(message, "Mission 3") == 0) {
        mission_3();
    }
    else if (strcmp(message, "Mission 4") == 0) {
        mission_4();
    }
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
void forward(){
  Serial.print("Forward debug");
  m1.setSpeed(250);
  m2.setSpeed(250);
  m1.run(FORWARD);
  m2.run(FORWARD);
  delay(1000);
  m1.setSpeed(0);
  m2.setSpeed(0);
  delay(100);
  m1.run(RELEASE);
  m2.run(RELEASE);
}
void backward(){
  Serial.print("Backward debug");
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
  m1.setSpeed(150);
  m2.setSpeed(150);
  m1.run(BACKWARD);
  m2.run(FORWARD);
  delay(500);
  m1.run(RELEASE);
  m2.run(RELEASE);
}
void left(){
  m1.setSpeed(150);
  m2.setSpeed(150);
  m1.run(FORWARD);
  m2.run(BACKWARD);
  delay(500);
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
  myServo.write(0); // Move servo to 180 degrees
  delay(1000);        // Wait 1 second
  myServo.write(180); // Move servo to 180 degrees
  delay(1000);        // Wait 1 second
  myServo.write(0); // Move servo to 180 degrees
  delay(1000);        // Wait 1 second
}
