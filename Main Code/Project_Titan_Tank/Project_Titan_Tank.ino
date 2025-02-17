/*
 * ==========================================================
 * Project Name: Ultrasonic Distance Sensor Example
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
// ==========================================================
//                      Program Variables
// ==========================================================
#define NUM_SAMPLES 5  // Number of readings to average
#define PULSE_TIMEOUT 30000  // Timeout for pulseIn() in microseconds

bool sensorsEnabled = true;  // Boolean flag to enable or disable sensors
unsigned long previousMillis = 0;  // Stores last time sensors were read
const long sensorInterval = 1000;  // 1-second interval
// ==========================================================
//                      Pin Declaration
// ==========================================================
#define TRIG_PIN_1 24  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN_1 23 // Pin connected to the Echo pin of the sensor
#define TRIG_PIN_2 26  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN_2 25 // Pin connected to the Echo pin of the sensor

#define STARTUP_ERROR 28
#define MAIN_ERROR 29

// ==========================================================
//                      Object Declaration
// ==========================================================
AF_DCMotor m1(1); // create motor #1, Right Side
AF_DCMotor m2(2); // create motor #2, Left Side

// Create ASK objects for RF Communication
RH_ASK rf_driver(2000,19,12,10,true); // Reciever
RH_ASK rf_driver1(2000,18,12,10,true); // Transmitter
// ==========================================================
//                  Initialization Section
// ==========================================================
uint16_t outer_tread = 100;
uint16_t inner_tread = 100;
uint8_t i; // For accel and deccel

void handshakeRF(){
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

void handshakeUltra(){

}
void handshakeMotor(){
  // Rotate Left 
  m1.setSpeed(outer_tread);
  m2.setSpeed(inner_tread);
  m1.run(FORWARD);
  m2.run(BACKWARD);
  delay(500);
  m1.run(RELEASE);
  m2.run(RELEASE);
  // Rotate Right
  m1.setSpeed(inner_tread);
  m2.setSpeed(outer_tread);
  m1.run(FORWARD);
  m2.run(BACKWARD);
  delay(500);
  m1.run(RELEASE);
  m2.run(RELEASE);
}
void setup() {
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  // Setup Serial Monitor
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println("Waiting for message");
  // Initialize ASK Object
  rf_driver.init();
  
  /* Motor handshake */
  if(handshakeMotor() == true){
    continue;
  }
  /* RF Handshake */
  if(handshakeRF() == true){
    continue;
  }
  /* Ultrasonic Handshake */
  if(handshakeUltra() == true){
    continue;
  }
}

void loop() {
      if (sensorsEnabled) {  // Check if sensors are enabled
        long distanceLeft = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
        long distanceRight = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

        if (distanceLeft >= 0) {
            Serial.print("Stable Left Side Distance: ");
            Serial.print(distanceLeft);
            Serial.println(" mm");
        }

        if (distanceRight >= 0) {
            Serial.print("Stable Right Side Distance: ");
            Serial.print(distanceRight);
            Serial.println(" mm");
        }
    } else {
        Serial.println("Sensors are disabled.");
    }
    /* RF Messaging */
    static uint8_t buf[10] = {0};  
    uint8_t buflen = sizeof(buf);
    
    if (rf_driver.recv(buf, &buflen)) {
        buf[buflen] = '\0';  // Null-terminate the received message
        Serial.print("Received: ");
        Serial.println((char*)buf);

        // Process movement commands
        parseCommand((char*)buf);
        Serial.print("Waitinf for next command");
        memset(buf, 0, sizeof(buf));
        delay(50);  // Small delay to prevent CPU overload
    }


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
    else if (strcmp(message, "MISSION1") == 0) {
        mission_1();
    }  
    else if (strcmp(message, "MISISON2") == 0) {
        mission_2();
    }
    else if (strcmp(message, "MISSION3") == 0) {
        mission_3();
    }
    else if (strcmp(message, "MISSION4") == 0) {
        mission_4();
    }
    else{
      return;
    }
}

void mission_1(){
}
void mission_2(){
}
void misison_3(){
}
void mission_4(){
}


void accel(){
  static int i = 0;
  static unsigned long lastUpdate = 0;
  
  if (i < 255 && millis() - lastUpdate > 5) {
    m1.setSpeed(i);
    m2.setSpeed(i);
    i++;
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
  m1.run(FORWARD);
  m2.run(FORWARD);
  accel();
  decel();
  m1.run(RELEASE);
  m2.run(RELEASE);
}
void backward(){
  m1.run(BACKWARD);
  m2.run(BACKWARD);
  accel();
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
  m1.setSpeed(outer_tread);
  m2.setSpeed(inner_tread);
  m1.run(FORWARD);
  m2.run(BACKWARD);
  delay(500);
  m1.run(RELEASE);
  m2.run(RELEASE);
}
void left(){
  m1.setSpeed(inner_tread);
  m2.setSpeed(outer_tread);
  m1.run(BACKWARD);
  m2.run(FORWARD);
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
