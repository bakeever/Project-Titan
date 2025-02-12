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
// ==========================================================
//                      Pin Declaration
// ==========================================================
#define TRIG_PIN_1 24  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN_1 23 // Pin connected to the Echo pin of the sensor
#define TRIG_PIN_2 26  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN_2 25 // Pin connected to the Echo pin of the sensor
#define TRIG_PIN_3 28  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN_3 27 // Pin connected to the Echo pin of the sensor
// ==========================================================
//                      Object Declaration
// ==========================================================
AF_DCMotor m1(1); // create motor #1, Left Side
AF_DCMotor m2(3); // create motor #2, Right Side

// Create ASK object
RH_ASK rf_driver(2000,19,12,10,true);

// ==========================================================
//                  Initialization Section
// ==========================================================
uint16_t inner_tread = 100;
uint16_t outer_tread = 100;
uint8_t i;


void setup() {
<<<<<<< Updated upstream
pinMode(TRIG_PIN_1, OUTPUT);
pinMode(ECHO_PIN_1, INPUT);
pinMode(TRIG_PIN_2, OUTPUT);
pinMode(ECHO_PIN_2, INPUT);
pinMode(TRIG_PIN_3, OUTPUT);
pinMode(ECHO_PIN_3, INPUT);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  motor_speed = 100;

  forward();
  stop();
  // backward();
  // stop();
=======
    // Setup Serial Monitor
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println("Waiting for message");
  // Initialize ASK Object
  rf_driver.init();
  /* Motor Initialize */
  // release();
  // // tenmp test code
  // // forward();
  // // backward();
>>>>>>> Stashed changes
  // right();
  // left();
}

void loop() {
    static uint8_t buf[10] = {0};  
    uint8_t buflen = sizeof(buf);
    
    if (rf_driver.recv(buf, &buflen)) {
        buf[buflen] = '\0';  // Null-terminate the received message
        Serial.print("Received: ");
        Serial.println((char*)buf);

        // If handshake message received, respond with "ACK"
        if (strcmp((char*)buf, "HELLO") == 0) {
            Serial.println("Sending ACK...");
            char ackMsg[] = "ACK";
            rf_driver.send((uint8_t *)ackMsg, strlen(ackMsg));
            rf_driver.waitPacketSent();
        }

        // Process movement commands
        parseCommand((char*)buf);
        Serial.print("Waitinf for next command");
        memset(buf, 0, sizeof(buf));
    }
      delay(50);  // ✅ Small delay to prevent CPU overload
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
    else{
      return;
    }
}

// void getDistance(){
//     long distanceRight    = getDistanceRight();  
//     if (distanceRight   >= 0) {Serial.print("Stable Right Side Distance: ");Serial.print(distanceRight);Serial.println(" mm");}
//     long distanceForward  = getDistanceForward(); 
//     if (distanceForward >= 0) {Serial.print("Stable Forward Distance: ");Serial.print(distanceForward);Serial.println(" mm");}
    
//     long distanceLeft     = getDistanceLeft();  

//     if (distanceLeft    >= 0) {Serial.print("Stable Left Side Distance: ");Serial.print(distanceLeft);Serial.println(" mm");}

//     delay(1000);  // Wait before next measurement
// }

// long getDistanceForward() {
//     long totalDistance = 0;
//     int validReadings = 0;

//     for (int i = 0; i < NUM_SAMPLES; i++) {
//         long duration, distance;

//         // Send trigger pulse
//         digitalWrite(TRIG_PIN_1, LOW);
//         delayMicroseconds(2);
//         digitalWrite(TRIG_PIN_1, HIGH);
//         delayMicroseconds(10);
//         digitalWrite(TRIG_PIN_1, LOW);

//         // Measure echo pulse duration with timeout
//         duration = pulseIn(ECHO_PIN_1, HIGH, PULSE_TIMEOUT);

//         // Check if timeout occurred
//         if (duration == 0) {
//             // Serial.println("Warning: No echo received! Skipping reading...");
//             continue;  // Skip this iteration if no echo is received
//         }

//         // Convert duration to distance (Speed of sound: 343 m/s or 0.034 cm/µs)
//         distance = (duration * 0.34) / 2;

//         // Ignore out-of-range values (e.g., sensor max range is ~400 cm)
//         if (distance > 400 || distance < 2) {
//             // Serial.println("Warning: Out-of-range reading! Skipping...");
//             continue;  // Ignore values outside the valid range
//         }

//         totalDistance += distance;
//         validReadings++;
//         delay(50);  // Small delay between readings to stabilize results
//     }

//     if (validReadings > 0) {

//         return totalDistance / validReadings;
//     } else {
//         // Serial.println("Error: No valid distance readings.");
//         return -1;  // Return -1 to indicate an error
//     }
// }
// long getDistanceRight() {
//     long totalDistance = 0;
//     int validReadings = 0;

//     for (int i = 0; i < NUM_SAMPLES; i++) {
//         long duration, distance;

//         // Send trigger pulse
//         digitalWrite(TRIG_PIN_2, LOW);
//         delayMicroseconds(2);
//         digitalWrite(TRIG_PIN_2, HIGH);
//         delayMicroseconds(10);
//         digitalWrite(TRIG_PIN_2, LOW);

//         // Measure echo pulse duration with timeout
//         duration = pulseIn(ECHO_PIN_2, HIGH, PULSE_TIMEOUT);

//         // Check if timeout occurred
//         if (duration == 0) {
//             // Serial.println("Warning: No echo received! Skipping reading...");
//             continue;  // Skip this iteration if no echo is received
//         }

//         // Convert duration to distance (Speed of sound: 343 m/s or 0.034 cm/µs)
//         distance = (duration * 0.34) / 2;

//         // Ignore out-of-range values (e.g., sensor max range is ~400 cm)
//         if (distance > 400 || distance < 2) {
//             // Serial.println("Warning: Out-of-range reading! Skipping...");
//             continue;  // Ignore values outside the valid range
//         }

//         totalDistance += distance;
//         validReadings++;
//         delay(50);  // Small delay between readings to stabilize results
//     }

//     if (validReadings > 0) {
//         Serial.print(totalDistance/validReadings);
//         return totalDistance / validReadings;
//     } else {
//         //Serial.println("Error: No valid distance readings.");
//         return -1;  // Return -1 to indicate an error
//     }
// }
// long getDistanceLeft() {
//     long totalDistance = 0;
//     int validReadings = 0;

//     for (int i = 0; i < NUM_SAMPLES; i++) {
//         long duration, distance;

//         // Send trigger pulse
//         digitalWrite(TRIG_PIN_3, LOW);
//         delayMicroseconds(2);
//         digitalWrite(TRIG_PIN_3, HIGH);
//         delayMicroseconds(10);
//         digitalWrite(TRIG_PIN_3, LOW);

//         // Measure echo pulse duration with timeout
//         duration = pulseIn(ECHO_PIN_3, HIGH, PULSE_TIMEOUT);

//         // Check if timeout occurred
//         if (duration == 0) {
//             // Serial.println("Warning: No echo received! Skipping reading...");
//             continue;  // Skip this iteration if no echo is received
//         }

//         // Convert duration to distance (Speed of sound: 343 m/s or 0.034 cm/µs)
//         distance = (duration * 0.34) / 2;

//         // Ignore out-of-range values (e.g., sensor max range is ~400 cm)
//         if (distance > 400 || distance < 2) {
//             // Serial.println("Warning: Out-of-range reading! Skipping...");
//             continue;  // Ignore values outside the valid range
//         }

//         totalDistance += distance;
//         validReadings++;
//         delay(50);  // Small delay between readings to stabilize results
//     }

//     if (validReadings > 0) {
//         return totalDistance / validReadings;
//     } else {
//         // Serial.println("Error: No valid distance readings.");
//         return -1;  // Return -1 to indicate an error
//     }
// }
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