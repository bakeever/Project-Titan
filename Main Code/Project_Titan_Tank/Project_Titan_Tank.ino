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
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(24, 25); // CE, CSN

const byte address[6] = "00001";
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
AF_DCMotor motor1(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
AF_DCMotor motor2(3, MOTOR12_64KHZ); // create motor #2, 64KHz pwm


// ==========================================================
//                  Initialization Section
// ==========================================================
int motor_speed = 0;

void setup() {
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
  // right();
  // stop();
  // left();
  // stop();
  Serial.println("Tank boi started");
}

void loop() {
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
    // Parse command
  parseCommand(text);
  }
  // getDistance();
}

void parseCommand(char *message) {
    char *command = strtok(message, ":"); // Split command
    char *params = strtok(NULL, ":"); // Get parameters

    if (strcmp(command, "MOVE") == 0) {
        int distance = atoi(strtok(params, ",")); // First parameter
        int speed = atoi(strtok(NULL, ",")); // Second parameter
        forward();
        delay(1000);
        stop();
    }
    else if (strcmp(command, "STOP") == 0) {
        stop();
    }
}

void getDistance(){
    long distanceRight    = getDistanceRight();  
    if (distanceRight   >= 0) {Serial.print("Stable Right Side Distance: ");Serial.print(distanceRight);Serial.println(" mm");}
    long distanceForward  = getDistanceForward(); 
    if (distanceForward >= 0) {Serial.print("Stable Forward Distance: ");Serial.print(distanceForward);Serial.println(" mm");}
    
    long distanceLeft     = getDistanceLeft();  

    if (distanceLeft    >= 0) {Serial.print("Stable Left Side Distance: ");Serial.print(distanceLeft);Serial.println(" mm");}

    delay(1000);  // Wait before next measurement
}

long getDistanceForward() {
    long totalDistance = 0;
    int validReadings = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        long duration, distance;

        // Send trigger pulse
        digitalWrite(TRIG_PIN_1, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN_1, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN_1, LOW);

        // Measure echo pulse duration with timeout
        duration = pulseIn(ECHO_PIN_1, HIGH, PULSE_TIMEOUT);

        // Check if timeout occurred
        if (duration == 0) {
            // Serial.println("Warning: No echo received! Skipping reading...");
            continue;  // Skip this iteration if no echo is received
        }

        // Convert duration to distance (Speed of sound: 343 m/s or 0.034 cm/µs)
        distance = (duration * 0.34) / 2;

        // Ignore out-of-range values (e.g., sensor max range is ~400 cm)
        if (distance > 400 || distance < 2) {
            // Serial.println("Warning: Out-of-range reading! Skipping...");
            continue;  // Ignore values outside the valid range
        }

        totalDistance += distance;
        validReadings++;
        delay(50);  // Small delay between readings to stabilize results
    }

    if (validReadings > 0) {

        return totalDistance / validReadings;
    } else {
        // Serial.println("Error: No valid distance readings.");
        return -1;  // Return -1 to indicate an error
    }
}
long getDistanceRight() {
    long totalDistance = 0;
    int validReadings = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        long duration, distance;

        // Send trigger pulse
        digitalWrite(TRIG_PIN_2, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN_2, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN_2, LOW);

        // Measure echo pulse duration with timeout
        duration = pulseIn(ECHO_PIN_2, HIGH, PULSE_TIMEOUT);

        // Check if timeout occurred
        if (duration == 0) {
            // Serial.println("Warning: No echo received! Skipping reading...");
            continue;  // Skip this iteration if no echo is received
        }

        // Convert duration to distance (Speed of sound: 343 m/s or 0.034 cm/µs)
        distance = (duration * 0.34) / 2;

        // Ignore out-of-range values (e.g., sensor max range is ~400 cm)
        if (distance > 400 || distance < 2) {
            // Serial.println("Warning: Out-of-range reading! Skipping...");
            continue;  // Ignore values outside the valid range
        }

        totalDistance += distance;
        validReadings++;
        delay(50);  // Small delay between readings to stabilize results
    }

    if (validReadings > 0) {
        Serial.print(totalDistance/validReadings);
        return totalDistance / validReadings;
    } else {
        //Serial.println("Error: No valid distance readings.");
        return -1;  // Return -1 to indicate an error
    }
}
long getDistanceLeft() {
    long totalDistance = 0;
    int validReadings = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        long duration, distance;

        // Send trigger pulse
        digitalWrite(TRIG_PIN_3, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN_3, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN_3, LOW);

        // Measure echo pulse duration with timeout
        duration = pulseIn(ECHO_PIN_3, HIGH, PULSE_TIMEOUT);

        // Check if timeout occurred
        if (duration == 0) {
            // Serial.println("Warning: No echo received! Skipping reading...");
            continue;  // Skip this iteration if no echo is received
        }

        // Convert duration to distance (Speed of sound: 343 m/s or 0.034 cm/µs)
        distance = (duration * 0.34) / 2;

        // Ignore out-of-range values (e.g., sensor max range is ~400 cm)
        if (distance > 400 || distance < 2) {
            // Serial.println("Warning: Out-of-range reading! Skipping...");
            continue;  // Ignore values outside the valid range
        }

        totalDistance += distance;
        validReadings++;
        delay(50);  // Small delay between readings to stabilize results
    }

    if (validReadings > 0) {
        return totalDistance / validReadings;
    } else {
        // Serial.println("Error: No valid distance readings.");
        return -1;  // Return -1 to indicate an error
    }
}
void forward(){
  motor1.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor2.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor1.run(FORWARD);
  motor2.run(FORWARD);


}
void backward(){
  motor1.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor2.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  delay(1000);
}
void stop(){
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}
void right(){
  motor1.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor2.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  delay(1000);
}
void left(){
  motor1.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor2.setSpeed(motor_speed);   // set the speed to 200/255 pwm signal
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  delay(1000);
}