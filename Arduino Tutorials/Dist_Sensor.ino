/*
 * ==========================================================
 * Project Name: Ultrasonic Distance Sensor Example
 * Author: Bryce Keever
 * Date: January 27, 2025
 * Version: 1.0
 * Description: This program uses an HC-SR04 ultrasonic sensor
 *              to measure distance and prints the value to
 *              the Serial Monitor.
 * ==========================================================
 * Pin Configuration:
 * - VCC_PIN: 5V
 * - GND_PIN: GND
 * - TRIG_PIN: 9 
 * - ECHO_PIN: 10 
 * ==========================================================
 */

// ==========================================================
//                      Pin Declaration
// ==========================================================
#define TRIG_PIN 9  // Pin connected to the Trig pin of the sensor
#define ECHO_PIN 10 // Pin connected to the Echo pin of the sensor

// ==========================================================
//                  Initialization Section
// ==========================================================
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600); // Start serial communication for debugging
}
// ==========================================================
//                        Main Loop 
// ==========================================================
void loop() {
  long duration, dist_mm, dist_cm, dist_m;

  // ======================================================
  // Trigger Pulse
  // ======================================================
  digitalWrite(TRIG_PIN, LOW);        // Set TRIG_PIN to digital low state
  delayMicroseconds(2);               // Send a 2-microsecond pulse to the TRIG_PIN
  digitalWrite(TRIG_PIN, HIGH);       // set TRIG_PIN to digital high state
  delayMicroseconds(10);              // Send a 10-microsecond pulse to the TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);        // Set TRIG_PIN to digital low state

  // ======================================================
  // Measure Echo and Calculate Distance
  // ======================================================
  // Speed of sound is ~340 m/s
  duration  = pulseIn(ECHO_PIN, HIGH); // Read the time for the pulse to return
  dist_mm   = (duration * 0.34)/2;     // Distance in mm
  dist_cm   = (duration * 0.034)/2;    // Distance in cm
  dist_m    = (duration * 0.00034)/2;  // Distance in meters
  // ======================================================
  // Print to Serial Monitor for User Feedback
  // ======================================================
  Serial.print("Distance: ");
  Serial.print(dist_mm);               // Variable for mm
  Serial.print(" mm,");                // Label for mm
  Serial.print(dist_cm);               // Variable for cm
  Serial.print(" cm,");                // Label for cm
  Serial.print(dist_m);                // Variable for m
  Serial.println(" m");                // Label for m
  delay(500);                          // Delay for stability
}
