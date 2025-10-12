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
compass.read();
float currentHeading = getBearing();
float targetBearing = calculateBearing(currLat, currLon, destLat, destLon);
float error = headingError(currentHeading, targetBearing);

// === Obstacle Avoidance ===
long L_dist = getDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
long R_dist = getDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

if (L_dist < 300 && R_dist < 300) {
  Serial.println("Obstacle ahead! Performing U-turn...");
  setBrakes(true);
  delay(200);
  right();     // turn away
  forward(500); // short move
  left();      // reorient
  return;      // skip this cycle
} else if (L_dist < 300) {
  Serial.println("Left obstacle! Turning right...");
  setBrakes(true);
  delay(200);
  right();
  forward(300);
  left();
  return;
} else if (R_dist < 300) {
  Serial.println("Right obstacle! Turning left...");
  setBrakes(true);
  delay(200);
  left();
  forward(300);
  right();
  return;
}

// === Normal GPS Heading Navigation ===
if (abs(error) < 20) {
  Serial.println("Path Clear — Moving Forward");
  Forward();
} else if (error > 0) {
  Serial.println("Adjusting Right");
  right();
} else {
  Serial.println("Adjusting Left");
  left();
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
// === Mission 4B Variables ===
float startLat = 0;
float startLon = 0;
float targetLat = 0;
float targetLon = 0;

bool waypointReceived = false;
unsigned long lastMessageTime = 0;

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
Serial.println("=== Starting Mission 4B: Navigate to Waypoint & Return ===");

// Step 1: Wait for RF waypoint
Serial.println("Waiting for RF waypoint...");
while (!waypointReceived) {
  if (rf_driver.recv(buf, &buflen)) {
    buf[buflen] = '\0';
    Serial.print("Received RF: ");
    Serial.println((char*)buf);

    if (parseWaypointMessage((char*)buf, targetLat, targetLon)) {
      waypointReceived = true;
      Serial.print("Waypoint Target: ");
      Serial.print(targetLat, 6); Serial.print(", ");
      Serial.println(targetLon, 6);
    } else {
      Serial.println("Invalid RF message.");
    }
  }
  delay(100);
}

// Step 2: Wait for valid GPS fix
Serial.println("Waiting for valid GPS fix...");
while (!gps.location.isValid()) {
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }
  delay(100);
}

// Step 3: Save start position
startLat = gps.location.lat();
startLon = gps.location.lng();
Serial.print("Start Position: ");
Serial.print(startLat, 6); Serial.print(", ");
Serial.println(startLon, 6);

// Step 4: Go to waypoint
Serial.println("Navigating to waypoint...");
bool reachedWaypoint = false;

while (!reachedWaypoint) {
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }

  if (gps.location.isUpdated()) {
    float currLat = gps.location.lat();
    float currLon = gps.location.lng();
    float distance = distanceToTarget(currLat, currLon, targetLat, targetLon);
    Serial.print("Distance to Target: ");
    Serial.println(distance);

    driveTowardsTarget(currLat, currLon, targetLat, targetLon);

    if (distance < 2.0) {
      Serial.println("Destination Reached!");
      setBrakes(true);
      setDutyRight(0);
      setDutyLeft(0);
      reachedWaypoint = true;
    }
  }
}

// Step 5: Deploy payload
Serial.println("Deploying payload...");
payload();
delay(500);

// Step 6: Return to start
Serial.println("Returning to start...");
bool returnedToStart = false;

while (!returnedToStart) {
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }

  if (gps.location.isUpdated()) {
    float currLat = gps.location.lat();
    float currLon = gps.location.lng();
    float distance = distanceToTarget(currLat, currLon, startLat, startLon);
    Serial.print("Distance to Start: ");
    Serial.println(distance);

    driveTowardsTarget(currLat, currLon, startLat, startLon);

    if (distance < 2.0) {
      Serial.println("Returned to Start Position!");
      setBrakes(true);
      setDutyRight(0);
      setDutyLeft(0);
      returnedToStart = true;
    }
  }
}

Serial.println("Mission 4B Complete");
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
int heading2 = getBearing();
Serial.print("Heading2: ");
Serial.println(heading2);\

/* RF Verfication */
// handshakeRF();

/* Ultrasonic Handhshake */
handshakeUltra();
/* GPS Verification */
handshakeGPS();

/* DEBUG & TESTING FUNCTIONS BELOW THIS */
// mission_42();

}
void loop() {
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