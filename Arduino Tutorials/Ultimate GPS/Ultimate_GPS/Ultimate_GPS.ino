 // ==========================================================
// Project Name: Ultimate GPS Tutorial
// Author: Bryce Keever
// Date: Febuary 1, 2025
// Version: 1.0
// Description: This program interfaces with the Adafruit Ultimate GPS Breakout 
// to obtain real-time location data. It allows users to request the current 
// latitude and longitude or calculate the distance and bearing to a predefined 
// target location using the TinyGPS++ library. 
// ===============================================

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Define GPS Module connection (SoftwareSerial for Arduino Uno)
SoftwareSerial mySerial(8,7);  // RX (GPS) -> 8, TX (GPS) -> 7
Adafruit_GPS GPS(&mySerial);
TinyGPSPlus gpsParser;

#define GPSECHO false  // Set to true for debugging

// Target GPS Coordinates (specified location)
// Example Location is Confluence Hall
const double targetLat = 39.080709173856086;  
const double targetLon = -108.56006936806281; 

void setup() {
    // Startup sequence for serial connections
    Serial.begin(115200);
    delay(5000);
    GPS.begin(9600);
    Serial.print("Initializing GPS Module...");

    // GPS configuration commands
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);  // Minimum recommended data
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // Set update rate to 1Hz

    // Serial feedback for user
    Serial.println("Searching for Starlink...");

    // Wait until the GPS gets a valid fix
    while (true) {
        // Read a character from the GPS module's serial output
        char c = GPS.read();

        // If GPSECHO is enabled, print the raw GPS data to Serial Monitor for debugging
        if (c && GPSECHO) Serial.write(c);

        // Check if a new NMEA sentence has been received from the GPS module
        if (GPS.newNMEAreceived()) {
          break;  // Exit the loop once valid GPS data is received
        }

        // Print a dot (.) every second to indicate waiting for GPS fix
        Serial.print(".");
        delay(1000);  // Wait 1 second before checking again

    }

    Serial.println("\nGPS fix acquired! Type 'Show Location' or 'Chart Course to Target'.");
}

void loop() {
    // Read data from the GPS module
    char c = GPS.read();
    
    // If GPSECHO is enabled, print raw GPS data to Serial Monitor for debugging
    if (c && GPSECHO) Serial.write(c);

    // Check if a new NMEA sentence has been received
    if (GPS.newNMEAreceived()) {
        // Attempt to parse the received GPS data
        // If parsing fails, exit the function and wait for the next data update
        if (!GPS.parse(GPS.lastNMEA())) return;
    }

    // Check if there is incoming data from the Serial Monitor (user input)
    if (Serial.available()) {
        // Read the full command string from Serial Monitor until newline character ('\n')
        String command = Serial.readStringUntil('\n');
        command.trim();  // Remove any leading/trailing whitespace and new lines

        // Compare the user input and execute the corresponding function
        if (command.equalsIgnoreCase("Show Location")) {
            displayCurrLoc();  // Display current latitude and longitude
        } 
        else if (command.equalsIgnoreCase("Chart Course to Target")) {
            navigateToTarget();  // Compute and display distance & bearing to the target location
        }
    }
}


// Function to display the current location
void displayCurrLoc() {
    Serial.println("\nCurrent Location:");
    Serial.print("Latitude: "); 
    Serial.println(GPS.latitudeDegrees, 6);
    Serial.print("Longitude: "); 
    Serial.println(GPS.longitudeDegrees, 6);
}

// Function to calculate distance and bearing to the target location
void navigateToTarget() {
    // Retrieve the current latitude and longitude from the GPS module
    double currentLat = GPS.latitudeDegrees;
    double currentLon = GPS.longitudeDegrees;

    // Calculate the distance (in meters) from the current location to the target location
    double distanceToTarget = gpsParser.distanceBetween(
        currentLat, currentLon, targetLat, targetLon
    );

    // Calculate the course (bearing in degrees) from the current location to the target location
    double courseToTarget = gpsParser.courseTo(
        currentLat, currentLon, targetLat, targetLon
    );

    Serial.println("\nNavigation Data:");
    Serial.print("Distance to Target: "); Serial.print(distanceToTarget, 2); Serial.println(" meters");
    Serial.print("Course to Target: "); Serial.print(courseToTarget, 2); Serial.println(" degrees");
}
