#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <Wire.h>
// #include <Arduino.h>

// Serial and GPS Settings
#define GPS_SERIAL Serial3
#define GPS_BAUD 38400
#define PC_BAUD 115200

TinyGPSPlus gps;
QMC5883LCompass compass;

void setup() {
    Serial.begin(PC_BAUD);
    GPS_SERIAL.begin(GPS_BAUD);
    Wire.begin(); // Start I2C for compass

    compass.init();

    Serial.println("Starting GPS + Compass reading...");
}

void loop() {
    // Read GPS data
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }

    if (gps.location.isUpdated()) {
        Serial.println("===============================");

        // --- GPS Data ---
        // Date and Time
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

        // Location
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

        // Altitude
        if (gps.altitude.isValid()) {
            Serial.print("Altitude: ");
            Serial.print(gps.altitude.meters());
            Serial.println(" m");
        } else {
            Serial.println("Altitude: Not Available");
        }

        // Speed
        if (gps.speed.isValid()) {
            Serial.print("Speed: ");
            Serial.print(gps.speed.kmph(), 2);
            Serial.println(" km/h");
        } else {
            Serial.println("Speed: Not Available");
        }

        // Course (GPS heading)
        if (gps.course.isValid()) {
            Serial.print("GPS Course: ");
            Serial.print(gps.course.deg(), 2);
            Serial.println("°");
        } else {
            Serial.println("GPS Course: Not Available");
        }

        // Satellites
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

        delay(500); // Update rate
        Serial.println();
    }

    
}
