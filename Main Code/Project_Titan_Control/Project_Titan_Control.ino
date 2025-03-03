/*
 * ==========================================================
 * Project Name: Project Titan Control Center
 * Author: Bryce Keever
 * Date: January 27, 2025
 * Version: 1.0
 * Description: 
 * ==========================================================
 */
// ==========================================================
//                      Library Includes
// ==========================================================
#include <TFT_HX8357.h>
#include <SPI.h>
#include <RH_ASK.h>

// ==========================================================
//                      Object Declaration
// ==========================================================
// Create TFT object
TFT_HX8357 tft = TFT_HX8357();
// Create ASK objects for RF Communication
RH_ASK rf_driver1(2000,17,12,10,true); // Reciever
  RH_ASK rf_driver(2000,17,16,10); // Transmitter
// ==========================================================
//                      Pin Declaration
// ==========================================================
// Define named button pins
const int button1 = 11;
const int button2 = 10;
const int button3 = 9;
const int button4 = 8;
const int button5 = 7; // Move Forward
const int button6 = 6;
const int button7 = 5;
const int button8 = 4;
const int button9 = 3; // E-STOP
const int m_select = 2; // Mission Part Selection

// ==========================================================
//                      Program Variables
// ==========================================================
volatile bool loopEnabled = true;
String activeMission = "None";
String missionData = "Awaiting Data";

// Button states for toggling
bool buttonState1 = false;
bool buttonState2 = false;
bool buttonState3 = false;
bool buttonState4 = false;
bool buttonState5 = false;
bool buttonState6 = false;
bool buttonState7 = false;
bool buttonState8 = false;

bool m_selectState = false;

// Previous button states
bool prevButtonState1 = HIGH;
bool prevButtonState2 = HIGH;
bool prevButtonState3 = HIGH;
bool prevButtonState4 = HIGH;
bool prevButtonState5 = HIGH;

void toggleLoop() {
    Serial.println("E-STOP Works");
}

void setup() {
    Serial.begin(115200);
    tft.begin();
    tft.setRotation(1);
    // Initialize ASK Object
    if (!rf_driver.init()) {
        Serial.println("RF Module Initialization Failed!");
    } else {
        Serial.println("RF Module Initialized.");
    }

    pinMode(button9, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(button9), toggleLoop, FALLING);

    // Display splash screen
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor((480 - (12 * 11)) / 2, 140);
    tft.print("Project Titan");
    delay(3000);

    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor((480 - (25 * 2 * 7)) / 2, 10);
    tft.print("Project Titan Control Center");

    // Draw mission data box
    int boxX = 10, boxY = 70, boxWidth = 460, boxHeight = 240;
    tft.drawRect(boxX, boxY, boxWidth, boxHeight, TFT_WHITE);
    tft.setCursor(boxX + 10, boxY + 10);
    tft.print("Mission Data:");
    tft.setCursor(boxX + 10, boxY + 30);
    tft.print(missionData);

    // Initialize button inputs
    for (int i = 4; i <= 11; i++) {
        pinMode(i, INPUT_PULLUP);
    }
}
void sendCommand(const char *command) {
    rf_driver.send((uint8_t *)command, strlen(command));
    rf_driver.waitPacketSent();
    delay(1000);
    Serial.print("Sent: ");
    Serial.println(command);
}


void updateActiveMission(String mission) {
    if (mission != activeMission) {
        activeMission = mission;
        // Center the updated Active Mission text
        String activeMissionText = "Active Mission: " + activeMission;
        int16_t mission_width = activeMissionText.length() * 12;
        int16_t mission_x_center = (480 - mission_width) / 2;
        tft.fillRect(0, 40, 480, 20, TFT_BLACK); // Clear previous mission text
        tft.setCursor(mission_x_center, 35);
        tft.print(activeMissionText);
    }
}

void updateMissionData(String data) {
    if (data != missionData) {
        missionData = data;
        tft.fillRect(20, 100, 440, 220, TFT_BLACK);
        tft.setCursor(20, 100);
        tft.print(missionData);
    }
}

void handleMissionButton(int button, bool &buttonState, bool &prevButtonState, const String &mission, const String &activeMsg, const String &stopMsg) {
    if (digitalRead(button) == LOW && prevButtonState == HIGH) {
        buttonState = !buttonState;
        updateActiveMission(mission);
        updateMissionData(buttonState ? activeMsg : "Idle");
        sendCommand(buttonState ? mission.c_str() : stopMsg.c_str());
    }
}

void moveForward() { 
  sendCommand("FORWARD"); 
  Serial.print("Jog Forward");
}
void moveBackward() { 
  sendCommand("BACKWARD"); 
  Serial.print("Jog Backward");
}
void turnLeft() { 
  sendCommand("LEFT"); 
  Serial.print("Jog Left");
}
void turnRight() { 
  sendCommand("RIGHT"); 
  Serial.print("Jog Right");
}
void mission_11(){
  sendCommand("M11");
    Serial.print("Mission 1 Part A");
}
void mission_12(){
  sendCommand("M12");
    Serial.print("Mission 1 Part B");
}
void mission_21(){
  sendCommand("M21");
}
void mission_22(){
  sendCommand("M22");
}

void loop() {
    handleMissionButton(button1, buttonState1, prevButtonState1, "Mission 1", "Scounting Perimeter", "Stop Mission 1");
    handleMissionButton(button2, buttonState2, prevButtonState2, "Mission 2", "Locating distress beacon", "Stop Mission 2");
    handleMissionButton(button3, buttonState3, prevButtonState3, "Mission 3", "Analyzing minerals", "Stop Mission 3");
    handleMissionButton(button4, buttonState4, prevButtonState4, "Mission 4", "Delivering payload", "Stop Mission 4");
    
    if (digitalRead(button5) == LOW) moveBackward();
    if (digitalRead(button6) == LOW) turnLeft();
    if (digitalRead(button7) == LOW) turnRight();
    if (digitalRead(button8) == LOW) moveForward();

}
