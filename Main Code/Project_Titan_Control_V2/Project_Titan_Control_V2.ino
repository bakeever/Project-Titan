/*
 * ==========================================================
 * Project Name: Project Titan Control Center
 * Author: Bryce Keever
 * Date: January 27, 2025
 * Version: 2.0 - Modular Mission Handler
 * Description: Modular mission logic using TFT_HX8357 and RH_ASK
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
TFT_HX8357 tft = TFT_HX8357();
RH_ASK rf_driver(2000, 19, 18, 10);  // TX on D16, RX on D17

// ==========================================================
//                      Mission Struct
// ==========================================================
struct Mission {
  int buttonPin;
  bool state;
  bool prevState;
  String missionName;
  String activeText;
  String stopText;
};

// ==========================================================
//                      Mission Array
// ==========================================================
Mission missions[] = {
  {11, false, HIGH, "Mission 1A", "Scouting Perimeter", "Stop Mission 1A"},
  {10, false, HIGH, "Mission 1B", "Kamikaze Assignment", "Stop Mission 1B"},
  {9,  false, HIGH, "Mission 2A", "Analyzing Minerals", "Stop Mission 2A"},
  {8,  false, HIGH, "Mission 3",  "Delivering Payload", "Stop Mission 3"}
};

const int NUM_MISSIONS = sizeof(missions) / sizeof(missions[0]);

// ==========================================================
//                      Pin Declaration
// ==========================================================
const int button7 = 7;  // Move Backward
const int button6 = 6;  // Move Left
const int button5 = 5;  // Move Right
const int button4 = 4;  // Move Forward
const int button3 = 3;  // E-STOP

// Manual Adjustment Buttons
const int button14 = 14;  // Set Manual Adjust 1
const int button15 = 15;  // Set Manual Adjust 2
const int button16 = 16;  // Set Manual Adjust 3
const int button17 = 17;  // Set Manual Adjust 4 

// Adjustment Mode Buttons
const int button31 = 40;  // Adjustment Mode 1
const int button32 = 41;  // Adjustment Mode 2
const int button33 = 42;  // Adjustment Mode 3
const int button34 = 43;  // Adjustment Mode 4

const int potPin_1 = A0;
const int potPin_2 = A1;
const int potPin_3 = A2;
const int potPin_4 = A3;
// ==========================================================
//                      Variables
// ==========================================================
String activeMission = "None";
String missionData = "Awaiting Data";

// ==========================================================
//                      Setup
// ==========================================================
void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(1);

  if (!rf_driver.init()) {
    Serial.println("RF Module Initialization Failed!");
  } else {
    Serial.println("RF Module Initialized.");
  }

  // E-STOP
  pinMode(button3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button3), toggleLoop, FALLING);

  splash();

  // Button Pins
  for (int i = 4; i <= 11; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  for (int i = 14; i <= 17; i++) {
    pinMode(i, INPUT_PULLUP);
  }
}


// ==========================================================
//                      Splash Screen
// ==========================================================
void splash() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(3);
  int16_t x = 65;  // original centering
  tft.setCursor(x, 140);
  tft.print("Project Titan");
  tft.setTextSize(2);
  tft.setCursor(x, 170);  // 30 pixels below
  tft.print("Tiny Frame. Boundless Vision.");
  delay(4000);

  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor((480 - (25 * 2 * 7)) / 2, 10); // Centered title
  tft.print("Project Titan Control Center");

  int boxX = 10, boxY = 70, boxWidth = 460, boxHeight = 240;
  tft.drawRect(boxX, boxY, boxWidth, boxHeight, TFT_WHITE);
  tft.setCursor(boxX + 10, boxY + 10);
  tft.print("Mission Data:");
  tft.setCursor(boxX + 10, boxY + 30);
  tft.print(missionData);
}

// ==========================================================
//                      Mission Display
// ==========================================================
void updateActiveMission(String mission) {
  if (mission != activeMission) {
    activeMission = mission;
    String activeMissionText = "Active Mission: " + activeMission;
    int16_t mission_width = activeMissionText.length() * 12; // Assume 12px/char
    int16_t mission_x_center = (480 - mission_width) / 2;

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillRect(0, 35, 480, 24, TFT_BLACK); // clear line at y=35
    tft.setCursor(mission_x_center, 35);
    tft.print(activeMissionText);
  }
}


void updateMissionData(String data) {
  if (data != missionData) {
    missionData = data;
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillRect(20, 100, 440, 200, TFT_BLACK);
    tft.setCursor(20, 100);
    tft.print(missionData);
  }
}


// ==========================================================
//                      Mission Handling
// ==========================================================
void handleMissions() {
  for (int i = 0; i < NUM_MISSIONS; i++) {
    bool current = digitalRead(missions[i].buttonPin);
    if (current == LOW && missions[i].prevState == HIGH) {
      missions[i].state = !missions[i].state;
      updateActiveMission(missions[i].missionName);
      updateMissionData(missions[i].state ? missions[i].activeText : "Idle");
      sendCommand(missions[i].state ? missions[i].missionName.c_str() : missions[i].stopText.c_str());
    }
    missions[i].prevState = current;
  }
}

// ==========================================================
//                      Movement Controls
// ==========================================================
void moveForward() {
  sendCommand("FORW");
  Serial.println("Jog Forward");
}
void moveBackward() {
  sendCommand("BACK");
  Serial.println("Jog Backward");
}
void turnLeft() {
  sendCommand("LEFT");
  Serial.println("Jog Left");
}
void turnRight() {
  sendCommand("RIGH");
  Serial.println("Jog Right");
}
// ==========================================================
//                      Adjustment Controls
// ==========================================================



// ==========================================================
//                      Communication
// ==========================================================
void sendCommand(const char *command) {
  rf_driver.send((uint8_t *)command, strlen(command));
  rf_driver.waitPacketSent();
  Serial.print("Sent: ");
  Serial.println(command);
}

// ==========================================================
//                      Emergency Stop
// ==========================================================
void toggleLoop() {
  Serial.println("E-STOP Activated");
  sendCommand("ESTOP");
}

// ==========================================================
//                      Loop
// ==========================================================
void loop() {
  handleMissions();

  if (digitalRead(button7) == LOW) moveBackward();
  if (digitalRead(button6) == LOW) turnLeft();
  if (digitalRead(button5) == LOW) turnRight();
  if (digitalRead(button4) == LOW) moveForward();

  if (digitalRead(button14) == LOW) Serial.println("Manual Adjust 1");
  if (digitalRead(button15) == LOW) Serial.println("Manual Adjust 2");
  if (digitalRead(button16) == LOW) Serial.println("Manual Adjust 3");
  if (digitalRead(button17) == LOW) Serial.println("Manual Adjust 4");
}
