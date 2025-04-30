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
RH_ASK rf_driver(2000, 19, 18, 10);  // TX on D18, RX on D18
bool wasManualAdjustActive = false;
int activeManualAdjust = -1; // Index in missions[], or -1 if none active
// ==========================================================
//                      Mission Struct
// ==========================================================
struct Mission {
  int pin;
  bool state;
  int prevState;
  const char* name;
  const char* onMessage;
  const char* offMessage;
  bool isManualAdjust;
  int potIndex;
};


// ==========================================================
//                      Mission Array
// ==========================================================
Mission missions[] = {
  {11, false, HIGH, "Mission 1A", "Scouting Perimeter", "Stop Mission 1A", false, -1},
  {10, false, HIGH, "Mission 1B", "Kamikaze Assignment", "Stop Mission 1B", false, -1},
  {9,  false, HIGH, "Mission 2A", "Analyzing Minerals", "Stop Mission 2A", false, -1},
  {8,  false, HIGH, "Mission 4A",  "I'm going on an adventure!", "Stop Mission 3", false, -1},

  {14, false, HIGH, "Adjust1", "Manual Adjust 1 Enabled", "Manual Adjust 1 Disabled", true, 0},
  {15, false, HIGH, "Adjust2", "Manual Adjust 2 Enabled", "Manual Adjust 2 Disabled", true, 1},
  {A9, false, HIGH, "Adjust3", "Manual Adjust 3 Enabled", "Manual Adjust 3 Disabled", true, 2},
  {A8, false, HIGH, "Adjust4", "Manual Adjust 4 Enabled", "Manual Adjust 4 Disabled", true, 3}
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
    for (int i = A4; i <= A9; i++) {
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
  delay(3000);

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
    bool current = digitalRead(missions[i].pin);
    if (current == LOW && missions[i].prevState == HIGH) {
      missions[i].state = !missions[i].state;
      updateActiveMission(missions[i].name);
      updateMissionData(missions[i].state ? missions[i].onMessage : "Idle");
      sendCommand(missions[i].state ? missions[i].name : missions[i].offMessage);
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

void displayPotValue(int potIndex) {
  int potValue = analogRead(A0 + potIndex);
  Serial.print("Pot Value: ");
  Serial.println(potValue);

  tft.fillRect(20, 200, 440, 30, TFT_BLACK);
  tft.setCursor(20, 200);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
  tft.print("Potentiometer ");
  tft.print(potIndex + 1);
  tft.print(" Value: ");
  tft.print(potValue);
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

  // if (digitalRead(button14) == LOW) Serial.println("Manual Adjust 1");
  // if (digitalRead(button15) == LOW) Serial.println("Manual Adjust 2");
  // if (digitalRead(A9) == LOW) Serial.println("Manual Adjust 3");
  // if (digitalRead(A8) == LOW) Serial.println("Manual Adjust 4");

  // if (digitalRead(A4) == LOW) Serial.println("Set Adjust 1");
  // if (digitalRead(A5) == LOW) Serial.println("Set Adjust 2");
  // if (digitalRead(A6) == LOW) Serial.println("Set Adjust 3");
  // if (digitalRead(A7) == LOW) Serial.println("Set Adjust 4");
for (int i = 0; i < NUM_MISSIONS; i++) {
  int current = digitalRead(missions[i].pin);

  if (current == LOW && missions[i].prevState == HIGH) {
    missions[i].state = !missions[i].state;

    if (missions[i].isManualAdjust) {
      if (missions[i].state) {
        for (int j = 0; j < NUM_MISSIONS; j++) {
          if (j != i && missions[j].isManualAdjust) {
            missions[j].state = false;
          }
        }
        activeManualAdjust = i;
      } else {
        activeManualAdjust = -1;
      }
    } else {
      updateActiveMission(missions[i].name);
      updateMissionData(missions[i].state ? missions[i].onMessage : "Idle");
      sendCommand(missions[i].state ? missions[i].name : missions[i].offMessage);
    }
  }

  missions[i].prevState = current;
}

if (activeManualAdjust != -1) {
  displayPotValue(missions[activeManualAdjust].potIndex);
  delay(200);
  wasManualAdjustActive = true;
  Serial.print("Active Manual Adjust: ");
  Serial.println(activeManualAdjust);

} 
else if (wasManualAdjustActive) {
  tft.fillRect(20, 200, 440, 30, TFT_BLACK);
  wasManualAdjustActive = false;
}


}
