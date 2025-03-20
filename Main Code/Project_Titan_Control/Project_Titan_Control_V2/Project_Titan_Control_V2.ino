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
// Create ASK objects for RF Communication [200byte/sec, TX on D3 (pin 2), RX on D4 (pin 3)]
RH_ASK rf_driverR(2000,19,18,10); // Reciever
RH_ASK rf_driverT(2000,19,18,10); // Transmitter

// ==========================================================
//                      Pin Declaration
// ==========================================================

// Mission Buttons
const int button12 = 12;  // Mission Go
const int button11 = 11;  // Mission 1
const int button10 = 10;  // Mission 2
const int button9 = 9;    // Mission 3
const int button8 = 8;    // Mission 4

// Movement Buttons
const int button7 = 7;    // Move Backward
const int button6 = 6;    // Move Left
const int button5 = 5;    // Move Right
const int button4 = 4;    // Move Forward
const int button3 = 3;    // E-STOP

// Manual Adjustment Buttons
const int button14 = 14;  // Set Manual Adjust 1
const int button15 = 15;  // Set Manual Adjust 2
const int button16 = 16;  // Set Manual Adjust 3
const int button17 = 17;  // Set Manual Adjust 4 

// Adjustment Mode Buttons
const int button27 = 27;  // Adjustment Mode 1 - 
const int button28 = 28;  // Adjustment Mode 2 - 
const int button29 = 29;  // Adjustment Mode 3 - 
const int button30 = 30;  // Adjustment Mode 4 - 

const int potPin_1 = A0;
const int potPin_2 = A1;
const int potPin_3 = A2;
const int potPin_4 = A3;
// ==========================================================
//                      Program Variables
// ==========================================================
volatile bool loopEnabled = true;
String activeMission = "None";
String missionData = "Awaiting Data";

// Variables to store value for pots
int potValue_1 = 0;
int potValue_2 = 0;
int potValue_3 = 0;
int potValue_4 = 0;

// Button states for toggling
// Mission Buttons
bool buttonState12 = false;  // Mission Go
bool buttonState11 = false;  // Mission 1
bool buttonState10 = false;  // Mission 2
bool buttonState9  = false;  // Mission 3
bool buttonState8  = false;  // Mission 4

// Movement Buttons
bool buttonState7  = false;  // Move Backward
bool buttonState6  = false;  // Move Left
bool buttonState5  = false;  // Move Right
bool buttonState4  = false;  // Move Forward
bool buttonState3  = false;  // E-STOP

// Manual Adjustment Set Buttons
bool buttonState14 = false;  // Set Manual Adjust 1 -
bool buttonState15 = false;  // Set Manual Adjust 2 -
bool buttonState16 = false;  // Set Manual Adjust 3 -
bool buttonState17 = false;  // Set Manual Adjust 4 -

// Adjustment Mode Buttons
bool buttonState27 = false; // Manual Adjust Mode 1
bool buttonState28 = false; // Manual Adjust Mode 2
bool buttonState29 = false; // Manual Adjust Mode 3
bool buttonState30 = false; // Manual Adjust Mode 4

// Previous button states
bool prevButtonState1 = HIGH;
bool prevButtonState2 = HIGH;
bool prevButtonState3 = HIGH;
bool prevButtonState4 = HIGH;
bool prevButtonState5 = HIGH;

void toggleLoop() {
    Serial.println("E-STOP Works");
    sendCommand("ESTOP");
}

void readPot1(){
  potValue_1 = analogRead(potPin_1); // Read the analog value (0-1023)
  Serial.print("Potentiometer 1 Value: ");
  Serial.println(potValue_1); // Print value to Serial Monitor
  delay(100); // Delay for stability
}
void readPot2(){
  potValue_2 = analogRead(potPin_2); // Read the analog value (0-1023)
  Serial.print("Potentiometer 2 Value: ");
  Serial.println(potValue_2); // Print value to Serial Monitor
  delay(100); // Delay for stability
}
void readPot3(){
  potValue_3 = analogRead(potPin_3); // Read the analog value (0-1023)
  Serial.print("Potentiometer 3 Value: ");
  Serial.println(potValue_3); // Print value to Serial Monitor
  delay(100); // Delay for stability
}
void readPot4(){
  potValue_4 = analogRead(potPin_4); // Read the analog value (0-1023)
  Serial.print("Potentiometer 4 Value: ");
  Serial.println(potValue_4); // Print value to Serial Monitor
  delay(100); // Delay for stability
}

void splash(){
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

}

void setup() {
    Serial.begin(115200);
    tft.begin();
    tft.setRotation(1);
    // Initialize ASK Object
    if (!rf_driverT.init()) {
        Serial.println("RF Module Initialization Failed!");
    } else {
        Serial.println("RF Module Initialized.");
    }

    // Initialize ETOP Interrupt
    pinMode(button3, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(button3), toggleLoop, FALLING);

    splash(); // Initializes display

    // Initialize button inputs
    for (int i = 4; i <= 12; i++) {
        pinMode(i, INPUT_PULLUP);
    }
    for (int i = 14; i <= 17; i++) {
        pinMode(i, INPUT_PULLUP);
    }
    for (int i = 27; i <= 30; i++) {
        pinMode(i, INPUT_PULLUP);
    }
}

void sendCommand(const char *command) {
    rf_driverT.send((uint8_t *)command, strlen(command));
    rf_driverT.waitPacketSent();
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
void moveBackward() { 
  sendCommand("BACKWARD"); 
  Serial.print("Jog Backward");
  while(digitalRead(button7) == LOW)
    if(digitalRead(button7) == HIGH){
      sendCommand("STOP");
      break;
    }
}
void turnLeft() { 
  sendCommand("LEFT"); 
  Serial.print("Jog Left");
  while(digitalRead(button6) == LOW)
    if(digitalRead(button6) == HIGH){
      sendCommand("STOP");
      break;
  }
}
void turnRight() { 
  sendCommand("RIGHT"); 
  Serial.print("Jog Right");
  while(digitalRead(button5) == LOW)
    if(digitalRead(button5) == HIGH){
      sendCommand("STOP");
      break;
  }
}
void moveForward() { 
  sendCommand("FORWARD"); 
  Serial.print("Jog Forward");
  while(digitalRead(button4) == LOW)
    if(digitalRead(button4) == HIGH){
      sendCommand("STOP");
      break;
  }
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
    handleMissionButton(button11, buttonState11, prevButtonState1, "Mission 1A", "Scounting Perimeter", "Mission 1B");
    handleMissionButton(button10, buttonState10, prevButtonState2, "Mission 2A", "Kamakaze Assignment", "Mission 2B");
    handleMissionButton(button9, buttonState9, prevButtonState3, "Mission 3A", "Analyzing minerals", "Mission 3B");
    handleMissionButton(button8, buttonState8, prevButtonState4, "Mission 4A", "Delivering payload", "Mission 4B");
    
    if (digitalRead(button7) == LOW) moveBackward();
    if (digitalRead(button6) == LOW) turnLeft();
    if (digitalRead(button5) == LOW) turnRight();
    if (digitalRead(button4) == LOW) moveForward();

}
