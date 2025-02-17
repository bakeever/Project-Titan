#include <TFT_HX8357.h>
#include <SPI.h> // Required for display
#include <RH_ASK.h> // Include RadioHead ASK Library

// Create TFT object
TFT_HX8357 tft = TFT_HX8357();
// Create RF module object
RH_ASK rf_driver;

// Define named button pins
const int button1 = 11;
const int button2 = 10;
const int button3 = 9;
const int button4 = 8;
const int button5 = 7;
const int button6 = 6;

volatile bool loopEnabled = true;

String activeMission = "None"; // Dynamic label for active mission
String missionData = "Awaiting Data"; // Dynamic label for mission data

// Button states for toggling
bool buttonState1 = false;
bool buttonState2 = false;
bool buttonState3 = false;
bool buttonState4 = false;
bool buttonState5 = false;

// Previous button states for detecting changes
bool prevButtonState1 = HIGH;
bool prevButtonState2 = HIGH;
bool prevButtonState3 = HIGH;
bool prevButtonState4 = HIGH;
bool prevButtonState5 = HIGH;

void toggleLoop() {
    loopEnabled = !loopEnabled;
}

void setup() {
    Serial.begin(115200);
    tft.begin();
    tft.setRotation(3);  // Flipped screen orientation
    rf_driver.init(); // Initialize RF module

    // Attach interrupt to button6
    pinMode(button6, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(button6), toggleLoop, FALLING);

    // Display splash screen with "Project Titan" text
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(3);
    int16_t text_x_center = (480 - (12 * 11)) / 2; // Approximate centering
    tft.setCursor(text_x_center, 140);
    tft.print("Project Titan");
    delay(3000); // Show splash screen for 3 seconds

    tft.fillScreen(TFT_BLACK);  // Clear screen with black background

    // Set text properties for title
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    int16_t x_center = (480 - (25 * 2 * 7)) / 2;
    tft.setCursor(x_center, 10);
    tft.print("Project Titan Control Center");

    // Centered Active Mission label and dynamic mission text
    String activeMissionText = "Active Mission: " + activeMission;
    int16_t mission_width = activeMissionText.length() * 12; // Approximate text width
    int16_t mission_x_center = (480 - mission_width) / 2;
    tft.setCursor(mission_x_center, 40);
    tft.print(activeMissionText);

    // Move and resize mission data box to align left edge with buffer
    int boxWidth = 460;
    int boxHeight = 240;
    int boxX = 10; // Left-aligned with 10px buffer
    int boxY = 320 - boxHeight - 10; // Bottom-aligned with 10px buffer
    
    tft.drawRect(boxX, boxY, boxWidth, boxHeight, TFT_WHITE); // Expanded box for mission data
    tft.setCursor(boxX + 10, boxY + 10);
    tft.print("Mission Data:");
    tft.setCursor(boxX + 10, boxY + 30);
    tft.print(missionData);

    // Initialize button inputs
    pinMode(button1, INPUT_PULLUP);
    pinMode(button2, INPUT_PULLUP);
    pinMode(button3, INPUT_PULLUP);
    pinMode(button4, INPUT_PULLUP);
    pinMode(button5, INPUT_PULLUP);
}

void sendCommand(const char *command) {
    rf_driver.send((uint8_t *)command, strlen(command));
    rf_driver.waitPacketSent();
    Serial.println(command);
}

void updateActiveMission(String mission) {
    if (mission == activeMission) {
        return; // Do nothing if the mission is unchanged
    }
    activeMission = mission;

    // Center the updated Active Mission text
    String activeMissionText = "Active Mission: " + activeMission;
    int16_t mission_width = activeMissionText.length() * 12;
    int16_t mission_x_center = (480 - mission_width) / 2;
    tft.fillRect(0, 40, 480, 20, TFT_BLACK); // Clear previous mission text
    tft.setCursor(mission_x_center, 40);
    tft.print(activeMissionText);
}

void updateMissionData(String data) {
    if (data == missionData) {
        return; // Do nothing if the data is unchanged
    }
    missionData = data;

    int boxWidth = 460;
    int boxHeight = 240;
    int boxX = 10;
    int boxY = 320 - boxHeight - 10;
    
    tft.fillRect(boxX + 10, boxY + 30, boxWidth - 20, boxHeight - 40, TFT_BLACK); // Clear previous mission data in larger box
    tft.setCursor(boxX + 10, boxY + 30);
    tft.print(missionData);
}

void loop() {
    if (!loopEnabled) return;

    delay(50); // Small debounce delay to avoid rapid toggles
    if (!loopEnabled) return;

    if (digitalRead(button1) == LOW && prevButtonState1 == HIGH) {
        buttonState1 = !buttonState1;
        updateActiveMission("Mission 1");
        updateMissionData(buttonState1 ? "Scanning terrain" : "Idle");
        sendCommand(buttonState1 ? "Mission 1" : "Stop Mission 1");
    }
    if (digitalRead(button2) == LOW && prevButtonState2 == HIGH) {
        buttonState2 = !buttonState2;
        updateActiveMission("Mission 2");
        updateMissionData(buttonState2 ? "Locating distress beacon" : "Idle");
        sendCommand(buttonState2 ? "Mission 2" : "Stop Mission 2");
    }
    if (digitalRead(button3) == LOW && prevButtonState3 == HIGH) {
        buttonState3 = !buttonState3;
        updateActiveMission("Mission 3");
        updateMissionData(buttonState3 ? "Analyzing minerals" : "Idle");
        sendCommand(buttonState3 ? "Mission 3" : "Stop Mission 3");
    }
    prevButtonState1 = digitalRead(button1);
    prevButtonState2 = digitalRead(button2);
    prevButtonState3 = digitalRead(button3);
    
    if (digitalRead(button4) == LOW && prevButtonState4 == HIGH) {
        buttonState4 = !buttonState4;
        updateActiveMission("Mission 4");
        updateMissionData(buttonState4 ? "Delivering payload" : "Idle");
        sendCommand(buttonState4 ? "Mission 4" : "Stop Mission 4");
    }
    if (digitalRead(button5) == LOW && prevButtonState5 == HIGH) {
        buttonState5 = !buttonState5;
        updateActiveMission("Maintenance");
        updateMissionData(buttonState5 ? "Inspecting rover" : "Idle");
        sendCommand(buttonState5 ? "Maintenance" : "Stop Maintenance");
    }
    prevButtonState4 = digitalRead(button4);
    prevButtonState5 = digitalRead(button5);
}
