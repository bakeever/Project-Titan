/* ==========================================================
 * Project Name: Project Titan - Control Console
 * Author: Bryce Keever
 * Date: January 27, 2025
 * Version: 1.0
 * Description: 
 * ==========================================================
 * Pin Configuration:
 * - 
 * ==========================================================
 * Program Functions:
 * - 
 * ==========================================================
 */

// #include <EEPROM.h>
// #include <SPI.h>
// #include <GD2.h>
/* RF Communication */
// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>


int activeMission = 1; // Default to Mission 1

/* RF Communication */
RH_ASK rf_driver(2000,19,12,10,true);// Create ASK object

void setup()
{
<<<<<<< Updated upstream
  // Serial.begin(115200);
  // GD.begin();
  /* RF Communication */
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void loop()
{
  // swapGUI();
=======
  Serial.begin(115200);
  // GD.begin();
  // Initialize RF communication
  Serial1.begin(9600);
  // Initialize ASK Object
  rf_driver.init();
>>>>>>> Stashed changes

  Serial.println("Enter a command: FORWARD, BACKWARD, RIGHT, LEFT");

}
void loop() {
    Serial.println("\nEnter a command: FORWARD, BACKWARD, RIGHT, LEFT");

    // Wait for user input
    while (Serial.available() == 0);

    // Read input from Serial Monitor
    String input = Serial.readStringUntil('\n');  
    input.trim();  // Remove any whitespace

    // Convert to char array for RF transmission
    char command[10];  
    input.toCharArray(command, sizeof(command));

    if (isValidCommand(command)) {
        // Send the command via RF
        Serial.print("Sending: ");
        Serial.println(command);
        
        rf_driver.send((uint8_t *)command, strlen(command));
        rf_driver.waitPacketSent();
    } else {
        Serial.println("Invalid command! Please enter FORWARD, BACKWARD, RIGHT, or LEFT.");
    }

    delay(1000); // Prevent rapid input spam
}


<<<<<<< Updated upstream
// void swapGUI(){
//   GD.get_inputs(); // Read touch inputs

//   // Update activeMission when a button is pressed
//   if (GD.inputs.tag > 0) {
//     activeMission = GD.inputs.tag; // Set the selected mission
//   }

//   GD.ClearColorRGB(0x202020); // Dark Gray Background for Retro CRT Look
//   GD.Clear();

//   // Title using default Font 24
//   GD.cmd_text(230, 15, 24, OPT_CENTER, "Project TITAN Main Control Center");

//   // Draw a beige-colored box for the mission UI
//   GD.ColorRGB(0xD8C3A5);  // Beige color
//   GD.Begin(RECTS);
//   GD.Vertex2ii(160, 40);  // Top-left corner
//   GD.Vertex2ii(460, 250); // Bottom-right corner

//   // ==== NEON GLOW EFFECT FOR BUTTONS ====
//   // Draw a slightly larger, semi-transparent rectangle behind each button to create a glow effect
//   GD.ColorRGB(0x00FFFF); // Neon Cyan Glow
//   GD.ColorA(128); // Set transparency for glow effect (128 = 50% opacity)

//   GD.Begin(RECTS); // Draw glow rectangles slightly larger than buttons
//   GD.Vertex2ii(8, 48);  GD.Vertex2ii(152, 92);
//   GD.Vertex2ii(8, 98);  GD.Vertex2ii(152, 142);
//   GD.Vertex2ii(8, 148); GD.Vertex2ii(152, 192);
//   GD.Vertex2ii(8, 198); GD.Vertex2ii(152, 242);

//   GD.ColorA(255); // Reset transparency to full

//   // ==== ACTUAL BUTTONS ====
//   GD.ColorRGB(0x00FFFF); // Neon Cyan Buttons
//   GD.Tag(1); GD.cmd_button(10, 50, 140, 40, 28, (activeMission == 1) ? OPT_FLAT : 0, "Mission 1");
//   GD.Tag(2); GD.cmd_button(10, 100, 140, 40, 28, (activeMission == 2) ? OPT_FLAT : 0, "Mission 2");
//   GD.Tag(3); GD.cmd_button(10, 150, 140, 40, 28, (activeMission == 3) ? OPT_FLAT : 0, "Mission 3");
//   GD.Tag(4); GD.cmd_button(10, 200, 140, 40, 28, (activeMission == 4) ? OPT_FLAT : 0, "Mission 4");

//   // Reset to black for text contrast
//   GD.ColorRGB(0x000000);

=======
// Function to validate user input
bool isValidCommand(const char* cmd) {
    return (strcmp(cmd, "FORWARD") == 0 || strcmp(cmd, "BACKWARD") == 0 || 
            strcmp(cmd, "RIGHT") == 0 || strcmp(cmd, "LEFT") == 0 || strcmp(cmd,"STOP") == 0);
}



// void swapGUI(){
//   GD.get_inputs(); // Read touch inputs

//   // Update activeMission when a button is pressed
//   if (GD.inputs.tag > 0) {
//     activeMission = GD.inputs.tag; // Set the selected mission
//   }

//   GD.ClearColorRGB(0x202020); // Dark Gray Background for Retro CRT Look
//   GD.Clear();

//   // Title using default Font 24
//   GD.cmd_text(230, 15, 24, OPT_CENTER, "Project TITAN Main Control Center");

//   // Draw a beige-colored box for the mission UI
//   GD.ColorRGB(0xD8C3A5);  // Beige color
//   GD.Begin(RECTS);
//   GD.Vertex2ii(160, 40);  // Top-left corner
//   GD.Vertex2ii(460, 250); // Bottom-right corner

//   // ==== NEON GLOW EFFECT FOR BUTTONS ====
//   // Draw a slightly larger, semi-transparent rectangle behind each button to create a glow effect
//   GD.ColorRGB(0x00FFFF); // Neon Cyan Glow
//   GD.ColorA(128); // Set transparency for glow effect (128 = 50% opacity)

//   GD.Begin(RECTS); // Draw glow rectangles slightly larger than buttons
//   GD.Vertex2ii(8, 48);  GD.Vertex2ii(152, 92);
//   GD.Vertex2ii(8, 98);  GD.Vertex2ii(152, 142);
//   GD.Vertex2ii(8, 148); GD.Vertex2ii(152, 192);
//   GD.Vertex2ii(8, 198); GD.Vertex2ii(152, 242);

//   GD.ColorA(255); // Reset transparency to full

//   // ==== ACTUAL BUTTONS ====
//   GD.ColorRGB(0x00FFFF); // Neon Cyan Buttons
//   GD.Tag(1); GD.cmd_button(10, 50, 140, 40, 28, (activeMission == 1) ? OPT_FLAT : 0, "Mission 1");
//   GD.Tag(2); GD.cmd_button(10, 100, 140, 40, 28, (activeMission == 2) ? OPT_FLAT : 0, "Mission 2");
//   GD.Tag(3); GD.cmd_button(10, 150, 140, 40, 28, (activeMission == 3) ? OPT_FLAT : 0, "Mission 3");
//   GD.Tag(4); GD.cmd_button(10, 200, 140, 40, 28, (activeMission == 4) ? OPT_FLAT : 0, "Mission 4");

//   // Reset to black for text contrast
//   GD.ColorRGB(0x000000);

>>>>>>> Stashed changes
//   // Display GUI elements for the selected mission
//   switch (activeMission) {
//     case 1:
//       GD.cmd_text(310, 80, 28, OPT_CENTER, "Mission 1: STATUS");
//       GD.cmd_progress(200, 120, 200, 20, 0, 50, 100); // Progress bar (50%)
//       GD.cmd_text(310, 160, 28, OPT_CENTER, "Sensors: ACTIVE");
//       break;
    
//     case 2:
//       GD.cmd_text(310, 80, 28, OPT_CENTER, "Mission 2: CALIBRATING");
//       GD.cmd_gauge(300, 150, 40, 0, 5, 10, 75, 100); // Fixed Gauge
//       GD.cmd_text(310, 190, 28, OPT_CENTER, "Stabilization: GOOD");
//       break;

//     case 3:
//       GD.cmd_text(310, 80, 28, OPT_CENTER, "Mission 3: NAVIGATION");
//       GD.cmd_slider(200, 120, 200, 20, 0, 30, 100); // Slider at 30%
//       GD.cmd_text(310, 160, 28, OPT_CENTER, "Direction: EAST");
//       break;

//     case 4:
//       GD.cmd_text(310, 80, 28, OPT_CENTER, "Mission 4: ANALYSIS");
//       GD.cmd_clock(300, 160, 40, 0, 3, 45, 20, 500); // Fixed Clock
//       GD.cmd_text(310, 200, 28, OPT_CENTER, "Data Sync: 72%");
//       break;
//   }

//   GD.swap();
// }

// void sendCommand(){
//   const char command[] = "MOVE"; // Example command
//   radio.write(&command, sizeof(command));
//   Serial.println("Command sent!");

//   // Switch to Receive Mode to get ACK
//   radio.startListening();
//   delay(50); // Small delay to allow response

//   if (radio.available()) {
//       char response[32] = "";
//       radio.read(&response, sizeof(response));
//       Serial.print("Received from Slave: ");
//       Serial.println(response);
//   }

//   // Switch back to Transmit Mode for next command
//   radio.stopListening();
//   delay(1000); // Wait before sending next command
// }
