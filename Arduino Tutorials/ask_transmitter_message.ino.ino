// Include RadioHead ASK Library
#include <RH_ASK.h>
// Include dependent SPI Library 
#include <SPI.h> 

// Create ASK object
RH_ASK rf_driver;

void setup()
{
    Serial.begin(9600);
    if (!rf_driver.init()) {
        Serial.println("RF Module Initialization Failed!");
    } else {
        Serial.println("RF Module Initialized.");
    }
}

void loop()
{
    Serial.println("Enter a number (0-99) to send:");
    
    // Wait for user input
    while (Serial.available() == 0);

    // Read input number
    int num = Serial.parseInt();
    
    // Clear any remaining characters in the serial buffer
    while (Serial.available()) {
        Serial.read();  // Flush out unwanted characters (like '\n')
    }

    // Validate input
    if (num < 0 || num > 99) {
        Serial.println("Invalid input. Enter a number between 0-99.");
        return;
    }

    // Format the message correctly: "X<num>ZZZZZZZZZ"
    char message[13];  // 12 characters + null terminator
    snprintf(message, sizeof(message), "X%02dZZZZZZZZZ", num);  // Always format properly

    // Send the formatted message
    Serial.print("Sending: ");
    Serial.println(message);
    
    rf_driver.send((uint8_t *)message, strlen(message));
    rf_driver.waitPacketSent();
    
    delay(1000); // Wait before allowing new input
}
