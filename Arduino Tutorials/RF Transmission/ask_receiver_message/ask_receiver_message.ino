// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependent SPI Library 
#include <SPI.h> 

// Create ASK object
RH_ASK rf_driver;

void setup()
{
    // Initialize Serial Monitor
    Serial.begin(9600);

    // Initialize ASK Object
    if (!rf_driver.init()) {
        Serial.println("RF Module Initialization Failed!");
    } else {
        Serial.println("RF Module Initialized.");
        Serial.println("Waiting for messages...");
    }
}

void loop()
{
    // Set buffer size for expected message
    uint8_t buf[13] = {0};  // 12 chars + null terminator
    uint8_t buflen = sizeof(buf);

    // Check if a valid message is received
    if (rf_driver.recv(buf, &buflen)) 
    {
        // Ensure the received buffer is null-terminated (prevents random characters)
        if (buflen < sizeof(buf)) {
            buf[buflen] = '\0';  
        } else {
            buf[sizeof(buf) - 1] = '\0';  
        }

        // Extract the number from the received message
        if (buf[0] == 'X') {  // Ensure correct format
            int receivedNum = atoi((char*)&buf[1]);  // Convert from string to integer (ignoring 'X')

            // Print received message
            Serial.print("Message Received: ");
            Serial.println((char*)buf);  

            // Determine if the number is even or odd
            Serial.print(receivedNum);
            if (receivedNum % 2 == 0) {
                Serial.println(" is even.");
            } else {
                Serial.println(" is odd.");
            }
        } else {
            Serial.println("Invalid message format received.");
        }

        // // Clear buffer after processing
        // memset(buf, 0, sizeof(buf));  
    }
}
