// LilyGO TTGO ESP32 Code (Receiver) - Cleaned Version

// --- Custom Serial Pin Definitions ---
// TTGO Pin to RECEIVE data from Nano (Connected to Nano D1/TX)
#define NANO_RX_PIN 21
// TTGO Pin to TRANSMIT data to Nano (Connected to Nano D0/RX)
#define NANO_TX_PIN  22
// We use Serial2 for communication with the Nano

void setup() {
  // Start the USB Serial (Serial0) for local debugging/output
  Serial.begin(115200); 
  Serial.println("TTGO Receiver Initialized. Monitoring Serial2...");
  
  // Start the SECONDARY Serial port (Serial2) for communication with the Nano
  // Format: Serial2.begin(baud, protocol, RX_pin, TX_pin)
  Serial2.begin(115200, SERIAL_8N1, NANO_RX_PIN, NANO_TX_PIN);
  
}

void loop() {
  // --- Receiving Data from Nano via Serial2 ---
  if (Serial2.available()) {
    // Read the incoming message string
    String incomingData = Serial2.readStringUntil('\n');
    
    // Clean up the string (removes carriage return/line feed)
    incomingData.trim(); 

    // Print the received data to the USB Serial Monitor for confirmation
    Serial.print("Received from Nano: ");
    Serial.println(incomingData);
    
    // --- Logic for handling the received state goes here ---
    if (incomingData == "ON") {
      // Logic for when the Nano LED is ON
      // e.g., Power up a LoRa radio, trigger a function, etc.
      Serial.println("Action: Nano LED is ON.");
    } else if (incomingData == "OFF") {
      // Logic for when the Nano LED is OFF
      Serial.println("Action: Nano LED is OFF.");
    } else {
      Serial.println("Warning: Unknown Message Received.");
    }
  }
}