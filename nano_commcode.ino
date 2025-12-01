// Nano ESP32 Code (Sender)

const int LED_PIN = 2; // D2 pin for the Debug LED
bool ledState = false; // Initial state is OFF
unsigned long lastToggleTime = 0;
const long toggleInterval = 1000; // Toggle and send every 1 second

void setup() {
  // Initialize the Debug LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED OFF
  
  // Start the Hardware Serial communication
  Serial.begin(115200); 
  while (!Serial); 

  Serial.println("Nano Sender Initialized. Sending LED states...");
}

void loop() {
  if (millis() - lastToggleTime >= toggleInterval) {
    
    // 1. Toggle the LED state
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    
    // 2. Determine the message to send
    String message = ledState ? "ON" : "OFF";

    // 3. Send the message across Serial (TX pin D1)
    Serial.println(message);
    
    Serial.print("Sent: ");
    Serial.println(message);
    
    lastToggleTime = millis();
  }

  // Optional: You can still check for incoming data from the TTGO if needed
  // if (Serial.available()) {
  //   String incomingData = Serial.readStringUntil('\n');
  //   Serial.print("Received: ");
  //   Serial.println(incomingData);
  // }
}