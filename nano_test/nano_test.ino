// ============================================================================
// NANO ESP32 - UART TEST VERSION (No Sensors Required)
// Generates fake sensor data to test UART communication with TTGO
// ============================================================================

// ========== UART COMMUNICATION ==========
const unsigned long uartSendInterval = 1000; // Send data every 1s
unsigned long lastUartSend = 0;

// ========== TEST DATA VARIABLES ==========
float testRPM = 1000.0;      // Simulated RPM
float testTemp = 30.0;       // Simulated temperature
float testpH = 5.0;          // Simulated pH

// Counters for generating changing test data
unsigned long counter = 0;

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);            // USB serial for debugging
  Serial1.begin(115200, SERIAL_8N1, 44, 43); // UART to TTGO (RX=44, TX=43)
  
  Serial.println("╔═══════════════════════════════════════╗");
  Serial.println("║  NANO ESP32 - UART TEST MODE         ║");
  Serial.println("║  Generating Fake Sensor Data         ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println();
  Serial.println("Setup complete. Sending test data every 1s...");
  Serial.println("Format: RPM,TEMP,PH");
  Serial.println();
}

// ========== MAIN LOOP ==========
void loop() {
  // Check for incoming UART setpoint commands from TTGO
  checkUARTCommands();
  
  // Send test data via UART to TTGO every 1s
  if (millis() - lastUartSend >= uartSendInterval) {
    lastUartSend = millis();
    
    // Generate varying test data (simulates real sensors changing)
    counter++;
    testRPM = 1000.0 + (counter % 100);           // Varies 1000-1099
    testTemp = 30.0 + (sin(counter * 0.1) * 2.0); // Oscillates 28-32°C
    testpH = 5.0 + (cos(counter * 0.1) * 0.5);    // Oscillates 4.5-5.5
    
    sendUARTData();
  }
}

// ========== UART DATA TRANSMISSION ==========
void sendUARTData() {
  // Format: RPM,TEMP,PH\n (same as real version)
  Serial1.print(testRPM, 1);
  Serial1.print(",");
  Serial1.print(testTemp, 2);
  Serial1.print(",");
  Serial1.println(testpH, 2);
  
  // Echo to USB Serial Monitor for debugging
  Serial.print("[SENT to TTGO] ");
  Serial.print(testRPM, 1);
  Serial.print(" RPM, ");
  Serial.print(testTemp, 2);
  Serial.print(" °C, pH ");
  Serial.println(testpH, 2);
}

// ========== UART COMMAND RECEPTION ==========
void checkUARTCommands() {
  // Format: SET:RPM:1000 or SET:TEMP:30 or SET:PH:5
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    
    Serial.print("[RECEIVED from TTGO] ");
    Serial.println(cmd);
    
    if (cmd.startsWith("SET:RPM:")) {
      float newRPM = cmd.substring(8).toFloat();
      if (newRPM >= 500 && newRPM <= 1500) {
        testRPM = newRPM;
        Serial.print("✓ RPM setpoint updated: ");
        Serial.println(testRPM);
      } else {
        Serial.println("✗ RPM out of range (500-1500)");
      }
    } 
    else if (cmd.startsWith("SET:TEMP:")) {
      float newTemp = cmd.substring(9).toFloat();
      if (newTemp >= 25 && newTemp <= 35) {
        testTemp = newTemp;
        Serial.print("✓ Temp setpoint updated: ");
        Serial.println(testTemp);
      } else {
        Serial.println("✗ Temp out of range (25-35)");
      }
    }
    else if (cmd.startsWith("SET:PH:")) {
      float newpH = cmd.substring(7).toFloat();
      if (newpH >= 3 && newpH <= 7) {
        testpH = newpH;
        Serial.print("✓ pH setpoint updated: ");
        Serial.println(testpH);
      } else {
        Serial.println("✗ pH out of range (3-7)");
      }
    }
    else {
      Serial.println("✗ Unknown command format");
    }
  }
}

// ============================================================================
// TESTING NOTES:
// 
// 1. Upload this to Arduino Nano ESP32
// 2. Open Serial Monitor (115200 baud)
// 3. You should see:
//    - "NANO ESP32 - UART TEST MODE" header
//    - "[SENT to TTGO] X RPM, Y °C, pH Z" every second
//    - Values changing slowly over time
//
// 4. When TTGO is connected and sends commands, you'll see:
//    - "[RECEIVED from TTGO] SET:RPM:1200"
//    - "✓ RPM setpoint updated: 1200.0"
//
// 5. The data format is identical to the real controller,
//    so TTGO and Node-RED will work exactly the same!
//
// ============================================================================
