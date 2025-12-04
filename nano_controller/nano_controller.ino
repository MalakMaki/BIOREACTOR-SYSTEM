// ============================================================================
// Arduino Nano ESP32 - HYBRID VERSION with Simple RPM-to-PWM Mapping
// Temperature: DUMMY (dashboard controlled)
// pH: DUMMY (dashboard controlled)
// Stirring: REAL (Hall sensor measurement + calibrated PWM control)
// ============================================================================

// ========== PIN CONFIGURATION ==========
const int MOTOR_PIN = 3;            // D3 - Motor MOSFET Gate (PWM)
const int HALL_SENSOR_PIN = 2;      // D2 - Hall Effect Sensor

// ========== COMMUNICATION ==========
unsigned long lastSendTime = 0;
const long sendInterval = 2000;     // Send data every 2 seconds

// ========== SENSOR DATA ==========
float pH_value = 7.0;               // DUMMY - only changes via dashboard
float temperature = 25.0;           // DUMMY - only changes via dashboard
int stirring_rpm = 0;               // REAL - measured from Hall sensor

// ========== SETPOINTS (Dashboard Controllable) ==========
float Tset = 35.0;                  // Temperature target (dummy)
float pHset = 7.0;                  // pH target (dummy)
int RPMset = 0;                     // RPM target (REAL) - set by dashboard

// ========== CONTROL STATES (Dashboard Controllable) ==========
bool heaterOn = true;               // Dummy state
bool motorOn = false;               // REAL motor control
bool pHpumpOn = false;              // Dummy state

// ========== STIRRING SUBSYSTEM ==========
volatile unsigned long pulseCount = 0;
unsigned long lastRpmTime = 0;
const long rpmCalculationInterval = 1000;  // Calculate RPM every 1 second
float currentRPM = 0.0;
int targetSpeedPWM = 0;             // PWM value (0-255)

// ========== PWM-TO-RPM CALIBRATION TABLE ==========
// *** ADJUST THESE VALUES BASED ON YOUR MOTOR ***
const int calibrationPoints = 6;
int calibrationRPM[6] = {0,   200,  400,  600,  900,  1200};
int calibrationPWM[6] = {0,    40,   60,   80,  110,   135};

// ========== INTERRUPT SERVICE ROUTINE ==========
void pulseCounter() {
  pulseCount++;
}

// ========== RPM TO PWM CONVERSION FUNCTION ==========
int rpmToPWM(int targetRPM) {
  // Handle zero case
  if (targetRPM <= 0) return 0;
  
  // Handle out of range (cap at max)
  if (targetRPM >= calibrationRPM[calibrationPoints-1]) {
    return calibrationPWM[calibrationPoints-1];
  }
  
  // Find the two calibration points to interpolate between
  for (int i = 0; i < calibrationPoints - 1; i++) {
    if (targetRPM >= calibrationRPM[i] && targetRPM <= calibrationRPM[i+1]) {
      // Linear interpolation
      int rpmRange = calibrationRPM[i+1] - calibrationRPM[i];
      int pwmRange = calibrationPWM[i+1] - calibrationPWM[i];
      int rpmOffset = targetRPM - calibrationRPM[i];
      
      int pwm = calibrationPWM[i] + (rpmOffset * pwmRange / rpmRange);
      return constrain(pwm, 0, 255);
    }
  }
  
  return 40; // Default minimum PWM
}

// ========== SETUP ==========
void setup() {
  // USB Serial for debugging
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("=== Arduino Nano ESP32 - HYBRID MODE ===");
  Serial.println("=== Simple RPM-to-PWM Mapping (No Feedback Control) ===\n");

  // Serial1 for TTGO/Node-RED communication
  Serial1.begin(115200, SERIAL_8N1, D0, D1);
  Serial.println("Serial1 started on D0(RX), D1(TX) for dashboard communication.");

  // Configure stirring subsystem pins
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  
  // Attach interrupt for Hall sensor (RISING edge)
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), pulseCounter, RISING);
  
  Serial.println("\n--- Subsystems Status ---");
  Serial.println("  Temperature: DUMMY (dashboard controlled)");
  Serial.println("  pH: DUMMY (dashboard controlled)");
  Serial.println("  Stirring: REAL with Calibrated PWM Mapping");
  Serial.println("    - Hall sensor on D2");
  Serial.println("    - Motor on D3");
  Serial.println("    - 70 Pulses Per Revolution");

  Serial.println("\n--- PWM Calibration Table ---");
  for (int i = 0; i < calibrationPoints; i++) {
    Serial.print("  ");
    Serial.print(calibrationRPM[i]);
    Serial.print(" RPM → PWM ");
    Serial.println(calibrationPWM[i]);
  }

  Serial.println("\n--- Dashboard Commands Available ---");
  Serial.println("  SET:TEMP:XX (20-45°C) - Sets dummy temperature value");
  Serial.println("  SET:PH:X.X (4.0-10.0) - Sets dummy pH value");
  Serial.println("  SET:RPM:XXXX (0-1500) - Sets target RPM (direct PWM mapping)");
  Serial.println("  HEATER:ON / HEATER:OFF - Dummy heater control");
  Serial.println("  MOTOR:ON / MOTOR:OFF - Enable/disable motor");
  Serial.println("  PHPUMP:ON / PHPUMP:OFF - Dummy pH pump control");
  
  Serial.println("\n--- System Ready ---");
  Serial.println("Motor is OFF. Set RPM via dashboard to start.\n");
  
  lastRpmTime = millis();
}

// ========== MAIN LOOP ==========
void loop() {
  unsigned long currentTime = millis();

  // ===== STIRRING RPM CALCULATION (every 1 second) =====
  if (currentTime - lastRpmTime >= rpmCalculationInterval) {
    lastRpmTime = currentTime;

    // Safely grab pulse count and reset
    noInterrupts();
    unsigned long currentPulses = pulseCount;
    pulseCount = 0;
    interrupts();

    // Calculate RPM (70 pulses per revolution)
    const float pulsesPerRevolution = 70.0;
    float revolutionsPerSecond = currentPulses / pulsesPerRevolution;
    currentRPM = revolutionsPerSecond * 60.0; // Convert to RPM
    stirring_rpm = (int)currentRPM;

    // Debug output for RPM measurement
    Serial.println("\n--- RPM Measurement ---");
    Serial.print("  Pulses (1 sec): ");
    Serial.println(currentPulses);
    Serial.print("  Measured RPM: ");
    Serial.print(stirring_rpm);
    Serial.print(" | Target RPM: ");
    Serial.print(RPMset);
    Serial.print(" | PWM: ");
    Serial.println(targetSpeedPWM);
    
    int rpmError = RPMset - stirring_rpm;
    Serial.print("  RPM Error: ");
    Serial.print(rpmError);
    Serial.println(" RPM");
    
    // Warning if motor should be running but isn't
    if (motorOn && RPMset > 0 && currentPulses == 0) {
      Serial.println("  ⚠ WARNING: Motor is ON but no pulses detected. Check Hall Sensor.");
    }
    
    Serial.println("---");
  }

  // ===== SEND DATA TO DASHBOARD (every 2 seconds) =====
  if (currentTime - lastSendTime >= sendInterval) {
    String message = "PH:" + String(pH_value, 1) +
                     ",TEMP:" + String(temperature, 1) +
                     ",RPM:" + String(stirring_rpm);

    // Send to Node-RED via TTGO
    Serial1.println(message);

    // Debug output
    Serial.println("\n=== Data Sent to Dashboard ===");
    Serial.println(message);
    Serial.println("  pH: " + String(pH_value, 1) + " (DUMMY, Target: " + String(pHset, 1) + ") " + 
                   (pHpumpOn ? "[PUMP ON]" : "[PUMP OFF]"));
    Serial.println("  Temperature: " + String(temperature, 1) + "°C (DUMMY, Target: " + 
                   String(Tset, 1) + "°C) " + (heaterOn ? "[HEATER ON]" : "[HEATER OFF]"));
    Serial.println("  Stirring: " + String(stirring_rpm) + " RPM (REAL, Target: " + 
                   String(RPMset) + " RPM) " + (motorOn ? "[MOTOR ON]" : "[MOTOR OFF]"));
    Serial.println("---");

    lastSendTime = currentTime;
  }

  // ===== RECEIVE COMMANDS FROM DASHBOARD =====
  if (Serial1.available()) {
    String incomingData = Serial1.readStringUntil('\n');
    incomingData.trim();
    
    Serial.println("\n========================================");
    Serial.print("🔵 Received from Dashboard: ");
    Serial.println(incomingData);
    Serial.println("========================================");

    // ===== TEMPERATURE SETPOINT (DUMMY) =====
    if (incomingData.startsWith("SET:TEMP:")) {
      float newTemp = incomingData.substring(9).toFloat();
      if (newTemp >= 20 && newTemp <= 45) {
        Tset = newTemp;
        temperature = newTemp;
        
        Serial.println("\n--- TEMPERATURE VALUE CHANGED (DUMMY) ---");
        Serial.print("New Temperature: ");
        Serial.print(temperature, 1);
        Serial.println(" °C");
        
        Serial1.print("ACK:TEMP:");
        Serial1.println(Tset, 1);
        Serial.println("✓ ACK sent to TTGO");
      } else {
        Serial.println("ERROR: Temperature out of range (20-45°C)");
        Serial1.println("ERROR:TEMP:OUT_OF_RANGE");
      }
    }

    // ===== pH SETPOINT (DUMMY) =====
    else if (incomingData.startsWith("SET:PH:")) {
      float newPH = incomingData.substring(7).toFloat();
      if (newPH >= 4.0 && newPH <= 10.0) {
        pHset = newPH;
        pH_value = newPH;
        
        Serial.println("\n--- pH VALUE CHANGED (DUMMY) ---");
        Serial.print("New pH: ");
        Serial.println(pH_value, 1);
        
        Serial1.print("ACK:PH:");
        Serial1.println(pHset, 1);
        Serial.println("✓ ACK sent to TTGO");
      } else {
        Serial.println("ERROR: pH out of range (4.0-10.0)");
        Serial1.println("ERROR:PH:OUT_OF_RANGE");
      }
    }

    // ===== RPM SETPOINT (REAL - DIRECT PWM MAPPING) =====
    else if (incomingData.startsWith("SET:RPM:")) {
      int requestedRPM = incomingData.substring(8).toInt();
      
      if (requestedRPM >= 0 && requestedRPM <= 1500) {
        RPMset = requestedRPM;
        
        if (requestedRPM == 0) {
          // Stop motor
          motorOn = false;
          targetSpeedPWM = 0;
          analogWrite(MOTOR_PIN, 0);
          
          Serial.println("\n--- MOTOR STOPPED (RPM = 0) ---");
        } else {
          // Convert RPM to PWM using calibration table
          motorOn = true;
          targetSpeedPWM = rpmToPWM(requestedRPM);
          analogWrite(MOTOR_PIN, targetSpeedPWM);
          
          Serial.println("\n--- RPM TARGET CHANGED (DIRECT PWM) ---");
          Serial.print("Target RPM: ");
          Serial.println(RPMset);
          Serial.print("Calculated PWM: ");
          Serial.println(targetSpeedPWM);
          Serial.println("Motor State: ON");
        }
        
        Serial1.print("ACK:RPM:");
        Serial1.println(RPMset);
        Serial.println("✓ ACK sent to TTGO");
      } else {
        Serial.println("ERROR: RPM out of range (0-1500)");
        Serial1.println("ERROR:RPM:OUT_OF_RANGE");
      }
    }

    // ===== HEATER CONTROL (DUMMY) =====
    else if (incomingData == "HEATER:ON") {
      heaterOn = true;
      Serial.println("\n--- HEATER turned ON (DUMMY) ---");
      Serial1.println("ACK:HEATER:ON");
      Serial.println("✓ ACK sent to TTGO");
    }
    else if (incomingData == "HEATER:OFF") {
      heaterOn = false;
      Serial.println("\n--- HEATER turned OFF (DUMMY) ---");
      Serial1.println("ACK:HEATER:OFF");
      Serial.println("✓ ACK sent to TTGO");
    }

    // ===== MOTOR CONTROL (REAL) =====
    else if (incomingData == "MOTOR:ON") {
      motorOn = true;
      
      if (RPMset > 0) {
        targetSpeedPWM = rpmToPWM(RPMset);
        analogWrite(MOTOR_PIN, targetSpeedPWM);
        
        Serial.println("\n--- MOTOR turned ON (REAL) ---");
        Serial.print("Target RPM: ");
        Serial.print(RPMset);
        Serial.print(" | PWM: ");
        Serial.println(targetSpeedPWM);
      } else {
        Serial.println("\n--- MOTOR ON (but RPM=0, motor stays off) ---");
      }
      
      Serial1.println("ACK:MOTOR:ON");
      Serial.println("✓ ACK sent to TTGO");
    }
    else if (incomingData == "MOTOR:OFF") {
      motorOn = false;
      analogWrite(MOTOR_PIN, 0);
      
      Serial.println("\n--- MOTOR turned OFF (REAL) ---");
      
      Serial1.println("ACK:MOTOR:OFF");
      Serial.println("✓ ACK sent to TTGO");
    }

    // ===== pH PUMP CONTROL (DUMMY) =====
    else if (incomingData == "PHPUMP:ON") {
      pHpumpOn = true;
      Serial.println("\n--- pH PUMP turned ON (DUMMY) ---");
      Serial1.println("ACK:PHPUMP:ON");
      Serial.println("✓ ACK sent to TTGO");
    }
    else if (incomingData == "PHPUMP:OFF") {
      pHpumpOn = false;
      Serial.println("\n--- pH PUMP turned OFF (DUMMY) ---");
      Serial1.println("ACK:PHPUMP:OFF");
      Serial.println("✓ ACK sent to TTGO");
    }

    // ===== ACKNOWLEDGMENT FROM TTGO =====
    else if (incomingData.startsWith("ACK:")) {
      Serial.print("✓ Acknowledgment from TTGO: ");
      Serial.println(incomingData);
    }

    // ===== ERROR FROM TTGO =====
    else if (incomingData.startsWith("ERROR:")) {
      Serial.print("✗ Error from TTGO: ");
      Serial.println(incomingData);
    }

    // ===== UNKNOWN COMMAND =====
    else {
      Serial.println("⚠ ERROR: Unknown command");
      Serial1.println("ERROR:UNKNOWN_COMMAND");
    }
    
    Serial.println("========================================\n");
  }
}
