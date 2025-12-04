// ============================================================================
// Arduino Nano ESP32 - FULL REAL SYSTEM
// Temperature: REAL (Thermistor + PI Control)
// pH: REAL (pH Sensor + PI Control with Pumps)
// Stirring: REAL (Hall sensor measurement + calibrated PWM control)
// ============================================================================

// ========== PIN CONFIGURATION ==========
const int MOTOR_PIN = 3;            // D3 - Stirring Motor MOSFET Gate (PWM)
const int HALL_SENSOR_PIN = 2;      // D2 - Hall Effect Sensor
const int THERMISTOR_PIN = A0;      // A0 - Thermistor for temperature
const int HEATER_PIN = 10;          // D10 - Heater MOSFET Gate (PWM)
const int PH_PIN = A1;              // A1 - pH Sensor
const int MOTOR_A_PIN = 5;          // D5 - pH Motor A (acid pump) - PWM
const int MOTOR_B_PIN = 4;          // D4 - pH Motor B (base pump) - PWM

// ========== COMMUNICATION ==========
unsigned long lastSendTime = 0;
const long sendInterval = 2000;     // Send data every 2 seconds

// ========== SENSOR DATA ==========
float pH_value = 0.0;               // REAL - measured from pH sensor
float temperature = 0.0;           // REAL - measured from thermistor
int stirring_rpm = 0;               // REAL - measured from Hall sensor

// ========== SETPOINTS (Dashboard Controllable) ==========
float Tset = 35.0;                  // Temperature target (REAL)
float pHset = 7.0;                  // pH target (REAL)
int RPMset = 0;                     // RPM target (REAL)

// ========== CONTROL STATES (Dashboard Controllable) ==========
bool heaterOn = true;               // REAL heater control
bool motorOn = false;               // REAL motor control
bool pHpumpOn = true;               // REAL pH pump control

// ========== STIRRING SUBSYSTEM ==========
volatile unsigned long pulseCount = 0;
unsigned long lastRpmTime = 0;
const long rpmCalculationInterval = 1000;
float currentRPM = 0.0;
int targetSpeedPWM = 0;

// ========== PWM-TO-RPM CALIBRATION TABLE ==========
const int calibrationPoints = 6;
int calibrationRPM[6] = {0,   200,  400,  600,  900,  1200};
int calibrationPWM[6] = {0,    40,   60,   80,  110,   135};

// ========== HEATING SUBSYSTEM (PI CONTROLLER) ==========
const float Vcc = 5.0;
const float R = 10000.0;
const float Ro = 10000.0;
const float To = 25.0;
const float beta = 3700.0;
const float Kadc = 3.3 / 4096.0;
const float ki_temp = 0.0168;
const float kp_temp = 1177.0;
const float kHeater = 1023.0 / 3000.0;

float Vadc, T, Rth, deltat, Te, KIIntTe = 0;
long currtime, prevtime = 0;
unsigned long lastTempUpdate = 0;
int HeaterPower = 0;

// ========== pH SUBSYSTEM (PI CONTROLLER) ==========
const float KpH = 3.3 / 4095.0;     // ADC conversion for pH sensor

// pH Calibration constants
const float voltage_pH4 = 0.592;
const float voltage_pH7 = 1.582;
const float voltage_pH10 = 2.524;

// pH PI Controller parameters
const float Kp_pH = 80.0;
const float Ki_pH = 5.0;
const float pH_deadband = 0.02;

// Motor speed parameters
const int MIN_MOTOR_SPEED = 10;
const int MOTOR_OFFSET = 70;
const int MAX_MOTOR_SPEED = 255;

float voltage_pH, pH_raw, pHsmoothed = 7.0;
float pH_integral = 0.0;
unsigned long lastpHTime = 0;
int motorA_pwm = 0;
int motorB_pwm = 0;

// ========== INTERRUPT SERVICE ROUTINE ==========
void pulseCounter() {
  pulseCount++;
}

// ========== RPM TO PWM CONVERSION FUNCTION ==========
int rpmToPWM(int targetRPM) {
  if (targetRPM <= 0) return 0;
  if (targetRPM >= calibrationRPM[calibrationPoints-1]) {
    return calibrationPWM[calibrationPoints-1];
  }
  
  for (int i = 0; i < calibrationPoints - 1; i++) {
    if (targetRPM >= calibrationRPM[i] && targetRPM <= calibrationRPM[i+1]) {
      int rpmRange = calibrationRPM[i+1] - calibrationRPM[i];
      int pwmRange = calibrationPWM[i+1] - calibrationPWM[i];
      int rpmOffset = targetRPM - calibrationRPM[i];
      
      int pwm = calibrationPWM[i] + (rpmOffset * pwmRange / rpmRange);
      return constrain(pwm, 0, 255);
    }
  }
  
  return 40;
}

// ========== pH CONTROL FUNCTION ==========
void controlpHMotorsPI(float currentpH) {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastpHTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  lastpHTime = currentTime;
  
  float error = currentpH - pHset;
  
  // Apply deadband
  if (abs(error) < pH_deadband) {
    motorA_pwm = 0;
    motorB_pwm = 0;
    analogWrite(MOTOR_A_PIN, 0);
    analogWrite(MOTOR_B_PIN, 0);
    return;
  }
  
  // Update integral term (with anti-windup)
  pH_integral += error * dt;
  pH_integral = constrain(pH_integral, -20.0, 20.0);
  
  // Calculate PI control output
  float output = Kp_pH * error + Ki_pH * pH_integral;
  
  // Convert output to motor PWM values
  if (output > 0) {
    // pH is too HIGH - activate Motor A to add acid
    int temp_pwm = constrain((int)output, 0, MAX_MOTOR_SPEED - MOTOR_OFFSET);
    
    if (temp_pwm > 0 && temp_pwm < MIN_MOTOR_SPEED) {
      motorA_pwm = 0;
    } else if (temp_pwm >= MIN_MOTOR_SPEED) {
      motorA_pwm = temp_pwm + MOTOR_OFFSET;
      motorA_pwm = constrain(motorA_pwm, MOTOR_OFFSET, MAX_MOTOR_SPEED);
    } else {
      motorA_pwm = 0;
    }
    
    motorB_pwm = 0;
  }
  else if (output < 0) {
    // pH is too LOW - activate Motor B to add base
    int temp_pwm = constrain((int)(-output), 0, MAX_MOTOR_SPEED - MOTOR_OFFSET);
    
    if (temp_pwm > 0 && temp_pwm < MIN_MOTOR_SPEED) {
      motorB_pwm = 0;
    } else if (temp_pwm >= MIN_MOTOR_SPEED) {
      motorB_pwm = temp_pwm + MOTOR_OFFSET;
      motorB_pwm = constrain(motorB_pwm, MOTOR_OFFSET, MAX_MOTOR_SPEED);
    } else {
      motorB_pwm = 0;
    }
    
    motorA_pwm = 0;
  }
  else {
    motorA_pwm = 0;
    motorB_pwm = 0;
  }
  
  // Apply PWM to motors (only if pH pump is enabled)
  if (pHpumpOn) {
    analogWrite(MOTOR_A_PIN, motorA_pwm);
    analogWrite(MOTOR_B_PIN, motorB_pwm);
  } else {
    analogWrite(MOTOR_A_PIN, 0);
    analogWrite(MOTOR_B_PIN, 0);
  }
}

// ========== SETUP ==========
void setup() {
  // USB Serial for debugging
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("=== Arduino Nano ESP32 - FULL REAL SYSTEM ===");
  Serial.println("=== Real Temperature + Real pH + Real Stirring ===\n");

  // Serial1 for TTGO/Node-RED communication
  Serial1.begin(115200, SERIAL_8N1, D0, D1);
  Serial.println("Serial1 started on D0(RX), D1(TX) for dashboard communication.");

  // Configure pins
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(THERMISTOR_PIN, INPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(MOTOR_A_PIN, OUTPUT);
  pinMode(MOTOR_B_PIN, OUTPUT);
  
  // Attach interrupt for Hall sensor
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), pulseCounter, RISING);
  
  // Initialize outputs
  analogWrite(HEATER_PIN, 0);
  analogWrite(MOTOR_A_PIN, 0);
  analogWrite(MOTOR_B_PIN, 0);
  
  Serial.println("\n--- Subsystems Status ---");
  Serial.println("  Temperature: REAL (Thermistor + PI Control)");
  Serial.println("    - Thermistor on A0");
  Serial.println("    - Heater on D10");
  Serial.println("    - Target: " + String(Tset, 1) + "°C");
  
  Serial.println("  pH: REAL (pH Sensor + PI Control)");
  Serial.println("    - pH Sensor on A1");
  Serial.println("    - Acid Pump (Motor A) on D5");
  Serial.println("    - Base Pump (Motor B) on D4");
  Serial.println("    - Target: " + String(pHset, 1));
  Serial.println("    - Calibration:");
  Serial.println("      pH 4:  " + String(voltage_pH4, 3) + " V");
  Serial.println("      pH 7:  " + String(voltage_pH7, 3) + " V");
  Serial.println("      pH 10: " + String(voltage_pH10, 3) + " V");
  
  Serial.println("  Stirring: REAL with Calibrated PWM Mapping");
  Serial.println("    - Hall sensor on D2");
  Serial.println("    - Motor on D3");

  Serial.println("\n--- PWM Calibration Table (Stirring) ---");
  for (int i = 0; i < calibrationPoints; i++) {
    Serial.print("  ");
    Serial.print(calibrationRPM[i]);
    Serial.print(" RPM → PWM ");
    Serial.println(calibrationPWM[i]);
  }

  Serial.println("\n--- Dashboard Commands Available ---");
  Serial.println("  SET:TEMP:XX (20-45°C) - Sets temperature SETPOINT");
  Serial.println("  SET:PH:X.X (4.0-10.0) - Sets pH SETPOINT");
  Serial.println("  SET:RPM:XXXX (0-1500) - Sets target RPM");
  Serial.println("  HEATER:ON / HEATER:OFF - Enable/disable heater");
  Serial.println("  MOTOR:ON / MOTOR:OFF - Enable/disable stirring motor");
  Serial.println("  PHPUMP:ON / PHPUMP:OFF - Enable/disable pH pumps");
  
  Serial.println("\n--- System Ready ---\n");
  
  lastRpmTime = millis();
  lastTempUpdate = micros();
  lastpHTime = millis();
}

// ========== MAIN LOOP ==========
void loop() {
  unsigned long currentTime = millis();
  currtime = micros();
  deltat = (currtime - prevtime) * 1e-6;

  // ===== TEMPERATURE CONTROL (every 100ms) =====
  if (currtime - lastTempUpdate > 100000) {
    prevtime = currtime;
    lastTempUpdate = currtime;

    // Read thermistor
    int rawADC = analogRead(THERMISTOR_PIN);
    Vadc = rawADC * Kadc;
    
    // Calculate temperature
    Rth = R * Vadc / (Vcc - Vadc);
    T = (To + 273.0) * beta / (beta + (To + 273.0) * log(Rth / Ro)) - 273.0 - 24.46;
    temperature = T;
    
    // PI Controller
    Te = Tset - T;
    KIIntTe += ki_temp * Te * deltat;
    KIIntTe = constrain(KIIntTe, 0, 3000);
    HeaterPower = round(kp_temp * Te + KIIntTe);
    HeaterPower = kHeater * constrain(HeaterPower, 0, 3000 * 0.8);
    
    // Apply heater control
    if (heaterOn) {
      analogWrite(HEATER_PIN, HeaterPower);
    } else {
      analogWrite(HEATER_PIN, 0);
    }
  }

  // ===== pH MEASUREMENT & CONTROL (every 100ms) =====
  static unsigned long lastpHMeasurement = 0;
  if (currentTime - lastpHMeasurement >= 100) {
    lastpHMeasurement = currentTime;
    
    // Read pH sensor
    voltage_pH = KpH * analogRead(PH_PIN);
    
    // Convert voltage to pH using piecewise linear calibration
    if (voltage_pH >= voltage_pH7) {
      // Alkaline range: between pH 7 and pH 10
      float slope = (10.0 - 7.0) / (voltage_pH10 - voltage_pH7);
      pH_raw = 7.0 + slope * (voltage_pH - voltage_pH7);
    } else {
      // Acidic range: between pH 4 and pH 7
      float slope = (7.0 - 4.0) / (voltage_pH7 - voltage_pH4);
      pH_raw = 4.0 + slope * (voltage_pH - voltage_pH4);
    }
    
    // Constrain pH to reasonable range
    pH_raw = constrain(pH_raw, 0, 14);
    
    // IIR smoothing
    pHsmoothed = 0.9 * pHsmoothed + 0.1 * pH_raw;
    pH_value = pHsmoothed;
    
    // Run pH control
    controlpHMotorsPI(pHsmoothed);
  }

  // ===== STIRRING RPM CALCULATION (every 1 second) =====
  if (currentTime - lastRpmTime >= rpmCalculationInterval) {
    lastRpmTime = currentTime;

    noInterrupts();
    unsigned long currentPulses = pulseCount;
    pulseCount = 0;
    interrupts();

    const float pulsesPerRevolution = 70.0;
    float revolutionsPerSecond = currentPulses / pulsesPerRevolution;
    currentRPM = revolutionsPerSecond * 60.0;
    stirring_rpm = (int)currentRPM;

    Serial.println("\n--- System Status ---");
    Serial.print("  Temperature: ");
    Serial.print(temperature, 1);
    Serial.print("°C (Target: ");
    Serial.print(Tset, 1);
    Serial.print("°C) | Error: ");
    Serial.print(Te, 1);
    Serial.print("°C | Heater PWM: ");
    Serial.print(HeaterPower);
    Serial.print(" | Heater: ");
    Serial.println(heaterOn ? "ON" : "OFF");
    
    Serial.print("  pH: ");
    Serial.print(pH_value, 2);
    Serial.print(" (Target: ");
    Serial.print(pHset, 1);
    Serial.print(") | Error: ");
    Serial.print(pH_value - pHset, 3);
    Serial.print(" | Integral: ");
    Serial.print(pH_integral, 2);
    Serial.print(" | Acid PWM: ");
    Serial.print(motorA_pwm);
    Serial.print(" | Base PWM: ");
    Serial.print(motorB_pwm);
    Serial.print(" | Pumps: ");
    Serial.println(pHpumpOn ? "ON" : "OFF");
    
    Serial.print("  Stirring RPM: ");
    Serial.print(stirring_rpm);
    Serial.print(" (Target: ");
    Serial.print(RPMset);
    Serial.print(") | PWM: ");
    Serial.print(targetSpeedPWM);
    Serial.print(" | Motor: ");
    Serial.println(motorOn ? "ON" : "OFF");
    
    if (motorOn && RPMset > 0 && currentPulses == 0) {
      Serial.println("  ⚠ WARNING: Motor ON but no pulses detected!");
    }
    Serial.println("---");
  }

  // ===== SEND DATA TO DASHBOARD (every 2 seconds) =====
  if (currentTime - lastSendTime >= sendInterval) {
    String message = "PH:" + String(pH_value, 1) +
                     ",TEMP:" + String(temperature, 1) +
                     ",RPM:" + String(stirring_rpm);

    Serial1.println(message);

    Serial.println("\n=== Data Sent to Dashboard ===");
    Serial.println(message);
    Serial.println("---");

    lastSendTime = currentTime;
  }

  // ===== RECEIVE COMMANDS FROM DASHBOARD =====
  if (Serial1.available()) {
    String incomingData = Serial1.readStringUntil('\n');
    incomingData.trim();
    
    Serial.println("\n========================================");
    Serial.print("🔵 Received: ");
    Serial.println(incomingData);
    Serial.println("========================================");

    // ===== TEMPERATURE SETPOINT (REAL) =====
    if (incomingData.startsWith("SET:TEMP:")) {
      float newTemp = incomingData.substring(9).toFloat();
      if (newTemp >= 20 && newTemp <= 45) {
        Tset = newTemp;
        KIIntTe = 0;
        
        Serial.println("\n--- TEMPERATURE SETPOINT CHANGED (REAL) ---");
        Serial.print("New Target: ");
        Serial.print(Tset, 1);
        Serial.println("°C");
        
        Serial1.print("ACK:TEMP:");
        Serial1.println(Tset, 1);
        Serial.println("✓ ACK sent to TTGO");
      } else {
        Serial.println("ERROR: Temperature out of range (20-45°C)");
        Serial1.println("ERROR:TEMP:OUT_OF_RANGE");
      }
    }

    // ===== pH SETPOINT (REAL) =====
    else if (incomingData.startsWith("SET:PH:")) {
      float newPH = incomingData.substring(7).toFloat();
      if (newPH >= 4.0 && newPH <= 10.0) {
        pHset = newPH;
        pH_integral = 0;  // Reset integral term
        
        Serial.println("\n--- pH SETPOINT CHANGED (REAL) ---");
        Serial.print("New Target pH: ");
        Serial.println(pHset, 1);
        Serial.println("PI Controller integral term reset.");
        
        Serial1.print("ACK:PH:");
        Serial1.println(pHset, 1);
        Serial.println("✓ ACK sent to TTGO");
      } else {
        Serial.println("ERROR: pH out of range (4.0-10.0)");
        Serial1.println("ERROR:PH:OUT_OF_RANGE");
      }
    }

    // ===== RPM SETPOINT (REAL) =====
    else if (incomingData.startsWith("SET:RPM:")) {
      int requestedRPM = incomingData.substring(8).toInt();
      
      if (requestedRPM >= 0 && requestedRPM <= 1500) {
        RPMset = requestedRPM;
        
        if (requestedRPM == 0) {
          motorOn = false;
          targetSpeedPWM = 0;
          analogWrite(MOTOR_PIN, 0);
          Serial.println("\n--- MOTOR STOPPED (RPM = 0) ---");
        } else {
          motorOn = true;
          targetSpeedPWM = rpmToPWM(requestedRPM);
          analogWrite(MOTOR_PIN, targetSpeedPWM);
          
          Serial.println("\n--- RPM TARGET CHANGED ---");
          Serial.print("Target RPM: ");
          Serial.println(RPMset);
          Serial.print("PWM: ");
          Serial.println(targetSpeedPWM);
        }
        
        Serial1.print("ACK:RPM:");
        Serial1.println(RPMset);
        Serial.println("✓ ACK sent to TTGO");
      } else {
        Serial.println("ERROR: RPM out of range (0-1500)");
        Serial1.println("ERROR:RPM:OUT_OF_RANGE");
      }
    }

    // ===== HEATER CONTROL (REAL) =====
    else if (incomingData == "HEATER:ON") {
      heaterOn = true;
      Serial.println("\n--- HEATER turned ON (REAL) ---");
      Serial1.println("ACK:HEATER:ON");
      Serial.println("✓ ACK sent to TTGO");
    }
    else if (incomingData == "HEATER:OFF") {
      heaterOn = false;
      analogWrite(HEATER_PIN, 0);
      Serial.println("\n--- HEATER turned OFF (REAL) ---");
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
        Serial.println("\n--- MOTOR ON (but RPM=0) ---");
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

    // ===== pH PUMP CONTROL (REAL) =====
    else if (incomingData == "PHPUMP:ON") {
      pHpumpOn = true;
      Serial.println("\n--- pH PUMPS turned ON (REAL) ---");
      Serial.println("PI controller will now control acid/base pumps.");
      Serial1.println("ACK:PHPUMP:ON");
      Serial.println("✓ ACK sent to TTGO");
    }
    else if (incomingData == "PHPUMP:OFF") {
      pHpumpOn = false;
      analogWrite(MOTOR_A_PIN, 0);
      analogWrite(MOTOR_B_PIN, 0);
      Serial.println("\n--- pH PUMPS turned OFF (REAL) ---");
      Serial1.println("ACK:PHPUMP:OFF");
      Serial.println("✓ ACK sent to TTGO");
    }

    // ===== ACKNOWLEDGMENT/ERROR FROM TTGO =====
    else if (incomingData.startsWith("ACK:")) {
      Serial.print("✓ ACK from TTGO: ");
      Serial.println(incomingData);
    }
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
