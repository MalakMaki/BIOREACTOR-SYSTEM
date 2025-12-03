// ============================================================================
// BIOREACTOR NANO ESP32 CONTROLLER
// Integrates: Stirrer + Heating + pH subsystems with PI control
// Communication: UART to TTGO T-Display
// ============================================================================

// ========== PIN CONFIGURATION ==========
// STIRRER SUBSYSTEM
const byte HALL_SENSOR_PIN = 3;
const byte MOTOR_PWM_PIN = 2;

// HEATING SUBSYSTEM  
const byte TEMP_SENSOR_PIN = A0;
const byte HEATER_PWM_PIN = 10;

// pH SUBSYSTEM
const byte PH_SENSOR_PIN = A1;
const byte ACID_PUMP_PIN = 6;
const byte BASE_PUMP_PIN = 7;

// ========== STIRRER SUBSYSTEM ==========
// Motor constants
const float Kv_motor = 250.0;      // RPM per Volt
const float T_motor = 0.15;        // Time constant (s)
const float Npulses = 70.0;        // Pulses per revolution
const int RPMmax = 1500;
const int Tmin = 6e7 / RPMmax / Npulses;

// PI controller parameters (fast design: wn=wo, zeta=1)
const float wo_motor = 1.0 / T_motor;
const float wn_motor = wo_motor;
const float zeta_motor = 1.0;
const float Kp_motor = (2.0 * zeta_motor * wn_motor / wo_motor - 1.0) / Kv_motor;
const float KI_motor = (wn_motor * wn_motor) / (Kv_motor * wo_motor);

// Stirrer variables
volatile unsigned long pulseCount = 0;
unsigned long lastRpmTime = 0;
const long rpmCalculationInterval = 1000;
float currentRPM = 0.0;
float setRPM = 1000.0;             // Default setpoint
float error_motor = 0.0;
float KIinterror_motor = 0.0;
int motorPWM = 0;

// ========== HEATING SUBSYSTEM ==========
// Thermistor constants
const float Vcc = 5.0;
const float R = 10000.0;
const float Ro = 10000.0;
const float To = 25.0;
const float beta = 3700.0;
const float Kadc = 3.3 / 4096.0;

// PI controller parameters (tuned for heating)
const float Kp_temp = 1177.0;
const float KI_temp = 0.0168;
const float kHeater = 1023.0 / 3000.0;

// Heating variables
float setTemp = 35.0;              // Default setpoint (°C)
float currentTemp = 0.0;
float error_temp = 0.0;
float KIinterror_temp = 0.0;
int heaterPWM = 0;

// ========== pH SUBSYSTEM ==========
// pH sensor constants
const float KpH = 3.3 / 4095.0;
const float pH_deadband = 0.1;     // Deadband to reduce pump oscillation

// PI controller parameters (tuned for pH)
const float Kp_pH = 100.0;
const float KI_pH = 5.0;

// pH variables
float setpH = 5.0;                 // Default setpoint
float currentpH = 0.0;
float pHsmoothed = 0.0;
float error_pH = 0.0;
float KIinterror_pH = 0.0;
int acidPumpPWM = 0;
int basePumpPWM = 0;

// ========== TIMING ==========
unsigned long prevtime = 0;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 100000; // 100ms in microseconds

// ========== UART COMMUNICATION ==========
const unsigned long uartSendInterval = 1000; // Send data every 1s
unsigned long lastUartSend = 0;

// ========== INTERRUPT SERVICE ROUTINE ==========
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);            // USB serial for debugging
  Serial1.begin(115200, SERIAL_8N1, 44, 43); // UART to TTGO (RX=44, TX=43)
  
  // Stirrer pins
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), pulseCounter, RISING);
  
  // Heating pins
  pinMode(TEMP_SENSOR_PIN, INPUT);
  pinMode(HEATER_PWM_PIN, OUTPUT);
  
  // pH pins
  pinMode(PH_SENSOR_PIN, INPUT);
  pinMode(ACID_PUMP_PIN, OUTPUT);
  pinMode(BASE_PUMP_PIN, OUTPUT);
  
  // Initialize PWM
  analogWrite(MOTOR_PWM_PIN, 0);
  analogWrite(HEATER_PWM_PIN, 0);
  analogWrite(ACID_PUMP_PIN, 0);
  analogWrite(BASE_PUMP_PIN, 0);
  
  lastUpdate = micros();
  lastRpmTime = millis();
  lastUartSend = millis();
  
  Serial.println("Nano Controller Initialized");
}

// ========== MAIN LOOP ==========
void loop() {
  unsigned long currtime = micros();
  float deltat = (currtime - prevtime) * 1e-6;
  
  // Check for incoming UART setpoint commands
  checkUARTCommands();
  
  // Update control loops every 100ms
  if (currtime - lastUpdate >= updateInterval) {
    prevtime = currtime;
    lastUpdate = currtime;
    
    // ===== STIRRER CONTROL =====
    if (millis() - lastRpmTime >= rpmCalculationInterval) {
      lastRpmTime = millis();
      
      noInterrupts();
      unsigned long currentPulses = pulseCount;
      pulseCount = 0;
      interrupts();
      
      // Calculate RPM (pulses per second * 60 / Npulses)
      currentRPM = (currentPulses * 60.0) / Npulses;
      
      // PI controller
      error_motor = setRPM - currentRPM;
      KIinterror_motor += KI_motor * error_motor * (rpmCalculationInterval / 1000.0);
      KIinterror_motor = constrain(KIinterror_motor, 0, 255);
      
      float controlSignal = Kp_motor * error_motor + KIinterror_motor;
      motorPWM = constrain((int)controlSignal, 0, 255);
      analogWrite(MOTOR_PWM_PIN, motorPWM);
    }
    
    // ===== HEATING CONTROL =====
    int rawADC = analogRead(TEMP_SENSOR_PIN);
    float Vadc = rawADC * Kadc;
    float Rth = R * Vadc / (Vcc - Vadc);
    currentTemp = (To + 273.0) * beta / (beta + (To + 273.0) * log(Rth / Ro)) - 273.0 - 24.46; // 21 -> 24.46, 03/12/25

    error_temp = setTemp - currentTemp;
    KIinterror_temp += KI_temp * error_temp * deltat;
    KIinterror_temp = constrain(KIinterror_temp, 0, 3000);
    
    float heaterPower = round((kp * Te + KIIntTe) * kHeater);
    heaterPWM = constrain((int)heaterPower, 0, 3000 * 0.8); // 1023 -> 3000*0.8, 03/12/25
    analogWrite(HEATER_PWM_PIN, 1023); // heaterPWM -> 1023, 03/12/25
    
    // ===== pH CONTROL =====
    float pH_raw = KpH * analogRead(PH_SENSOR_PIN);
    pHsmoothed = 0.9 * pHsmoothed + 0.1 * pH_raw;
    currentpH = pHsmoothed;
    
    error_pH = setpH - currentpH;
    
    // Apply deadband to reduce pump cycling
    if (abs(error_pH) > pH_deadband) {
      KIinterror_pH += KI_pH * error_pH * deltat;
      KIinterror_pH = constrain(KIinterror_pH, -255, 255);
      
      float pumpSignal = Kp_pH * error_pH + KIinterror_pH;
      
      if (pumpSignal > 0) {
        // pH too low, add base
        basePumpPWM = constrain((int)pumpSignal, 0, 255);
        acidPumpPWM = 0;
      } else {
        // pH too high, add acid
        acidPumpPWM = constrain((int)abs(pumpSignal), 0, 255);
        basePumpPWM = 0;
      }
    } else {
      // Within deadband, stop pumps
      acidPumpPWM = 0;
      basePumpPWM = 0;
      KIinterror_pH = 0; // Reset integral
    }
    
    analogWrite(ACID_PUMP_PIN, acidPumpPWM);
    analogWrite(BASE_PUMP_PIN, basePumpPWM);
  }
  
  // Send data via UART to TTGO every 1s
  if (millis() - lastUartSend >= uartSendInterval) {
    lastUartSend = millis();
    sendUARTData();
  }
}

// ========== UART DATA TRANSMISSION ==========
void sendUARTData() {
  // Format: RPM,TEMP,PH\n
  Serial1.print(currentRPM, 1);
  Serial1.print(",");
  Serial1.print(currentTemp, 2);
  Serial1.print(",");
  Serial1.println(currentpH, 2);
  
  // Debug echo to USB
  Serial.print("Sent: ");
  Serial.print(currentRPM, 1);
  Serial.print(" RPM, ");
  Serial.print(currentTemp, 2);
  Serial.print(" °C, pH ");
  Serial.println(currentpH, 2);
}

// ========== UART COMMAND RECEPTION ==========
void checkUARTCommands() {
  // Format: SET:RPM:1000 or SET:TEMP:30 or SET:PH:5
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("SET:RPM:")) {
      float newRPM = cmd.substring(8).toFloat();
      if (newRPM >= 500 && newRPM <= 1500) {
        setRPM = newRPM;
        Serial.print("RPM setpoint updated: ");
        Serial.println(setRPM);
      }
    } 
    else if (cmd.startsWith("SET:TEMP:")) {
      float newTemp = cmd.substring(9).toFloat();
      if (newTemp >= 25 && newTemp <= 35) {
        setTemp = newTemp;
        Serial.print("Temp setpoint updated: ");
        Serial.println(setTemp);
      }
    }
    else if (cmd.startsWith("SET:PH:")) {
      float newpH = cmd.substring(7).toFloat();
      if (newpH >= 3 && newpH <= 7) {
        setpH = newpH;
        Serial.print("pH setpoint updated: ");
        Serial.println(setpH);
      }
    }
  }
}
