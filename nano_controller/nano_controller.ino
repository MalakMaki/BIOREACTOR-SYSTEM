// BIOREACTOR NANO ESP32 CONTROLLER
// Format: PH:8.0,TEMP:22.2,RPM:1020

const byte HALL_SENSOR_PIN = 3;
const byte MOTOR_PWM_PIN = 2;
const byte TEMP_SENSOR_PIN = A0;
const byte HEATER_PWM_PIN = 10;
const byte PH_SENSOR_PIN = A1;
const byte ACID_PUMP_PIN = 6;
const byte BASE_PUMP_PIN = 7;

// STIRRER SUBSYSTEM
const float magnetsPerRevolution = 70.0;
volatile unsigned long pulseCount = 0;
unsigned long lastRpmTime = 0;
const long rpmCalculationInterval = 1000;
float currentRPM = 0.0;
float setRPM = 1000.0;
float error_motor = 0.0;
float KIinterror_motor = 0.0;
const float Kp_motor = 0.1;
const float KI_motor = 0.01;
int motorPWM = 0;

// HEATING SUBSYSTEM
const float Vcc = 5.0;
const float R = 10000.0;
const float Ro = 10000.0;
const float To = 25.0;
const float beta = 3700.0;
const float Kadc = 3.3 / 4096.0;
const float Kp_temp = 1177.0;
const float KI_temp = 0.0168;
const float kHeater = 1023.0 / 3000.0;

float setTemp = 35.0;
float currentTemp = 0.0;
float error_temp = 0.0;
float KIinterror_temp = 0.0;
int heaterPWM = 0;

// pH SUBSYSTEM
const float KpH = 3.3 / 4095.0;
const float pH_deadband = 0.1;
const float Kp_pH = 100.0;
const float KI_pH = 5.0;

float setpH = 5.0;
float currentpH = 0.0;
float error_pH = 0.0;
float KIinterror_pH = 0.0;
int acidPumpPWM = 0;
int basePumpPWM = 0;

unsigned long prevtime = 0;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 100000;
const unsigned long uartSendInterval = 2000;
unsigned long lastUartSend = 0;

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 44, 43);
  
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(TEMP_SENSOR_PIN, INPUT);
  pinMode(HEATER_PWM_PIN, OUTPUT);
  pinMode(PH_SENSOR_PIN, INPUT);
  pinMode(ACID_PUMP_PIN, OUTPUT);
  pinMode(BASE_PUMP_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), pulseCounter, RISING);
  
  analogWrite(MOTOR_PWM_PIN, motorPWM);
  analogWrite(HEATER_PWM_PIN, 0);
  analogWrite(ACID_PUMP_PIN, 0);
  analogWrite(BASE_PUMP_PIN, 0);
  
  lastUpdate = micros();
  lastRpmTime = millis();
  lastUartSend = millis();
  
  Serial.println("Nano Controller Initialized");
}

void loop() {
  unsigned long currtime = micros();
  float deltat = (currtime - prevtime) * 1e-6;
  
  checkUARTCommands();
  
  if (currtime - lastUpdate >= updateInterval) {
    prevtime = currtime;
    lastUpdate = currtime;
    
    // STIRRER CONTROL
    if (millis() - lastRpmTime >= rpmCalculationInterval) {
      lastRpmTime = millis();
      
      noInterrupts();
      unsigned long currentPulses = pulseCount;
      pulseCount = 0;
      interrupts();
      
      currentRPM = (currentPulses * 60.0) / magnetsPerRevolution;
      
      error_motor = setRPM - currentRPM;
      KIinterror_motor += KI_motor * error_motor;
      KIinterror_motor = constrain(KIinterror_motor, 0, 255);
      motorPWM = constrain(Kp_motor * error_motor + KIinterror_motor, 0, 255);
      analogWrite(MOTOR_PWM_PIN, motorPWM);
    }
    
    // HEATING CONTROL
    int rawADC = analogRead(TEMP_SENSOR_PIN);
    float Vadc = rawADC * Kadc;
    float Rth = R * Vadc / (Vcc - Vadc);
    currentTemp = (To + 273.0) * beta / (beta + (To + 273.0) * log(Rth / Ro)) - 273.0 - 21; // Line58 in heating_subsystem.ino

    error_temp = setTemp - currentTemp;
    KIinterror_temp += KI_temp * error_temp * deltat;
    KIinterror_temp = constrain(KIinterror_temp, 0, 3000);
    
    float heaterPower = round((kp * Te + KIIntTe) * kHeater); // Line70 in heating_subsystem.ino
    heaterPWM = constrain((int)heaterPower, 0, 1023); // Line71 in heating_subsystem.ino
    analogWrite(HEATER_PWM_PIN, heaterPWM);
    
    // pH CONTROL
    int pHRaw = analogRead(PH_SENSOR_PIN);
    float pHVoltage = pHRaw * KpH;
    currentpH = 7.0 - (pHVoltage - 2.5) * 2.0;
    
    error_pH = setpH - currentpH;
    
    if (abs(error_pH) > pH_deadband) {
      if (error_pH > 0) {
        KIinterror_pH += KI_pH * error_pH * deltat;
        KIinterror_pH = constrain(KIinterror_pH, 0, 255);
        basePumpPWM = constrain(Kp_pH * error_pH + KIinterror_pH, 0, 255);
        acidPumpPWM = 0;
      } else {
        KIinterror_pH += KI_pH * abs(error_pH) * deltat;
        KIinterror_pH = constrain(KIinterror_pH, 0, 255);
        acidPumpPWM = constrain(Kp_pH * abs(error_pH) + KIinterror_pH, 0, 255);
        basePumpPWM = 0;
      }
    } else {
      acidPumpPWM = 0;
      basePumpPWM = 0;
      KIinterror_pH = 0;
    }
    
    analogWrite(ACID_PUMP_PIN, acidPumpPWM);
    analogWrite(BASE_PUMP_PIN, basePumpPWM);
  }
  
  if (millis() - lastUartSend >= uartSendInterval) {
    sendUARTData();
    lastUartSend = millis();
  }
}

void sendUARTData() {
  String message = "PH:" + String(currentpH, 1) +
                   ",TEMP:" + String(currentTemp, 1) +
                   ",RPM:" + String((int)currentRPM);
  
  Serial1.println(message);
  Serial.print("[SENT] ");
  Serial.println(message);
}

void checkUARTCommands() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("SET:")) {
      int firstColon = cmd.indexOf(':');
      int secondColon = cmd.indexOf(':', firstColon + 1);
      
      if (secondColon > 0) {
        String param = cmd.substring(firstColon + 1, secondColon);
        float value = cmd.substring(secondColon + 1).toFloat();
        
        if (param == "RPM") {
          setRPM = constrain(value, 500, 1500);
          Serial.print("RPM setpoint: ");
          Serial.println(setRPM);
        }
        else if (param == "TEMP") {
          setTemp = constrain(value, 25, 35);
          Serial.print("Temp setpoint: ");
          Serial.println(setTemp);
        }
        else if (param == "PH") {
          setpH = constrain(value, 3, 7);
          Serial.print("pH setpoint: ");
          Serial.println(setpH);
        }
      }
    }
  }
}
