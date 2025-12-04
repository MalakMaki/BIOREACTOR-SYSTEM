//Make sure to calibrate KpH against substances of known pH

const byte pHPin = A1;
const float KpH = 3.3/4095; // ADC conversion: 12-bit ADC (0-4095) to voltage (0-3.3V)

// Calibration constants
const float voltage_pH4 = 0.592;   // Voltage reading at pH 4 (acidic)
const float voltage_pH7 = 1.582;   // Voltage reading at pH 7 (neutral)
const float voltage_pH10 = 2.524; // Voltage reading at pH 10 (alkaline)

// Motor pump pins
const byte motorA_pin = 5;  // D5 - Motor A (for adding acid/lowering pH) - PWM capable
const byte motorB_pin = 4;  // D4 - Motor B (for adding base/raising pH) - PWM capable

// pH control parameters
float target_pH = 7.0;      // Target pH value (adjustable via serial input)

// PI Controller parameters
const float Kp = 80.0;      // Proportional gain
const float Ki = 5.0;       // Integral gain
const float deadband = 0.02; // Deadband to prevent unnecessary corrections

// Motor speed threshold and offset - OPTIMIZED FOR FINE CONTROL
const int MIN_MOTOR_SPEED = 10;   // Minimum PI output before motor turns off
const int MOTOR_OFFSET = 70;      // REDUCED from 100 for finer control
const int MAX_MOTOR_SPEED = 255;  // Maximum PWM value

float voltage, pH, pHsmoothed;
int Timems, T2;

// PI controller variables
float integral = 0.0;
unsigned long lastTime = 0;
int motorA_pwm = 0;
int motorB_pwm = 0;

void setup() {
  pinMode(pHPin, INPUT);
  pinMode(motorA_pin, OUTPUT);
  pinMode(motorB_pin, OUTPUT);
  
  // Ensure motors are off at startup
  analogWrite(motorA_pin, 0);
  analogWrite(motorB_pin, 0);
  
  Serial.begin(2000000);
  Serial.println("pH Meter with 3-Point Calibration & Motor Control");
  Serial.println("Calibration points:");
  Serial.print("  pH 4:  "); Serial.print(voltage_pH4, 3); Serial.println(" V");
  Serial.print("  pH 7:  "); Serial.print(voltage_pH7, 3); Serial.println(" V");
  Serial.print("  pH 10: "); Serial.print(voltage_pH10, 3); Serial.println(" V");
  Serial.print("Target pH: "); Serial.println(target_pH, 1);
  Serial.print("Tolerance: ±"); Serial.println(deadband, 3);
  Serial.print("Kp: "); Serial.print(Kp); Serial.print(" | Ki: "); Serial.println(Ki);
  Serial.print("Min Motor Speed: "); Serial.print(MIN_MOTOR_SPEED); Serial.println(" (0-255)");
  Serial.print("Motor Offset: "); Serial.print(MOTOR_OFFSET); Serial.println(" (starting PWM)");
  Serial.print("Motor PWM Range: "); Serial.print(MOTOR_OFFSET + MIN_MOTOR_SPEED); 
  Serial.print(" to "); Serial.println(MAX_MOTOR_SPEED);
  Serial.println("---");
  
  lastTime = millis();
}

void loop() {
  // Check for serial input to change target pH
  if (Serial.available() > 0) {
    float newTarget = Serial.parseFloat();
    
    // Clear any remaining characters in the buffer
    while(Serial.available() > 0) {
      Serial.read();
    }
    
    // Validate and set new target pH
    if (newTarget >= 0 && newTarget <= 14) {
      target_pH = newTarget;
      integral = 0.0;  // Reset integral term when target changes
      Serial.println("---");
      Serial.print("New Target pH: "); Serial.println(target_pH, 2);
      Serial.println("---");
    }
  }
  
  // Read voltage from sensor
  voltage = KpH * analogRead(pHPin);
  
  // Convert voltage to pH using piecewise linear calibration
  if (voltage >= voltage_pH7) {
    // Alkaline range: between pH 7 and pH 10
    float slope = (10.0 - 7.0) / (voltage_pH10 - voltage_pH7);
    pH = 7.0 + slope * (voltage - voltage_pH7);
  } 
  else {
    // Acidic range: between pH 4 and pH 7
    float slope = (7.0 - 4.0) / (voltage_pH7 - voltage_pH4);
    pH = 4.0 + slope * (voltage - voltage_pH4);
  }
  
  // Constrain pH to reasonable range
  pH = constrain(pH, 0, 14);
  
  // IIR smoothing to reduce noise
  pHsmoothed = 0.9 * pHsmoothed + 0.1 * pH;
  
  // PI control logic
  controlMotorsPI(pHsmoothed);
  
  Timems = millis();
  if(Timems - T2 > 0) {
    T2 = T2 + 1000; // Print pH value once per second
    Serial.print("Voltage: ");
    Serial.print(voltage, 3);
    Serial.print(" V | pH: ");
    Serial.print(pH, 2);
    Serial.print(" | Smoothed pH: ");
    Serial.print(pHsmoothed, 2);
    Serial.print(" | Error: ");
    Serial.print(pHsmoothed - target_pH, 3);
    Serial.print(" | Integral: ");
    Serial.print(integral, 2);
    Serial.print(" | Motor A PWM: ");
    Serial.print(motorA_pwm);
    Serial.print(" | Motor B PWM: ");
    Serial.println(motorB_pwm);
  }
}

void controlMotorsPI(float currentpH) {
  // Calculate time delta
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
  if (dt <= 0) dt = 0.001; // Prevent division by zero
  lastTime = currentTime;
  
  // Calculate error
  float error = currentpH - target_pH;
  
  // Apply deadband ONLY to stop motors, NOT to stop integral accumulation
  if (abs(error) < deadband) {
    // Within deadband - turn off both motors but DON'T reset integral
    motorA_pwm = 0;
    motorB_pwm = 0;
    analogWrite(motorA_pin, 0);
    analogWrite(motorB_pin, 0);
    return;
  }
  
  // Update integral term (with anti-windup) - ONLY accumulate when outside deadband
  integral += error * dt;
  integral = constrain(integral, -20.0, 20.0);  // Anti-windup limits
  
  // Calculate PI control output
  float output = Kp * error + Ki * integral;
  
  // Convert output to motor PWM values (0-255) and apply minimum speed threshold with offset
  if (output > 0) {
    // pH is too HIGH - activate Motor A to add acid
    int temp_pwm = constrain((int)output, 0, MAX_MOTOR_SPEED - MOTOR_OFFSET);
    
    // Apply minimum speed threshold and add offset
    if (temp_pwm > 0 && temp_pwm < MIN_MOTOR_SPEED) {
      motorA_pwm = 0;  // Turn off if below minimum speed
    } else if (temp_pwm >= MIN_MOTOR_SPEED) {
      motorA_pwm = temp_pwm + MOTOR_OFFSET;  // Add offset to ensure motor actually runs
      motorA_pwm = constrain(motorA_pwm, MOTOR_OFFSET, MAX_MOTOR_SPEED);
    } else {
      motorA_pwm = 0;
    }
    
    motorB_pwm = 0;
  }
  else if (output < 0) {
    // pH is too LOW - activate Motor B to add base
    int temp_pwm = constrain((int)(-output), 0, MAX_MOTOR_SPEED - MOTOR_OFFSET);
    
    // Apply minimum speed threshold and add offset
    if (temp_pwm > 0 && temp_pwm < MIN_MOTOR_SPEED) {
      motorB_pwm = 0;  // Turn off if below minimum speed
    } else if (temp_pwm >= MIN_MOTOR_SPEED) {
      motorB_pwm = temp_pwm + MOTOR_OFFSET;  // Add offset to ensure motor actually runs
      motorB_pwm = constrain(motorB_pwm, MOTOR_OFFSET, MAX_MOTOR_SPEED);
    } else {
      motorB_pwm = 0;
    }
    
    motorA_pwm = 0;
  }
  else {
    // Within deadband - turn off both motors
    motorA_pwm = 0;
    motorB_pwm = 0;
  }
  
  // Apply PWM to motors
  analogWrite(motorA_pin, motorA_pwm);
  analogWrite(motorB_pin, motorB_pwm);
}
