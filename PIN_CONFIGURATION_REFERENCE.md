# Complete Pin Configuration Reference

## 🎯 Arduino Nano ESP32 - Pin Assignments

### Code Pin Configuration (from nano_controller.ino):

```cpp
// ========== PIN CONFIGURATION ==========
// STIRRER SUBSYSTEM
const byte HALL_SENSOR_PIN = 3;    // Hall effect sensor input (interrupt capable)
const byte MOTOR_PWM_PIN = 2;      // Motor PWM output

// HEATING SUBSYSTEM  
const byte TEMP_SENSOR_PIN = A0;   // Thermistor analog input
const byte HEATER_PWM_PIN = 10;    // Heater PWM output

// pH SUBSYSTEM
const byte PH_SENSOR_PIN = A1;     // pH sensor analog input
const byte ACID_PUMP_PIN = 6;      // Acid pump PWM output
const byte BASE_PUMP_PIN = 7;      // Base pump PWM output

// UART TO TTGO
// TX1 = GPIO 43 (hardware serial)
// RX1 = GPIO 44 (hardware serial)
// Configured in setup: Serial1.begin(115200, SERIAL_8N1, 44, 43);
```

---

## 📋 Full Arduino Nano ESP32 Pinout

```
        Arduino Nano ESP32
        ┌─────────────┐
        │   USB-C     │
        └─────────────┘

LEFT SIDE:              RIGHT SIDE:
D13/SCK                 3V3
D12/MISO                AREF (internal reference)
D11/MOSI                A0  ← Thermistor
D10/SS   ← Heater PWM   A1  ← pH Sensor
D9                      A2
D8                      A3
D7       ← Base Pump    A4/SDA (I2C)
D6       ← Acid Pump    A5/SCL (I2C)
D5                      A6
D4                      A7
D3       ← Hall Sensor  5V (output when on USB)
D2       ← Motor PWM    RST (reset)
GND                     GND
RST                     VIN (external power input)
RX0 (GPIO 44) ← UART    TX0 (GPIO 43) → UART
```

### Important Notes:

1. **PWM Capable Pins:** Almost all digital pins support PWM on ESP32-S3
   - D2, D6, D7, D10 ✅ All good for PWM

2. **Interrupt Capable Pins:** All GPIO pins support interrupts
   - D3 ✅ Good for Hall sensor

3. **Analog Inputs:**
   - A0-A7 are 12-bit ADC (0-4095)
   - Input range: 0-3.3V (NOT 5V!)
   - ⚠️ **Use voltage divider if sensor outputs >3.3V**

4. **Serial Ports:**
   - Serial (USB): GPIO1/GPIO3 - Used for programming & debug
   - Serial1 (UART): GPIO43/GPIO44 - Used for TTGO communication
   - Don't mix them up!

5. **Power:**
   - Logic level: 3.3V
   - 5V pin: Available when powered via USB
   - Max current per pin: 40mA
   - Total max current all pins: 500mA (USB limit)

---

## 🎯 TTGO T-Display - Pin Assignments

### Code Pin Configuration (from ttgo_gateway.ino):

```cpp
// ========== UART CONFIGURATION ==========
#define RXD2 16  // Receives data FROM Nano TX1
#define TXD2 17  // Sends data TO Nano RX1

// Configured in setup: Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
```

---

## 📋 Full TTGO T-Display Pinout

```
       TTGO T-Display
       ┌─────────────┐
       │   USB-C     │
       │             │
       │  ┌───────┐  │
       │  │ TFT   │  │  ← 1.14" Display (135×240)
       │  │Screen │  │
       │  └───────┘  │
       │  [Button1]  │  ← GPIO 0 (Boot button)
       │  [Button2]  │  ← GPIO 35
       └─────────────┘

LEFT SIDE:                RIGHT SIDE:
GND                       GND
3V3                       GND
GPIO 1 (TX0 - USB)        GPIO 27
GPIO 3 (RX0 - USB)        GPIO 26
GPIO 22                   GPIO 25
GPIO 21                   GPIO 33
GPIO 17 → TX2 (to Nano)   GPIO 32
GPIO 16 ← RX2 (from Nano) GPIO 15 (TFT_BL - backlight)
GPIO 4  (TFT_RST)         GPIO 2
GPIO 5  (TFT_MISO)        GPIO 0 (Boot button)
GPIO 18 (TFT_SCK)         GPIO 4 (TFT_CS)
GPIO 19 (TFT_MOSI)        GPIO 23 (TFT_DC)
GPIO 13                   GND
GND                       5V
```

### TTGO Pins Used by TFT Display:
```cpp
// These pins are RESERVED - don't use them!
#define TFT_MISO 5
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS   5
#define TFT_DC   16  // Note: Conflicts with common RX2!
#define TFT_RST  23
#define TFT_BL   4   // Backlight
```

⚠️ **CRITICAL CONFLICT ISSUE:**
Some TTGO boards use GPIO 16 for TFT_DC, which conflicts with RX2!

### Alternative UART Pins (if GPIO 16/17 don't work):

Try these instead:
```cpp
#define RXD2 21  // Alternative RX
#define TXD2 22  // Alternative TX
```

Or:
```cpp
#define RXD2 25
#define TXD2 26
```

---

## 🔌 UART Connection Summary

```
Arduino Nano ESP32          TTGO T-Display
──────────────────          ──────────────
GPIO 43 (TX1) ─────────────→ GPIO 16 (RX2)  *see note below
GPIO 44 (RX1) ←───────────── GPIO 17 (TX2)
GND           ←─────────────→ GND (MUST CONNECT!)

* If GPIO 16 doesn't work, try GPIO 21 or GPIO 25
```

---

## ⚠️ CRITICAL Pin Warnings

### For Arduino Nano ESP32:

1. **Voltage Levels:**
   - ADC inputs: MAX 3.3V (not 5V!)
   - Use voltage divider for 5V sensors
   
2. **Current Limits:**
   - Don't drive motors/heaters/pumps directly from GPIO
   - Use MOSFETs with gate resistors (100-220Ω)
   - Add flyback diodes for inductive loads

3. **Reserved Pins:**
   - GPIO 0: Boot mode (avoid using)
   - GPIO 1/3: USB serial (leave for debug)

### For TTGO T-Display:

1. **Display Pins - DON'T USE:**
   - GPIO 4, 5, 16, 18, 19, 23 (reserved for TFT)
   - If you need these, disable display

2. **Button Pins:**
   - GPIO 0: Boot button (can use, but careful)
   - GPIO 35: User button

3. **UART Pin Conflicts:**
   - If RX2 doesn't work on GPIO 16, it's because TFT uses it
   - Solution: Use alternative pins (GPIO 21/22)

---

## 🔧 How to Change UART Pins

### If GPIO 16/17 Don't Work on TTGO:

In `ttgo_gateway.ino`, change lines 35-36:

```cpp
// Original (may conflict with TFT):
#define RXD2 16
#define TXD2 17

// Alternative 1:
#define RXD2 21
#define TXD2 22

// Alternative 2:
#define RXD2 25
#define TXD2 26
```

Then update your breadboard wiring accordingly!

---

## 📊 Pin Usage Summary Table

### Arduino Nano ESP32:
| Pin | Function | Type | Notes |
|-----|----------|------|-------|
| D2 | Motor PWM | Digital Output | PWM for stirrer motor |
| D3 | Hall Sensor | Digital Input | Interrupt for RPM sensing |
| D6 | Acid Pump | Digital Output | PWM for pH down |
| D7 | Base Pump | Digital Output | PWM for pH up |
| D10 | Heater PWM | Digital Output | PWM for heating |
| A0 | Thermistor | Analog Input | 0-3.3V, 12-bit ADC |
| A1 | pH Sensor | Analog Input | 0-3.3V, 12-bit ADC |
| GPIO 43 | TX to TTGO | UART TX | Serial1 |
| GPIO 44 | RX from TTGO | UART RX | Serial1 |

### TTGO T-Display:
| Pin | Function | Type | Notes |
|-----|----------|------|-------|
| GPIO 16 | RX from Nano | UART RX | Serial2, may conflict with TFT |
| GPIO 17 | TX to Nano | UART TX | Serial2 |
| GPIO 21 | RX (alt) | UART RX | Use if GPIO 16 conflicts |
| GPIO 22 | TX (alt) | UART TX | Use if GPIO 17 conflicts |
| GPIO 0 | Boot Button | Input | Can use for user input |
| GPIO 4-23 | TFT Display | SPI | Reserved, don't use |

---

## 🧪 Testing Pin Functionality

### Test Hall Sensor (D3):
```cpp
void setup() {
  pinMode(3, INPUT_PULLUP);
  Serial.begin(115200);
}
void loop() {
  Serial.println(digitalRead(3));
  delay(100);
}
// Should toggle when magnet passes
```

### Test Thermistor (A0):
```cpp
void setup() {
  Serial.begin(115200);
}
void loop() {
  int raw = analogRead(A0);
  float voltage = raw * (3.3 / 4095.0);
  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print(" Voltage: ");
  Serial.println(voltage);
  delay(500);
}
// Should show stable readings ~1.5-2.5V at room temp
```

### Test UART (Both boards):
Upload blink code to both, then check UART with logic analyzer or oscilloscope.

---

## 💡 Quick Reference Card

**For Testing UART Communication:**
```
Nano → TTGO:
- TX1 (GPIO 43) → RX2 (GPIO 16 or 21)
- RX1 (GPIO 44) ← TX2 (GPIO 17 or 22)
- GND ← → GND
```

**For Adding Sensors:**
```
Stirrer:  D3 (Hall) + D2 (Motor)
Heating:  A0 (Temp) + D10 (Heater)  
pH:       A1 (pH)   + D6 (Acid) + D7 (Base)
```

**Voltage Reminder:**
```
⚠️ ADC inputs: MAX 3.3V!
⚠️ Use voltage dividers for 5V sensors
⚠️ Use MOSFETs + gate resistors for motors/pumps
```
