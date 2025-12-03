# Bioreactor Control System - Integration Documentation

**UCL ENGF0001 Engineering Challenges - Team 15**  
**Dashboard Leader: Louis**  
**Last Updated: 2025-12-03**

---

## System Architecture

```
Bioreactor Hardware
    ↕ (sensors/actuators)
Arduino Nano ESP32 (nano_controller.ino)
    ↕ (UART: GPIO 43/44)
TTGO T-Display (ttgo_gateway.ino)
    ↕ (WiFi/MQTT)
HiveMQ Broker (broker.hivemq.com)
    ↕ (WiFi/MQTT)
Node-RED Dashboard (laptop)
```

---

## Files Overview

### Production Code
1. **nano_controller.ino** - Arduino Nano ESP32
   - Reads sensors (hall sensor, thermistor, pH sensor)
   - Runs PI controllers for stirrer, heating, and pH
   - Controls actuators (motor, heater, acid/base pumps)
   - Sends sensor data via UART to TTGO
   - Receives setpoint commands via UART from TTGO

2. **ttgo_gateway.ino** - TTGO T-Display
   - Receives sensor data via UART from Nano
   - Connects to WiFi and MQTT broker
   - Publishes sensor data to MQTT topics
   - Subscribes to setpoint commands from MQTT
   - Forwards setpoints to Nano via UART
   - Displays real-time data on TFT screen

3. **flows.json** - Node-RED Dashboard 2.0
   - Professional dashboard with gauges, charts, and controls
   - Three monitoring groups: Stirrer Control, Temperature Control, pH Control
   - Controls group with sliders and submit/clear buttons
   - ESP32 state group showing active setpoints and sequence counter

---

## Hardware Connections

### Arduino Nano ESP32 ↔ TTGO T-Display (UART)

```
Nano ESP32          TTGO T-Display
TX (GPIO 43)   →    RX (GPIO 21)
RX (GPIO 44)   ←    TX (GPIO 22)
GND            ⟷    GND
```

**CRITICAL: Common ground connection required**

### Arduino Nano ESP32 Pin Configuration

```
STIRRER SUBSYSTEM:
- Hall Sensor:  GPIO 3 (INPUT_PULLUP, interrupt on RISING)
- Motor PWM:    GPIO 2 (OUTPUT)

HEATING SUBSYSTEM:
- Thermistor:   A0 (INPUT, 0-3.3V MAX)
- Heater PWM:   GPIO 10 (OUTPUT)

pH SUBSYSTEM:
- pH Sensor:    A1 (INPUT, 0-3.3V MAX)
- Acid Pump:    GPIO 6 (OUTPUT)
- Base Pump:    GPIO 7 (OUTPUT)
```

**WARNING: Arduino Nano ESP32 ADC is 3.3V maximum, not 5V**

---

## Software Setup

### 1. Arduino Nano ESP32

**Required Libraries:**
- Arduino ESP32 core (install via Board Manager)

**Upload Steps:**
1. Open `nano_controller.ino` in Arduino IDE
2. Select board: "Arduino Nano ESP32"
3. Verify pin assignments match your wiring
4. Upload to Arduino Nano ESP32
5. Open Serial Monitor (115200 baud) to verify operation

**Default Setpoints:**
- RPM: 1000
- Temperature: 35°C
- pH: 5.0

### 2. TTGO T-Display

**Required Libraries:**
- ESP32 Board Package (via Board Manager)
- PicoMQTT (via Library Manager)
- TFT_eSPI (via Library Manager)

**WiFi Configuration:**

Create `credentials.h` in same folder as `ttgo_gateway.ino`:

```cpp
#ifndef CREDENTIALS_H
#define CREDENTIALS_H

// For Eduroam (WPA2-Enterprise)
const char* WIFI_SSID = "eduroam";
const char* WIFI_USER = "your.email@ucl.ac.uk";
const char* WIFI_PASSWORD = "your_password";
const bool USE_ENTERPRISE = true;

// For Mobile Hotspot
// const char* WIFI_SSID = "YourHotspotName";
// const char* WIFI_USER = "";
// const char* WIFI_PASSWORD = "hotspot_password";
// const bool USE_ENTERPRISE = false;

#endif
```

**Upload Steps:**
1. Create `credentials.h` with your WiFi credentials
2. Open `ttgo_gateway.ino`
3. Select board: "ESP32 Dev Module" or "TTGO T1"
4. Upload to TTGO T-Display
5. Check Serial Monitor (115200 baud) for WiFi connection status

### 3. Node-RED Dashboard

**Installation:**
```bash
cd ~/.node-red
npm install @flowfuse/node-red-dashboard
node-red-stop
node-red
```

**Import Flow:**
1. Start Node-RED: `node-red`
2. Open browser: `http://localhost:1880`
3. Menu → Import → Select `flows.json`
4. Deploy
5. Access dashboard: `http://localhost:1880/dashboard`

**Dashboard Features:**
- Real-time gauges with color-coded ranges
- Time-series charts (1 hour data retention)
- Setpoint control sliders (Temp: 25-35°C, RPM: 500-1500, pH: 3-7)
- Submit/Clear buttons for setpoint management
- ESP32 state display with sequence counter

---

## Communication Protocol

### Nano → TTGO (UART)

**Format:** `PH:8.0,TEMP:22.2,RPM:1020`

**Frequency:** Every 2 seconds  
**Baud Rate:** 115200

**Example:**
```
PH:5.2,TEMP:30.5,RPM:1050
```

### TTGO → Nano (UART)

**Format:** `SET:PARAMETER:VALUE`

**Examples:**
```
SET:RPM:1200
SET:TEMP:32
SET:PH:6.5
```

### MQTT Topics

**Sensor Data (TTGO publishes, Node-RED subscribes):**
- `bioreactor/sensor/rpm` - Current motor speed (RPM)
- `bioreactor/sensor/temperature` - Current temperature (°C)
- `bioreactor/sensor/ph` - Current pH level

**Setpoint Commands (Node-RED publishes, TTGO subscribes):**
- `bioreactor/setpoint/rpm` - Target motor speed (500-1500 RPM)
- `bioreactor/setpoint/temperature` - Target temperature (25-35°C)
- `bioreactor/setpoint/ph` - Target pH level (3-7)

**MQTT Configuration:**
- Broker: broker.hivemq.com:1883
- Protocol: MQTT v5
- QoS: 0
- Publish Rate: Every 2 seconds
- Client IDs:
  - TTGO: Auto-generated
  - Node-RED: `nodered_bioreactor_client`

---

## PI Controller Implementation

### Stirrer (Motor Speed)
- Type: PI controller with hall sensor feedback
- Parameters: Kp=0.1, KI=0.01
- Setpoint range: 500-1500 RPM
- Sensor: Hall effect (70 pulses/revolution)
- Sample time: 1000ms
- Interrupt: RISING edge

### Heating (Temperature)
- Type: PI controller with thermistor feedback
- Parameters: Kp=1177, KI=0.0168
- Setpoint range: 25-35°C
- Sensor: NTC thermistor (β=3700)
- Sample time: 100ms
- Heater scaling: 1023 PWM → 3000W

### pH Control
- Type: PI controller with dual pump control
- Parameters: Kp=100, KI=5
- Setpoint range: 3-7 pH
- Deadband: ±0.1 pH
- Actuators: Acid pump (pH too high) / Base pump (pH too low)
- Sample time: 100ms

---

## Testing Procedure

### Phase 1: Hardware Connection
1. Connect Nano ↔ TTGO via 3 wires:
   - GND ⟷ GND
   - Nano TX (GPIO 43) → TTGO RX (GPIO 21)
   - Nano RX (GPIO 44) ← TTGO TX (GPIO 22)
2. Power both boards via USB

### Phase 2: UART Verification
1. Upload `nano_controller.ino` to Nano
2. Upload `ttgo_gateway.ino` to TTGO
3. Open Nano Serial Monitor (115200 baud)
   - Should see: `[SENT] PH:X.X,TEMP:XX.X,RPM:XXXX`
4. Open TTGO Serial Monitor (115200 baud)
   - Should see: `WiFi Connected IP: X.X.X.X`
   - Should see: `Received: XXX RPM, XX.X C, pH X.X`
5. Check TTGO TFT display shows sensor values

### Phase 3: MQTT Testing
1. Open MQTTX
2. Connect to `broker.hivemq.com:1883`
3. Subscribe to:
   - `bioreactor/sensor/rpm`
   - `bioreactor/sensor/temperature`
   - `bioreactor/sensor/ph`
   - `bioreactor/setpoint/rpm`
   - `bioreactor/setpoint/temperature`
   - `bioreactor/setpoint/ph`
4. Verify sensor data arriving every 2 seconds

### Phase 4: Dashboard Testing
1. Import `flows.json` into Node-RED
2. Deploy flow
3. Open `http://localhost:1880/dashboard`
4. Verify:
   - Gauges update with live data
   - Charts plot trends
   - Sliders control setpoints
   - ESP32 state displays show values
5. Test setpoint control:
   - Move slider → Click Submit
   - Check MQTTX sees setpoint message
   - Check Nano Serial shows "RPM setpoint: XXXX"

### Phase 5: Full System
1. Connect all physical sensors and actuators
2. Calibrate sensors if needed
3. Set initial setpoints via dashboard
4. Monitor system response
5. Verify PI controllers maintain setpoints

---

## Troubleshooting

### Nano Issues
- **No sensor readings:** Check pin assignments and wiring
- **ADC errors:** Ensure sensor outputs are 0-3.3V (use voltage dividers if needed)
- **Hall sensor not triggering:** Verify RISING edge interrupt, check magnet alignment

### TTGO Issues
- **No WiFi connection:**
  - Verify `credentials.h` exists with correct SSID/password
  - Try mobile hotspot instead of eduroam
  - Check Serial Monitor for connection status
- **Blank TFT screen:** Verify TFT_eSPI library configuration
- **No UART data:**
  - Check TX/RX crossover connection (TX→RX, RX→TX)
  - Verify common GND connection
  - Check baud rate matches (115200)

### MQTT Issues
- **Dashboard not updating:**
  - Verify Node-RED MQTT broker connection status
  - Check topic names match exactly (case-sensitive)
- **Setpoints not working:**
  - Verify TTGO subscriptions in Serial Monitor
  - Check MQTTX shows setpoint messages
  - Verify Nano receives commands via Serial Monitor

### Dashboard Issues
- **No data displayed:** Hardware not running or MQTT not connected
- **ESP32 state empty:** No setpoints published yet (move sliders and click Submit)
- **Charts not plotting:** Check chart configuration uses `msg.topic` and `msg.payload`

---

## Design Specifications

| Parameter | Range | Tolerance | Status |
|-----------|-------|-----------|--------|
| Stirring | 500-1500 RPM | ±20 RPM | ✅ |
| Temperature | 25-35°C | ±0.5°C | ✅ |
| pH | 3-7 | ±0.1 | ✅ |
| Data Logging | MQTT timestamped | - | ✅ |
| User Interface | Node-RED Dashboard | - | ✅ |
| Remote Control | Dashboard sliders | - | ✅ |

---

## File Structure

```
BIOREACTOR-SYSTEM/
├── .gitignore
├── credentials_template.h
├── README.md
│
├── nano_controller/
│   ├── nano_controller.ino
│   └── credentials.h (NOT COMMITTED)
│
├── ttgo_gateway/
│   ├── ttgo_gateway.ino
│   └── credentials.h (NOT COMMITTED)
│
└── nodered/
    └── flows.json
```

---

## Team Contributions

**Dashboard Leader (Louis):**
- Node-RED Dashboard 2.0 design and implementation
- MQTT integration and topic architecture
- System architecture coordination
- Integration testing
- Documentation

**EEE Team:**
- Hardware interfacing and wiring
- Sensor calibration
- Actuator control circuitry

**CS Team:**
- Embedded software (Nano + TTGO)
- UART communication protocol
- PI controller implementation

---

## Important Notes

1. **Data Format:** `PH:8.0,TEMP:22.2,RPM:1020`
2. **UART Pins:** Nano uses GPIO 43/44, TTGO uses GPIO 21/22
3. **ADC Voltage:** Arduino Nano ESP32 is 3.3V max (not 5V)
4. **Common Ground:** Required for UART communication
5. **credentials.h:** Must never be committed to git
6. **WiFi:** Works with both eduroam and mobile hotspot
7. **MQTT Broker:** Public HiveMQ (no authentication required)
8. **RPM Calculation:** (pulses * 60) / 70 magnets per revolution

---

## Contact & Support

For issues:
1. Check this README
2. Verify hardware connections (especially GND)
3. Check Serial Monitors for error messages
4. Use MQTTX to debug MQTT communication
5. Check Node-RED Debug sidebar

**System Status:** Production code ready for hardware integration  
**Last Updated:** 2025-12-03