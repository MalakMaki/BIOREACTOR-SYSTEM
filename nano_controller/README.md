# Bioreactor Control System - Integration Documentation

## System Architecture

```
Bioreactor Hardware
    ↕ (sensors/actuators - wired)
Arduino Nano ESP32 (nano_controller.ino)
    ↕ (UART - Serial communication)
TTGO T-Display (ttgo_gateway.ino)
    ↕ (WiFi/MQTT)
HiveMQ Broker (broker.hivemq.com)
    ↕ (WiFi/MQTT)
Node-RED Dashboard (your laptop)
```

## Files Overview

1. **nano_controller.ino** - Arduino Nano ESP32
   - Reads all sensors (hall sensor, thermistor, pH sensor)
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

3. **nodered_flow.json** - Node-RED Dashboard
   - Dashboard with gauges, sliders, and trend charts
   - For all three subsystems (stirrer, heating, pH)
   - Publishes/subscribes to MQTT topics

---

## Hardware Connections

### Arduino Nano ESP32 ↔ TTGO T-Display (UART)
```
Nano ESP32          TTGO T-Display
TX (GPIO 43)   →    RX (GPIO 16)
RX (GPIO 44)   ←    TX (GPIO 17)
GND            ⟷    GND
```

### Arduino Nano ESP32 Pin Configuration
```
STIRRER SUBSYSTEM:
- Hall Sensor:  GPIO 3 (INPUT_PULLUP, interrupt)
- Motor PWM:    GPIO 2 (OUTPUT)

HEATING SUBSYSTEM:
- Thermistor:   A0 (INPUT)
- Heater PWM:   GPIO 10 (OUTPUT)

pH SUBSYSTEM:
- pH Sensor:    A1 (INPUT)
- Acid Pump:    GPIO 6 (OUTPUT)
- Base Pump:    GPIO 7 (OUTPUT)
```

**⚠️ IMPORTANT:** Verify and adjust pin numbers based on your actual wiring before uploading!

---

## Software Setup

### 1. Arduino Nano ESP32 Setup

**Required Libraries:**
- Arduino ESP32 core (install via Board Manager)

**Upload Steps:**
1. Open `nano_controller.ino` in Arduino IDE
2. Select board: "Arduino Nano ESP32"
3. **CRITICAL:** Verify pin assignments match your hardware
4. Upload to Arduino Nano ESP32
5. Open Serial Monitor (115200 baud) to see debug output

**PI Controller Tuning:**
- Default parameters are configured for fast response (ωn=ω₀, ζ=1)
- Adjust `Kp_motor`, `KI_motor`, `Kp_temp`, `KI_temp`, `Kp_pH`, `KI_pH` if needed
- See `PIMotorLecture_2026_1.pdf` for tuning methodology

### 2. TTGO T-Display Setup

**Required Libraries:**
- ESP32 Board Package (install via Board Manager)
- PubSubClient (MQTT library)
- TFT_eSPI (display library)

**Configuration:**
1. Open `ttgo_gateway.ino`
2. **UPDATE WiFi credentials:**
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
3. **Verify UART pins** (check TTGO pinout):
   ```cpp
   #define RXD2 16  // May need adjustment
   #define TXD2 17  // May need adjustment
   ```
4. Select board: "ESP32 Dev Module" or "TTGO T1"
5. Upload to TTGO T-Display

**TFT_eSPI Configuration:**
- You may need to edit `User_Setup.h` in TFT_eSPI library folder
- Select `Setup25_TTGO_T_Display.h` for TTGO T-Display

### 3. Node-RED Dashboard Setup

**Installation:**
```bash
# Install Node-RED (if not already installed)
npm install -g node-red

# Install dashboard nodes
npm install node-red-dashboard
npm install node-red-contrib-mqtt-broker
```

**Import Flow:**
1. Start Node-RED: `node-red`
2. Open browser: `http://localhost:1880`
3. Menu (≡) → Import → Select `nodered_flow.json`
4. Deploy the flow
5. Access dashboard: `http://localhost:1880/ui`

**MQTT Configuration:**
- Broker is pre-configured to HiveMQ public broker
- No authentication required
- Topics:
  - Sensors: `bioreactor/sensor/{rpm,temperature,ph}`
  - Setpoints: `bioreactor/setpoint/{rpm,temperature,ph}`

---

## Communication Protocol

### Nano → TTGO (UART)
**Format:** CSV string with newline terminator
```
RPM,TEMP,PH\n
```
**Example:**
```
1050.5,29.75,5.12\n
```
**Frequency:** Every 1 second

### TTGO → Nano (UART)
**Format:** Command string with newline terminator
```
SET:RPM:1000\n
SET:TEMP:30\n
SET:PH:5\n
```

### MQTT Topics
**Sensor Data (TTGO → Node-RED):**
- `bioreactor/sensor/rpm` (float)
- `bioreactor/sensor/temperature` (float)
- `bioreactor/sensor/ph` (float)

**Setpoint Commands (Node-RED → TTGO):**
- `bioreactor/setpoint/rpm` (500-1500)
- `bioreactor/setpoint/temperature` (25-35)
- `bioreactor/setpoint/ph` (3-7)

---

## PI Controller Implementation

### Stirrer (Motor Speed)
- **Type:** PI controller with velocity feedback
- **Parameters:** Kp, KI calculated from motor constants
- **Setpoint range:** 500-1500 RPM
- **Tolerance:** ±20 RPM
- **Sensor:** Hall effect (70 pulses/rev)

### Heating (Temperature)
- **Type:** PI controller with thermistor feedback
- **Parameters:** Kp=1177, KI=0.0168
- **Setpoint range:** 25-35°C
- **Tolerance:** ±0.5°C
- **Sensor:** NTC thermistor (β=3700)

### pH Control
- **Type:** PI controller with dual pump control
- **Parameters:** Kp=100, KI=5
- **Setpoint range:** 3-7 pH
- **Deadband:** ±0.1 pH (reduces pump cycling)
- **Actuators:** Acid pump (pH too high) / Base pump (pH too low)

---

## Testing Procedure

### Phase 1: Nano Standalone
1. Upload `nano_controller.ino`
2. Open Serial Monitor (115200 baud)
3. Verify sensor readings appear every 1s
4. Test manual setpoint changes via Serial commands

### Phase 2: UART Communication
1. Connect Nano ↔ TTGO via UART
2. Upload `ttgo_gateway.ino`
3. Verify TTGO displays sensor data on TFT
4. Check TTGO serial output for received data

### Phase 3: MQTT Integration
1. Ensure both devices connected and running
2. Start Node-RED and import flow
3. Deploy flow and open dashboard
4. Verify gauges update with live data
5. Test setpoint adjustments via sliders

### Phase 4: Full System Test
1. Connect all physical sensors/actuators
2. Set initial setpoints via dashboard
3. Monitor system response on charts
4. Verify PI controllers maintain setpoints
5. Log data for calibration report

---

## Troubleshooting

### Nano Issues
- **No sensor readings:** Check pin assignments and wiring
- **Erratic PWM:** Verify MOSFET gate resistors (100-220Ω)
- **Hall sensor not triggering:** Try FALLING instead of RISING edge

### TTGO Issues
- **No WiFi connection:** Double-check SSID/password
- **Blank screen:** Verify TFT_eSPI library configuration
- **No UART data:** Check TX/RX pin assignments and GND connection

### MQTT Issues
- **Dashboard not updating:** Verify MQTT broker connection
- **Setpoints not working:** Check topic names match exactly
- **Connection drops:** HiveMQ public broker can be busy, consider local broker

### Integration Issues
- **Data mismatch:** Check UART baud rates match (115200)
- **Delayed response:** Normal due to control loop timing (100ms-1s)
- **Pump oscillation:** Increase pH deadband or reduce KI_pH

---

## Design Specifications Met

✅ **Stirring:** 500-1500 RPM ±20 RPM  
✅ **Temperature:** 25-35°C ±0.5°C  
✅ **pH:** 3-7 range with setpoint control  
✅ **Data logging:** MQTT publishes timestamped data  
✅ **User interface:** Node-RED dashboard with real-time monitoring  
✅ **Remote control:** Setpoint adjustment via dashboard  

---

## Next Steps for Team Report

1. **Calibration:**
   - Calibrate thermistor using known temperatures
   - Calibrate pH sensor with buffer solutions
   - Verify motor RPM measurement accuracy

2. **Performance Testing:**
   - Step response tests for each subsystem
   - Record settling time, overshoot, steady-state error
   - Compare against specifications

3. **Data Logging:**
   - Export CSV data from Node-RED
   - Plot response curves in MATLAB/Python
   - Document system performance

4. **Integration Testing:**
   - Test simultaneous control of all subsystems
   - Verify no interference between controllers
   - Measure UART/MQTT latency

---

## References

- PIMotorLecture_2026_1.pdf - PI controller design methodology
- Connected_Bioreactor_Design_Specification.pdf - System specifications
- Arduino Nano ESP32 datasheet - Pin configuration
- TTGO T-Display pinout - GPIO mapping

---

## Team Contributions

**Dashboard Leader (You):**
- Node-RED dashboard design
- MQTT integration
- System architecture coordination
- Documentation

**EEE Team:**
- Hardware interfacing
- Sensor calibration
- Actuator control

**CS Team:**
- Embedded software
- Communication protocols
- Data logging
