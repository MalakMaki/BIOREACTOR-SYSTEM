# UART Communication Testing Guide

## 🎯 Purpose

These simplified test versions let you verify UART communication works **WITHOUT** needing any physical sensors, motors, or pumps connected. Perfect for breadboard testing!

---

## 📦 Test Files

### 1. `nano_test.ino` - Arduino Nano ESP32
**What it does:**
- Generates **fake sensor data** (simulated RPM, temperature, pH)
- Sends data to TTGO via UART every 1 second
- Receives setpoint commands from TTGO
- Values change slowly to simulate real sensors

**No hardware needed!** Just upload and run.

### 2. `ttgo_test.ino` - TTGO T-Display  
**What it does:**
- Receives data from Nano via UART
- Displays data on TFT screen with packet counter
- Connects to WiFi and publishes to MQTT
- Forwards setpoint commands from Node-RED to Nano

**Only needs:** WiFi hotspot credentials (no sensors)

---

## 🚀 Quick Start - Step by Step

### Step 1: Breadboard Wiring (3 wires only!)

```
Arduino Nano ESP32          TTGO T-Display
──────────────────          ──────────────
GPIO 43 (TX1)   ──────────→ GPIO 16 (RX2)
GPIO 44 (RX1)   ←────────── GPIO 17 (TX2)
GND             ←──────────→ GND

Both powered via USB cables
```

**Visual Check:**
- ✅ GND connected to GND?
- ✅ TX goes to RX (crossover)?
- ✅ RX comes from TX (crossover)?

---

### Step 2: Configure TTGO WiFi

Open `ttgo_test.ino` and edit lines 19-21:

```cpp
const char* ssid = "Your_Phone_Hotspot";      // ← Your hotspot name
const char* user = "";                         // Leave empty
const char* password = "your_hotspot_pass";    // ← Your hotspot password
const bool useEnterpriseWiFi = false;          // Must be false
```

---

### Step 3: Upload Nano Test Code

1. **IMPORTANT:** Create a folder called `nano_test/`
2. Put `nano_test.ino` inside that folder
3. Open `nano_test.ino` from Arduino IDE
4. Select board: **"Arduino Nano ESP32"**
5. Connect Nano via USB
6. Upload

---

### Step 4: Upload TTGO Test Code

1. **IMPORTANT:** Create a folder called `ttgo_test/`
2. Put `ttgo_test.ino` inside that folder
3. Open `ttgo_test.ino` from Arduino IDE
4. Select board: **"ESP32 Dev Module"**
5. Connect TTGO via USB
6. Upload

**Note:** You may need to install **PicoMQTT** and **TFT_eSPI** libraries first.

---

### Step 5: Wire It Up & Test!

1. **Disconnect** TX/RX wires temporarily
2. Upload both codes
3. **Reconnect** TX/RX wires
4. Connect **both** devices via USB
5. Turn on your **phone hotspot**

---

## 🔍 What You Should See

### Nano Serial Monitor (115200 baud):

```
╔═══════════════════════════════════════╗
║  NANO ESP32 - UART TEST MODE         ║
║  Generating Fake Sensor Data         ║
╚═══════════════════════════════════════╝

Setup complete. Sending test data every 1s...
Format: RPM,TEMP,PH

[SENT to TTGO] 1000.0 RPM, 30.00 °C, pH 5.00
[SENT to TTGO] 1001.0 RPM, 30.12 °C, pH 5.05
[SENT to TTGO] 1002.0 RPM, 30.24 °C, pH 5.08
...
```

### TTGO Serial Monitor (115200 baud):

```
╔═══════════════════════════════════════╗
║  TTGO T-DISPLAY - UART TEST MODE     ║
║  Waiting for data from Nano...       ║
╚═══════════════════════════════════════╝

Connecting to WiFi: Your_Phone_Hotspot
......
✓ WiFi Connected!
IP: 192.168.43.100

Setup complete. Listening on UART...
Expected format: RPM,TEMP,PH

[UART IN #1] 1000.0 RPM, 30.00 °C, pH 5.00
[UART IN #2] 1001.0 RPM, 30.12 °C, pH 5.05
[UART IN #3] 1002.0 RPM, 30.24 °C, pH 5.08
[MQTT OUT] Published sensor data
...
```

### TTGO Screen Display:

```
┌─────────────────────────┐
│ UART TEST               │
│ WiFi:OK MQTT:OK         │
│ Packets: 23             │
│                         │
│ RPM: 1023.0             │
│ Tmp: 30.45              │
│ pH:  5.12               │
└─────────────────────────┘
```

---

## ✅ Success Criteria

### You know it's working when:

1. **Nano sends data:**
   - ✅ Serial Monitor shows "[SENT to TTGO]" every 1s
   - ✅ Values are changing slowly

2. **TTGO receives data:**
   - ✅ Serial Monitor shows "[UART IN #X]" increasing
   - ✅ Screen shows "Packets: X" increasing
   - ✅ Screen shows live values (not "Waiting for Nano...")

3. **WiFi connected:**
   - ✅ TTGO screen shows "WiFi:OK"
   - ✅ Serial Monitor shows IP address

4. **MQTT working:**
   - ✅ TTGO screen shows "MQTT:OK"
   - ✅ Serial Monitor shows "[MQTT OUT] Published sensor data"

---

## ❌ Troubleshooting

### Problem: TTGO shows "Waiting for Nano data..."

**Possible Causes:**
1. TX/RX wires not connected
2. TX/RX wires not crossed over (TX→RX)
3. GND not connected
4. Wrong UART pins used

**Fix:**
```
Check wiring:
✓ Nano TX (GPIO 43) → TTGO RX (GPIO 16)
✓ Nano RX (GPIO 44) ← TTGO TX (GPIO 17)
✓ Nano GND → TTGO GND

Try alternative TTGO pins if GPIO 16/17 don't work:
In ttgo_test.ino, change:
#define RXD2 21
#define TXD2 22
```

---

### Problem: Nano sends but TTGO shows garbage/wrong values

**Possible Causes:**
1. Baud rate mismatch
2. Loose connection
3. Data format error

**Fix:**
```
Verify baud rates match:
- Nano:  Serial1.begin(115200, ...)
- TTGO:  Serial2.begin(115200, ...)

Check connections are firm
Try shorter wires
```

---

### Problem: WiFi won't connect

**Possible Causes:**
1. Wrong SSID/password
2. Hotspot not active
3. Out of range

**Fix:**
```
1. Double-check hotspot name and password
2. Make sure phone hotspot is ON
3. Move TTGO closer to phone
4. Try different WiFi credentials
```

---

### Problem: MQTT shows "--" on screen

**Possible Causes:**
1. No internet on hotspot
2. HiveMQ broker down (rare)
3. Firewall blocking

**Fix:**
```
1. Check phone has mobile data
2. Try pinging broker.hivemq.com
3. Wait a minute and it should reconnect
```

---

## 🧪 Testing Setpoint Commands

Once UART and MQTT are working, test bidirectional communication:

### From Node-RED Dashboard:

1. Open Node-RED dashboard: `http://localhost:1880/ui`
2. Move the **RPM slider** to 1200
3. Check **Nano Serial Monitor** - should show:
   ```
   [RECEIVED from TTGO] SET:RPM:1200
   ✓ RPM setpoint updated: 1200.0
   ```
4. Check **TTGO Serial Monitor** - should show:
   ```
   [MQTT IN] bioreactor/setpoint/rpm: 1200
   [UART OUT] SET:RPM:1200
   ```

### Test All Three:
- Move **RPM slider** → Nano updates RPM
- Move **Temp slider** → Nano updates temperature
- Move **pH slider** → Nano updates pH

---

## 📊 Data Flow Verification

Complete system test:

```
Nano generates fake data
    ↓ (UART)
TTGO receives data
    ↓ (WiFi/MQTT)
Node-RED receives data
    ↓ (displays on dashboard)
User moves slider
    ↓ (MQTT)
TTGO receives setpoint
    ↓ (UART)
Nano receives setpoint
    ↓ (Serial Monitor confirms)
```

---

## 🎓 Next Steps

Once this test works perfectly:

1. ✅ UART communication verified
2. ✅ WiFi/MQTT verified
3. ✅ Bidirectional control verified

**Then you can:**
- Replace `nano_test.ino` with `nano_controller.ino`
- Replace `ttgo_test.ino` with `ttgo_gateway.ino`
- Add physical sensors one by one
- Test each subsystem individually

---

## 💡 Pro Tips

### Tip 1: Use Both Serial Monitors
Open TWO Arduino IDE windows:
- One for Nano Serial Monitor
- One for TTGO Serial Monitor

Watch data flow in real-time!

### Tip 2: Packet Counter
The "Packets: X" on TTGO screen is super useful:
- Should increase by 1 every second
- If it stops, UART connection broken
- If it's erratic, check wiring

### Tip 3: Color Coding
On TTGO screen:
- **Green** = Working fine
- **Orange** = MQTT not connected (WiFi works, broker issue)
- **Red** = WiFi failed
- **Grey** = Data timeout (no data from Nano for 5s)

### Tip 4: Debugging Commands
You can send commands manually via Nano Serial Monitor:
1. Type: `SET:RPM:1200`
2. This simulates what TTGO would send
3. Good for testing Nano's command parser

---

## 📸 Expected Outputs Gallery

### Successful Nano Output:
```
[SENT to TTGO] 1045.0 RPM, 31.23 °C, pH 5.15
[SENT to TTGO] 1046.0 RPM, 31.25 °C, pH 5.16
[RECEIVED from TTGO] SET:RPM:1200
✓ RPM setpoint updated: 1200.0
[SENT to TTGO] 1200.0 RPM, 31.27 °C, pH 5.17
```

### Successful TTGO Output:
```
[UART IN #45] 1045.0 RPM, 31.23 °C, pH 5.15
[UART IN #46] 1046.0 RPM, 31.25 °C, pH 5.16
[MQTT OUT] Published sensor data
[MQTT IN] bioreactor/setpoint/rpm: 1200
[UART OUT] SET:RPM:1200
[UART IN #47] 1200.0 RPM, 31.27 °C, pH 5.17
```

---

## ⚡ Quick Troubleshooting Checklist

Before asking for help, check:

- [ ] Both devices have power (LEDs on)?
- [ ] GND wire connected?
- [ ] TX→RX crossover correct?
- [ ] RX←TX crossover correct?
- [ ] Baud rates both 115200?
- [ ] Correct folders (separate for each .ino)?
- [ ] Libraries installed (PicoMQTT, TFT_eSPI)?
- [ ] WiFi credentials correct?
- [ ] Phone hotspot active?
- [ ] Both Serial Monitors open and set to 115200?

---

## 🎉 When Everything Works

You'll see:
- ✅ Nano sending data every second
- ✅ TTGO receiving data every second  
- ✅ TTGO screen showing live values
- ✅ Node-RED dashboard updating
- ✅ Sliders controlling setpoints
- ✅ Setpoints appearing in Nano Serial Monitor

**Congratulations!** Your UART communication and full system architecture is working perfectly. You're ready to add real sensors!
