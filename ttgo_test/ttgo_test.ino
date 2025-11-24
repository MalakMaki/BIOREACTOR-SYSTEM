// ============================================================================
// TTGO T-DISPLAY - UART TEST VERSION
// Receives and displays UART data from Nano - Tests communication
// ============================================================================

#include <WiFi.h>
#include <PicoMQTT.h>
#include <TFT_eSPI.h>
#include "esp_eap_client.h"

// ========== WiFi CONFIGURATION ==========
// TESTING: Mobile Hotspot Configuration
const char* ssid = "YOUR_PHONE_HOTSPOT";   // ← CHANGE THIS
const char* user = "";
const char* password = "YOUR_HOTSPOT_PASS"; // ← CHANGE THIS
const bool useEnterpriseWiFi = false;

// ========== MQTT CONFIGURATION ==========
const char* mqtt_server = "broker.hivemq.com";
const char* topic_rpm = "bioreactor/sensor/rpm";
const char* topic_temp = "bioreactor/sensor/temperature";
const char* topic_ph = "bioreactor/sensor/ph";
const char* topic_set_rpm = "bioreactor/setpoint/rpm";
const char* topic_set_temp = "bioreactor/setpoint/temperature";
const char* topic_set_ph = "bioreactor/setpoint/ph";

// ========== UART CONFIGURATION ==========
#define RXD2 16
#define TXD2 17

// ========== TFT DISPLAY ==========
TFT_eSPI tft = TFT_eSPI();

// ========== DATA VARIABLES ==========
float sensorRPM = 0.0;
float sensorTemp = 0.0;
float sensorpH = 0.0;
unsigned long lastDataReceived = 0;
unsigned long dataReceivedCount = 0;
const unsigned long dataTimeout = 5000;

// ========== MQTT CLIENT ==========
PicoMQTT::Client mqtt(mqtt_server);

// ========== TIMING ==========
unsigned long lastMqttPublish = 0;
const unsigned long mqttPublishInterval = 2000;
unsigned long lastScreenUpdate = 0;
const unsigned long screenUpdateInterval = 500;

// ========== MQTT CALLBACK ==========
void onMqttMessage(const char* topic, const char* payload) {
  Serial.print("[MQTT IN] ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);
  
  // Forward setpoint commands to Nano via UART
  Serial.print("[UART OUT] ");
  if (strcmp(topic, topic_set_rpm) == 0) {
    Serial2.print("SET:RPM:");
    Serial2.println(payload);
    Serial.print("SET:RPM:");
    Serial.println(payload);
  } 
  else if (strcmp(topic, topic_set_temp) == 0) {
    Serial2.print("SET:TEMP:");
    Serial2.println(payload);
    Serial.print("SET:TEMP:");
    Serial.println(payload);
  }
  else if (strcmp(topic, topic_set_ph) == 0) {
    Serial2.print("SET:PH:");
    Serial2.println(payload);
    Serial.print("SET:PH:");
    Serial.println(payload);
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println("╔═══════════════════════════════════════╗");
  Serial.println("║  TTGO T-DISPLAY - UART TEST MODE     ║");
  Serial.println("║  Waiting for data from Nano...       ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println();
  
  // Initialize TFT
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("UART TEST");
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.println("Waiting for Nano...");
  
  // Connect to WiFi
  connectWiFi();
  
  // Setup MQTT
  mqtt.subscribe(topic_set_rpm, onMqttMessage);
  mqtt.subscribe(topic_set_temp, onMqttMessage);
  mqtt.subscribe(topic_set_ph, onMqttMessage);
  mqtt.begin();
  
  Serial.println("Setup complete. Listening on UART...");
  Serial.println("Expected format: RPM,TEMP,PH");
  Serial.println();
}

// ========== MAIN LOOP ==========
void loop() {
  // Maintain WiFi
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  
  // MQTT loop
  mqtt.loop();
  
  // Read UART data
  readUARTData();
  
  // Publish to MQTT
  if (millis() - lastMqttPublish >= mqttPublishInterval) {
    lastMqttPublish = millis();
    publishMQTT();
  }
  
  // Update display
  if (millis() - lastScreenUpdate >= screenUpdateInterval) {
    lastScreenUpdate = millis();
    updateDisplay();
  }
}

// ========== WiFi CONNECTION ==========
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.setTextSize(1);
  tft.print("WiFi: ");
  tft.println(ssid);
  
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  
  if (useEnterpriseWiFi) {
    esp_eap_client_set_identity((uint8_t *)user, strlen(user));
    esp_eap_client_set_username((uint8_t *)user, strlen(user));
    esp_eap_client_set_password((uint8_t *)password, strlen(password));
    esp_wifi_sta_enterprise_enable();
    WiFi.begin(ssid);
  } else {
    WiFi.begin(ssid, password);
  }
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    tft.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✓ WiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    tft.println("\nConnected!");
    tft.println(WiFi.localIP());
    delay(1000);
  } else {
    Serial.println("✗ WiFi Failed!");
    tft.fillScreen(TFT_RED);
    tft.setCursor(10, 50);
    tft.setTextSize(2);
    tft.println("WiFi FAIL");
  }
}

// ========== READ UART DATA ==========
void readUARTData() {
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    data.trim();
    
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    
    if (firstComma > 0 && secondComma > firstComma) {
      sensorRPM = data.substring(0, firstComma).toFloat();
      sensorTemp = data.substring(firstComma + 1, secondComma).toFloat();
      sensorpH = data.substring(secondComma + 1).toFloat();
      
      lastDataReceived = millis();
      dataReceivedCount++;
      
      // Print to Serial Monitor with clear formatting
      Serial.print("[UART IN #");
      Serial.print(dataReceivedCount);
      Serial.print("] ");
      Serial.print(sensorRPM, 1);
      Serial.print(" RPM, ");
      Serial.print(sensorTemp, 2);
      Serial.print(" °C, pH ");
      Serial.println(sensorpH, 2);
    } else {
      Serial.print("[UART ERROR] Invalid format: ");
      Serial.println(data);
    }
  }
}

// ========== PUBLISH TO MQTT ==========
void publishMQTT() {
  if (WiFi.status() == WL_CONNECTED && dataReceivedCount > 0) {
    char buffer[16];
    
    dtostrf(sensorRPM, 6, 1, buffer);
    mqtt.publish(topic_rpm, buffer);
    
    dtostrf(sensorTemp, 6, 2, buffer);
    mqtt.publish(topic_temp, buffer);
    
    dtostrf(sensorpH, 6, 2, buffer);
    mqtt.publish(topic_ph, buffer);
    
    Serial.println("[MQTT OUT] Published sensor data");
  }
}

// ========== UPDATE DISPLAY ==========
void updateDisplay() {
  tft.fillScreen(TFT_BLACK);
  
  // Title
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(5, 5);
  tft.println("UART TEST");
  
  // Status line
  tft.setTextSize(1);
  tft.setCursor(5, 30);
  
  // WiFi status
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(TFT_GREEN);
    tft.print("WiFi:OK ");
  } else {
    tft.setTextColor(TFT_RED);
    tft.print("WiFi:-- ");
  }
  
  // MQTT status
  if (mqtt.connected()) {
    tft.setTextColor(TFT_GREEN);
    tft.print("MQTT:OK");
  } else {
    tft.setTextColor(TFT_ORANGE);
    tft.print("MQTT:--");
  }
  
  // Data received counter
  tft.setCursor(5, 45);
  tft.setTextColor(TFT_WHITE);
  tft.print("Packets: ");
  tft.println(dataReceivedCount);
  
  // Check if data is stale
  bool dataStale = (millis() - lastDataReceived) > dataTimeout;
  
  if (dataReceivedCount == 0) {
    // No data received yet
    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(5, 70);
    tft.println("Waiting for");
    tft.setCursor(5, 90);
    tft.println("Nano data...");
  } else {
    // Display sensor values
    tft.setTextSize(2);
    tft.setTextColor(dataStale ? TFT_DARKGREY : TFT_YELLOW);
    tft.setCursor(5, 65);
    tft.print("RPM:");
    tft.setCursor(100, 65);
    tft.print(sensorRPM, 1);
    
    tft.setCursor(5, 85);
    tft.setTextColor(dataStale ? TFT_DARKGREY : TFT_ORANGE);
    tft.print("Tmp:");
    tft.setCursor(100, 85);
    tft.print(sensorTemp, 1);
    
    tft.setCursor(5, 105);
    tft.setTextColor(dataStale ? TFT_DARKGREY : TFT_MAGENTA);
    tft.print("pH:");
    tft.setCursor(100, 105);
    tft.print(sensorpH, 2);
    
    // Show stale warning
    if (dataStale) {
      tft.setTextSize(1);
      tft.setTextColor(TFT_RED);
      tft.setCursor(5, 125);
      tft.println("DATA TIMEOUT!");
    }
  }
}

// ============================================================================
// TESTING NOTES:
// 
// 1. Update WiFi credentials at the top (lines 19-21)
// 2. Upload this to TTGO T-Display
// 3. Open Serial Monitor (115200 baud)
// 4. You should see:
//    - "TTGO T-DISPLAY - UART TEST MODE" header
//    - WiFi connection attempts
//    - "[UART IN #X] Y RPM, Z °C, pH W" when data arrives from Nano
//
// 5. On TTGO screen:
//    - "UART TEST" header
//    - WiFi/MQTT status
//    - Packet counter (increases with each message from Nano)
//    - Live sensor values
//
// 6. If you see "Waiting for Nano data..." on screen:
//    - Check UART wiring (TX→RX crossover)
//    - Check GND connection
//    - Verify Nano is sending (check Nano Serial Monitor)
//
// ============================================================================
