// ============================================================================
// BIOREACTOR TTGO T-DISPLAY GATEWAY - INTEGRATED VERSION
// - Receives sensor data from Arduino Nano via UART
// - Displays data on TFT screen
// - Publishes to MQTT IMMEDIATELY when new data arrives
// - Receives setpoint commands from Node-RED and forwards to Nano
// Format: PH:8.0,TEMP:22.2,RPM:101
// ============================================================================

#include <WiFi.h>
#include <PicoMQTT.h>
#include <TFT_eSPI.h>
#include "esp_eap_client.h"

// ========== WiFi Configuration ==========
// Choose your WiFi type
#define USE_ENTERPRISE 1  // Set to 1 for eduroam, 0 for regular WiFi

#if USE_ENTERPRISE
  const char* WIFI_SSID = "eduroam";
  const char* WIFI_USER = ";      // Change this
  const char* WIFI_PASSWORD = "";  // Change this
#else
  const char* WIFI_SSID = "";          // Change this
  const char* WIFI_PASSWORD = "";     // Change this
#endif

// ========== MQTT Configuration ==========
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

// MQTT Topics
const char* topic_rpm = "bioreactor/sensor/rpm";
const char* topic_temp = "bioreactor/sensor/temperature";
const char* topic_ph = "bioreactor/sensor/ph";
const char* topic_set_rpm = "bioreactor/setpoint/rpm";
const char* topic_set_temp = "bioreactor/setpoint/temperature";
const char* topic_set_ph = "bioreactor/setpoint/ph";
const char* topic_heater = "bioreactor/control/heater";
const char* topic_motor = "bioreactor/control/motor";
const char* topic_phpump = "bioreactor/control/phpump";
const char* topic_status = "bioreactor/status";  // For acknowledgments

// ========== UART Configuration ==========
#define RXD2 21  // TTGO RX (from Nano TX on D1)
#define TXD2 22  // TTGO TX (to Nano RX on D0)

// ========== Display ==========
TFT_eSPI tft = TFT_eSPI();

// ========== Sensor Data ==========
float sensorRPM = 0.0;
float sensorTemp = 0.0;
float sensorpH = 0.0;
bool dataReceived = false;
unsigned long lastDataReceived = 0;
const unsigned long dataTimeout = 5000;

// ========== MQTT Client ==========
PicoMQTT::Client mqtt(mqtt_server);

// ========== Timing ==========
unsigned long lastScreenUpdate = 0;
const unsigned long screenUpdateInterval = 500;  // Update screen every 500ms

// ========== MQTT Message Handler ==========
void onMqttMessage(const char* topic, const char* payload) {
  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(payload);
  
  Serial.print("Forwarding to Nano via Serial2: ");
  
  // Forward setpoint commands to Arduino Nano
  if (strcmp(topic, topic_set_rpm) == 0) {
    Serial.print("SET:RPM:");
    Serial.println(payload);
    Serial2.print("SET:RPM:");
    Serial2.println(payload);
  } 
  else if (strcmp(topic, topic_set_temp) == 0) {
    Serial.print("SET:TEMP:");
    Serial.println(payload);
    Serial2.print("SET:TEMP:");
    Serial2.println(payload);
  }
  else if (strcmp(topic, topic_set_ph) == 0) {
    Serial.print("SET:PH:");
    Serial.println(payload);
    Serial2.print("SET:PH:");
    Serial2.println(payload);
  }
  // Forward control commands
  else if (strcmp(topic, topic_heater) == 0) {
    if (strcmp(payload, "ON") == 0) {
      Serial.println("HEATER:ON");
      Serial2.println("HEATER:ON");
    } else if (strcmp(payload, "OFF") == 0) {
      Serial.println("HEATER:OFF");
      Serial2.println("HEATER:OFF");
    }
  }
  else if (strcmp(topic, topic_motor) == 0) {
    if (strcmp(payload, "ON") == 0) {
      Serial.println("MOTOR:ON");
      Serial2.println("MOTOR:ON");
    } else if (strcmp(payload, "OFF") == 0) {
      Serial.println("MOTOR:OFF");
      Serial2.println("MOTOR:OFF");
    }
  }
  else if (strcmp(topic, topic_phpump) == 0) {
    if (strcmp(payload, "ON") == 0) {
      Serial.println("PHPUMP:ON");
      Serial2.println("PHPUMP:ON");
    } else if (strcmp(payload, "OFF") == 0) {
      Serial.println("PHPUMP:OFF");
      Serial2.println("PHPUMP:OFF");
    }
  }
}

// ========== WiFi Connection ==========
void connectWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  
  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);
  
  #if USE_ENTERPRISE
    esp_eap_client_set_identity((uint8_t *)WIFI_USER, strlen(WIFI_USER));
    esp_eap_client_set_username((uint8_t *)WIFI_USER, strlen(WIFI_USER));
    esp_eap_client_set_password((uint8_t *)WIFI_PASSWORD, strlen(WIFI_PASSWORD));
    esp_wifi_sta_enterprise_enable();
    WiFi.begin(WIFI_SSID);
  #else
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  #endif
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi Connection Failed");
  }
}

// ========== Parse UART Data ==========
void readUARTData() {
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    data.trim();
    
    Serial.print("UART Received: ");
    Serial.println(data);
    
    // Check if it's sensor data (PH:7.2,TEMP:25.5,RPM:105)
    int phIndex = data.indexOf("PH:");
    int tempIndex = data.indexOf("TEMP:");
    int rpmIndex = data.indexOf("RPM:");
    
    if (phIndex != -1 && tempIndex != -1 && rpmIndex != -1) {
      // Extract pH
      int phEnd = data.indexOf(",", phIndex);
      sensorpH = data.substring(phIndex + 3, phEnd).toFloat();
      
      // Extract Temperature
      int tempEnd = data.indexOf(",", tempIndex);
      sensorTemp = data.substring(tempIndex + 5, tempEnd).toFloat();
      
      // Extract RPM
      sensorRPM = data.substring(rpmIndex + 4).toFloat();
      
      dataReceived = true;
      lastDataReceived = millis();
      
      Serial.print("Parsed - pH: ");
      Serial.print(sensorpH, 1);
      Serial.print(" | Temp: ");
      Serial.print(sensorTemp, 1);
      Serial.print("°C | RPM: ");
      Serial.println((int)sensorRPM);
      
      // ✅ PUBLISH TO MQTT IMMEDIATELY WHEN NEW DATA ARRIVES
      publishMQTT();
    }
    // Check if it's an acknowledgment (ACK:TEMP:38.0)
    else if (data.startsWith("ACK:")) {
      Serial.print("✓ Acknowledgment from Nano: ");
      Serial.println(data);
      
      // Optionally publish ACK to MQTT status topic
      if (WiFi.status() == WL_CONNECTED) {
        mqtt.publish(topic_status, data.c_str());
        Serial.println("ACK published to MQTT status topic");
      }
    }
    // Check if it's an error (ERROR:TEMP:OUT_OF_RANGE)
    else if (data.startsWith("ERROR:")) {
      Serial.print("✗ Error from Nano: ");
      Serial.println(data);
      
      // Optionally publish error to MQTT status topic
      if (WiFi.status() == WL_CONNECTED) {
        mqtt.publish(topic_status, data.c_str());
        Serial.println("Error published to MQTT status topic");
      }
    }
    // Unknown format
    else {
      Serial.print("⚠ Unknown format from Nano: ");
      Serial.println(data);
    }
  }
}

// ========== Publish to MQTT ==========
void publishMQTT() {
  if (WiFi.status() == WL_CONNECTED && dataReceived) {
    char buffer[16];
    
    // Publish RPM
    dtostrf(sensorRPM, 6, 1, buffer);
    mqtt.publish(topic_rpm, buffer);
    
    // Publish Temperature
    dtostrf(sensorTemp, 6, 1, buffer);
    mqtt.publish(topic_temp, buffer);
    
    // Publish pH
    dtostrf(sensorpH, 6, 1, buffer);
    mqtt.publish(topic_ph, buffer);
    
    Serial.println("✓ Published to MQTT immediately");
  }
}

// ========== Update Display ==========
void updateDisplay() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  
  const int OX = 60;  // Offset X
  const int OY = 50;  // Offset Y
  
  // Title
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(OX, OY);
  tft.println("Sensor Monitor");
  
  // Separator line
  tft.drawLine(OX, OY + 18, OX + 120, OY + 18, TFT_CYAN);
  
  // Check if data is stale
  bool dataStale = (millis() - lastDataReceived) > dataTimeout;
  
  if (!dataReceived || dataStale) {
    // Waiting for data
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(OX, OY + 30);
    tft.println("Waiting for data...");
  } else {
    // Display pH
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(OX, OY + 30);
    tft.print("pH: ");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print(sensorpH, 1);
    
    // Display Temperature
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(OX, OY + 42);
    tft.print("Temp: ");
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.print(sensorTemp, 1);
    tft.print(" C");
    
    // Display RPM
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(OX, OY + 54);
    tft.print("RPM: ");
    tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
    tft.print((int)sensorRPM);
  }
  
  // WiFi Status
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(OX, OY + 66);
  if (WiFi.status() == WL_CONNECTED) {
    tft.print("WiFi: ");
    tft.print(WiFi.localIP().toString());
  } else {
    tft.print("WiFi: Disconnected");
  }
  
  // MQTT Status
  tft.setCursor(OX, OY + 78);
  if (mqtt.connected()) {
    tft.print("MQTT: Connected");
  } else {
    tft.print("MQTT: Disconnected");
  }
}

// ========== SETUP ==========
void setup() {
  // USB Serial for debugging
  Serial.begin(115200);
  Serial.println("\n=== TTGO Bioreactor Gateway ===");
  Serial.println("Real-Time MQTT Publishing Version");
  
  // Configure backlight
  #ifndef TFT_BL
    #define TFT_BL 4
  #endif
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  
  // Initialize display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(60, 50);
  tft.println("BIOREACTOR");
  tft.setCursor(60, 62);
  tft.println("Starting...");
  
  // Connect to WiFi
  connectWiFi();
  delay(500);
  
  // Initialize UART (Nano <-> TTGO)
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("UART initialized on RX=21, TX=22 @ 115200");
  
  // Subscribe to MQTT topics
  mqtt.subscribe(topic_set_rpm, onMqttMessage);
  mqtt.subscribe(topic_set_temp, onMqttMessage);
  mqtt.subscribe(topic_set_ph, onMqttMessage);
  mqtt.subscribe(topic_heater, onMqttMessage);
  mqtt.subscribe(topic_motor, onMqttMessage);
  mqtt.subscribe(topic_phpump, onMqttMessage);
  
  // Start MQTT client
  mqtt.begin();
  
  Serial.println("TTGO Gateway Initialized");
  Serial.println("Real-time MQTT publishing enabled");
  Serial.println("Subscribed to MQTT topics:");
  Serial.println("  - " + String(topic_set_rpm));
  Serial.println("  - " + String(topic_set_temp));
  Serial.println("  - " + String(topic_set_ph));
  Serial.println("  - " + String(topic_heater));
  Serial.println("  - " + String(topic_motor));
  Serial.println("  - " + String(topic_phpump));
  Serial.println();
  
  // Initial display update
  updateDisplay();
}

// ========== MAIN LOOP ==========
void loop() {
  // Handle MQTT
  mqtt.loop();
  
  // Read data from Arduino Nano (and publish immediately if sensor data)
  readUARTData();
  
  // Update display periodically
  if (millis() - lastScreenUpdate >= screenUpdateInterval) {
    updateDisplay();
    lastScreenUpdate = millis();
  }
  
  // Small delay to prevent watchdog issues
  delay(10);
}
