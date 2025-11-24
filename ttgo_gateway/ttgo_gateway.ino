// ============================================================================
// BIOREACTOR TTGO T-DISPLAY GATEWAY
// Receives data from Nano via UART, publishes to MQTT, displays on TFT
// Uses PicoMQTT and supports both simple and enterprise WiFi
// ============================================================================

#include <WiFi.h>
#include <PicoMQTT.h>
#include <TFT_eSPI.h>
#include "esp_eap_client.h"  // For enterprise WiFi (eduroam)

// FOR EDUROAM (uncomment and use these instead):
const char* ssid = "eduroam";
const char* user = "zcabnlm@ucl.ac.uk";
const char* password = "CocoLia2006!?!";
const bool useEnterpriseWiFi = true;

// ========== MQTT CONFIGURATION ==========
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

// MQTT Topics
const char* topic_rpm = "bioreactor/sensor/rpm";
const char* topic_temp = "bioreactor/sensor/temperature";
const char* topic_ph = "bioreactor/sensor/ph";
const char* topic_set_rpm = "bioreactor/setpoint/rpm";
const char* topic_set_temp = "bioreactor/setpoint/temperature";
const char* topic_set_ph = "bioreactor/setpoint/ph";

// ========== UART CONFIGURATION ==========
// TTGO T-Display UART pins (verify with your board pinout)
#define RXD2 16
#define TXD2 17

// ========== TFT DISPLAY ==========
TFT_eSPI tft = TFT_eSPI();

// ========== DATA VARIABLES ==========
float sensorRPM = 0.0;
float sensorTemp = 0.0;
float sensorpH = 0.0;
unsigned long lastDataReceived = 0;
const unsigned long dataTimeout = 5000; // 5s timeout

// ========== MQTT CLIENT (PicoMQTT) ==========
PicoMQTT::Client mqtt(mqtt_server);

// ========== TIMING ==========
unsigned long lastMqttPublish = 0;
const unsigned long mqttPublishInterval = 2000; // Publish every 2s
unsigned long lastScreenUpdate = 0;
const unsigned long screenUpdateInterval = 500; // Update screen every 500ms

// ========== MQTT CALLBACK ==========
void onMqttMessage(const char* topic, const char* payload) {
  Serial.print("MQTT received [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(payload);
  
  // Forward setpoint commands to Nano via UART
  if (strcmp(topic, topic_set_rpm) == 0) {
    Serial2.print("SET:RPM:");
    Serial2.println(payload);
  } 
  else if (strcmp(topic, topic_set_temp) == 0) {
    Serial2.print("SET:TEMP:");
    Serial2.println(payload);
  }
  else if (strcmp(topic, topic_set_ph) == 0) {
    Serial2.print("SET:PH:");
    Serial2.println(payload);
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);  // USB debug
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // UART from Nano
  
  // Initialize TFT
  tft.init();
  tft.setRotation(1); // Landscape
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println("BIOREACTOR");
  tft.println("Starting...");
  
  // Connect to WiFi
  connectWiFi();
  
  // Setup MQTT with PicoMQTT
  mqtt.subscribe(topic_set_rpm, onMqttMessage);
  mqtt.subscribe(topic_set_temp, onMqttMessage);
  mqtt.subscribe(topic_set_ph, onMqttMessage);
  
  mqtt.begin();  // Start MQTT client
  
  Serial.println("TTGO Gateway Initialized");
}

// ========== MAIN LOOP ==========
void loop() {
  // Maintain WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  
  // PicoMQTT handles connection automatically
  mqtt.loop();
  
  // Read UART data from Nano
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
  tft.setCursor(0, 0);
  tft.println("WiFi...");
  
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  
  if (useEnterpriseWiFi) {
    // Enterprise WiFi (eduroam/WPA2-Enterprise) - using new API
    esp_eap_client_set_identity((uint8_t *)user, strlen(user));
    esp_eap_client_set_username((uint8_t *)user, strlen(user));
    esp_eap_client_set_password((uint8_t *)password, strlen(password));
    esp_wifi_sta_enterprise_enable();
    WiFi.begin(ssid);
  } else {
    // Simple WiFi (WPA2-Personal)
    WiFi.begin(ssid, password);
  }
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.setTextSize(1);
    tft.println("WiFi OK");
    tft.println(WiFi.localIP());
    tft.setTextSize(2);
    delay(1000);
  } else {
    Serial.println("\nWiFi Failed!");
    tft.fillScreen(TFT_RED);
    tft.setCursor(0, 0);
    tft.println("WiFi FAIL");
    delay(2000);
  }
}

// ========== READ UART DATA ==========
void readUARTData() {
  // Expected format: RPM,TEMP,PH\n
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
      
      Serial.print("Received: ");
      Serial.print(sensorRPM);
      Serial.print(" RPM, ");
      Serial.print(sensorTemp);
      Serial.print(" °C, pH ");
      Serial.println(sensorpH);
    }
  }
}

// ========== PUBLISH TO MQTT ==========
void publishMQTT() {
  if (WiFi.status() == WL_CONNECTED) {
    char buffer[16];
    
    dtostrf(sensorRPM, 6, 1, buffer);
    mqtt.publish(topic_rpm, buffer);
    
    dtostrf(sensorTemp, 6, 2, buffer);
    mqtt.publish(topic_temp, buffer);
    
    dtostrf(sensorpH, 6, 2, buffer);
    mqtt.publish(topic_ph, buffer);
    
    Serial.println("Published to MQTT");
  }
}

// ========== UPDATE DISPLAY ==========
void updateDisplay() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  
  // Header
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(5, 5);
  tft.println("BIOREACTOR");
  
  // Connection status
  tft.setTextSize(1);
  tft.setCursor(5, 30);
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(TFT_GREEN);
    tft.print("WiFi OK");
  } else {
    tft.setTextColor(TFT_RED);
    tft.print("WiFi OFF");
  }
  
  tft.setCursor(80, 30);
  if (mqtt.connected()) {
    tft.setTextColor(TFT_GREEN);
    tft.print("MQTT OK");
  } else {
    tft.setTextColor(TFT_RED);
    tft.print("MQTT OFF");
  }
  
  // Data timeout check
  bool dataStale = (millis() - lastDataReceived) > dataTimeout;
  
  // RPM
  tft.setTextSize(2);
  tft.setTextColor(dataStale ? TFT_DARKGREY : TFT_YELLOW);
  tft.setCursor(5, 50);
  tft.print("RPM:");
  tft.setCursor(80, 50);
  tft.print(sensorRPM, 1);
  
  // Temperature
  tft.setCursor(5, 75);
  tft.setTextColor(dataStale ? TFT_DARKGREY : TFT_ORANGE);
  tft.print("Temp:");
  tft.setCursor(80, 75);
  tft.print(sensorTemp, 1);
  tft.print("C");
  
  // pH
  tft.setCursor(5, 100);
  tft.setTextColor(dataStale ? TFT_DARKGREY : TFT_MAGENTA);
  tft.print("pH:");
  tft.setCursor(80, 100);
  tft.print(sensorpH, 2);
}
