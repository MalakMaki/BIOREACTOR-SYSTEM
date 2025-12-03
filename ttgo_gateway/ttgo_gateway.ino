// BIOREACTOR TTGO T-DISPLAY GATEWAY
// Format: PH:8.0,TEMP:22.2,RPM:101

#include <WiFi.h>
#include <PicoMQTT.h>
#include <TFT_eSPI.h>
#include "esp_eap_client.h"
#include "credentials.h"

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

const char* topic_rpm = "bioreactor/sensor/rpm";
const char* topic_temp = "bioreactor/sensor/temperature";
const char* topic_ph = "bioreactor/sensor/ph";
const char* topic_set_rpm = "bioreactor/setpoint/rpm";
const char* topic_set_temp = "bioreactor/setpoint/temperature";
const char* topic_set_ph = "bioreactor/setpoint/ph";

#define RXD2 21
#define TXD2 22

TFT_eSPI tft = TFT_eSPI();

float sensorRPM = 0.0;
float sensorTemp = 0.0;
float sensorpH = 0.0;
unsigned long lastDataReceived = 0;
const unsigned long dataTimeout = 5000;

PicoMQTT::Client mqtt(mqtt_server);

unsigned long lastMqttPublish = 0;
const unsigned long mqttPublishInterval = 2000;
unsigned long lastScreenUpdate = 0;
const unsigned long screenUpdateInterval = 500;

void onMqttMessage(const char* topic, const char* payload) {
  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(payload);
  
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

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("BIOREACTOR");
  tft.println("Starting...");
  
  connectWiFi();
  
  mqtt.subscribe(topic_set_rpm, onMqttMessage);
  mqtt.subscribe(topic_set_temp, onMqttMessage);
  mqtt.subscribe(topic_set_ph, onMqttMessage);
  mqtt.begin();
  
  Serial.println("TTGO Gateway Initialized");
}

void loop() {
  mqtt.loop();
  
  if (millis() - lastMqttPublish >= mqttPublishInterval) {
    publishMQTT();
    lastMqttPublish = millis();
  }
  
  if (millis() - lastScreenUpdate >= screenUpdateInterval) {
    updateDisplay();
    lastScreenUpdate = millis();
  }
  
  readUARTData();
}

void connectWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  
  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);
  
  if (USE_ENTERPRISE) {
    esp_eap_client_set_identity((uint8_t *)WIFI_USER, strlen(WIFI_USER));
    esp_eap_client_set_username((uint8_t *)WIFI_USER, strlen(WIFI_USER));
    esp_eap_client_set_password((uint8_t *)WIFI_PASSWORD, strlen(WIFI_PASSWORD));
    esp_wifi_sta_enterprise_enable();
    WiFi.begin(WIFI_SSID);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi Failed");
  }
}

void readUARTData() {
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    data.trim();
    
    int phIndex = data.indexOf("PH:");
    int tempIndex = data.indexOf("TEMP:");
    int rpmIndex = data.indexOf("RPM:");
    
    if (phIndex != -1 && tempIndex != -1 && rpmIndex != -1) {
      int phEnd = data.indexOf(",", phIndex);
      sensorpH = data.substring(phIndex + 3, phEnd).toFloat();
      
      int tempEnd = data.indexOf(",", tempIndex);
      sensorTemp = data.substring(tempIndex + 5, tempEnd).toFloat();
      
      sensorRPM = data.substring(rpmIndex + 4).toFloat();
      
      lastDataReceived = millis();
      
      Serial.print("Received: ");
      Serial.print(sensorRPM, 1);
      Serial.print(" RPM, ");
      Serial.print(sensorTemp, 1);
      Serial.print(" C, pH ");
      Serial.println(sensorpH, 1);
    }
  }
}

void publishMQTT() {
  if (WiFi.status() == WL_CONNECTED) {
    char buffer[16];
    
    dtostrf(sensorRPM, 6, 1, buffer);
    mqtt.publish(topic_rpm, buffer);
    
    dtostrf(sensorTemp, 6, 1, buffer);
    mqtt.publish(topic_temp, buffer);
    
    dtostrf(sensorpH, 6, 1, buffer);
    mqtt.publish(topic_ph, buffer);
    
    Serial.println("Published to MQTT");
  }
}

void updateDisplay() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(60, 50);
  tft.println("Sensor Monitor");
  
  tft.drawLine(60, 68, 180, 68, TFT_CYAN);
  
  bool dataStale = (millis() - lastDataReceived) > dataTimeout;
  
  if (dataStale) {
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(60, 80);
    tft.println("Waiting for data...");
  } else {
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(60, 80);
    tft.print("pH: ");
    tft.setTextColor(TFT_GREEN);
    tft.print(sensorpH, 1);
    
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(60, 92);
    tft.print("Temp: ");
    tft.setTextColor(TFT_ORANGE);
    tft.print(sensorTemp, 1);
    tft.print(" C");
    
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(60, 104);
    tft.print("RPM: ");
    tft.setTextColor(TFT_MAGENTA);
    tft.print((int)sensorRPM);
  }
  
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(60, 116);
  if (WiFi.status() == WL_CONNECTED) {
    tft.print("WiFi OK ");
    tft.print(WiFi.localIP().toString());
  } else {
    tft.print("WiFi: Disconnected");
  }
}
