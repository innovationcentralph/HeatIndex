#include "SystemConfig.h"
#include <EEPROM.h>
#include <Arduino.h>

// MQTT configuration variables

char deviceESN[DEVICE_ESN_MAX_LEN] = "";

MQTTConfig_t mqttConfig;

MQTTConfig_t defaultMQTTConfig = {
  "3.27.210.100",
  "mqtt",
  "ICPHmqtt!",
  1883
};

ThresholdConfig_t thresholdConfig;
ThresholdConfig_t defaultThresholdConfig = {
  55,  // heatIndexCritical
  50,  // heatIndexAlert
  45,  // heatIndexWarning
  40   // heatIndexNormal
};

String initTopic;
String intervalConfig;
String mqttConfiguration;
String willTopic;
String willMessage;
String thresholdConfiguration;




// ==================== Threshold EEPROM ====================
void loadThresholdConfigFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_THRESHOLD_ADDR, thresholdConfig);

  // Check for uninitialized EEPROM (0xFFFF for ints)
  if (thresholdConfig.heatIndexCritical == 0xFFFF) {
    thresholdConfig = defaultThresholdConfig;
  }
}

void saveThresholdConfigToEEPROM(const ThresholdConfig_t& config) {
  EEPROM.put(EEPROM_THRESHOLD_ADDR, config);
  EEPROM.commit();
}

void loadMQTTConfigFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_MQTT_CONFIG_ADDR, mqttConfig);

  // If EEPROM is not initialized, fall back to default
  if (strlen(mqttConfig.mqttServer) == 0 || mqttConfig.mqttServer[0] == 0xFF) {
    mqttConfig = defaultMQTTConfig;
  }

  // Ensure valid port
  if (mqttConfig.mqttPort <= 0 || mqttConfig.mqttPort > 65535) {
    mqttConfig.mqttPort = defaultMQTTConfig.mqttPort;
  }
}

void saveMQTTConfigToEEPROM(const MQTTConfig_t& config) {
  EEPROM.put(EEPROM_MQTT_CONFIG_ADDR, config);
  EEPROM.commit();
}

void loadDeviceESNFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ESN_ADDR, deviceESN);

  // Check if uninitialized
  if (strlen(deviceESN) == 0 || deviceESN[0] == 0xFF) {
    strncpy(deviceESN, "HeatIndex", DEVICE_ESN_MAX_LEN);  // ------------------------------------------------> need to be default
  }
}

void saveDeviceESNToEEPROM(const char* esn) {
  strncpy(deviceESN, esn, DEVICE_ESN_MAX_LEN);
  EEPROM.put(EEPROM_ESN_ADDR, deviceESN);
  EEPROM.commit();
}

void initializeDynamicTopics() {
  willTopic = "status/last_will";
  willMessage = String(deviceESN);
  initTopic = "RHT/" + String(deviceESN) + "/init";
  intervalConfig = "RHT/" + String(deviceESN) + "/interval";
  mqttConfiguration = "RHT/" + String(deviceESN) + "/mqtt";
  thresholdConfiguration = "RHT/" + String(deviceESN) + "/threshold";  // <-- New
}