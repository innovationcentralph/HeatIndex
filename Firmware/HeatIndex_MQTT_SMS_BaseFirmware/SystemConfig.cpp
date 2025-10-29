#include "SystemConfig.h"
#include <EEPROM.h>
#include <Arduino.h>

// Gateway Heartbeat Interval
uint32_t hbInterval = 0;

// MQTT configuration variables
char deviceESN[DEVICE_ESN_MAX_LEN] = "";
MQTTConfig_t mqttConfig;
MQTTConfig_t defaultMQTTConfig = {"3.27.210.100", "mqtt", "ICPHmqtt!", 1883 };

Thresholds_t thresholds;
TowerLight_t towerLights;

String willTopic = "";
String willMessage = "";
String hbConfig = "";

String initTopic;
String intervalConfig;
String mqttConfiguration;
String topicPing;
String topicPingResp;

String topicSensorData;

String topicRequestThreshold;
String topicThresholds;

String topicRequestContacts;
String topicContacts;

String topicRequestDateTime;
String topicGetDateTime;


void loadDeviceESNFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ESN_ADDR, deviceESN);

  // Check if uninitialized
  if (strlen(deviceESN) == 0 || deviceESN[0] == 0xFF) {
    strncpy(deviceESN, "XXX-XX-XX", DEVICE_ESN_MAX_LEN);
  }
}

void saveDeviceESNToEEPROM(const char* esn) {
  strncpy(deviceESN, esn, DEVICE_ESN_MAX_LEN);
  EEPROM.put(EEPROM_ESN_ADDR, deviceESN);
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


void initializeDynamicTopics() {
  willTopic     = "status/last_will";
  willMessage   = String(deviceESN);

  initTopic     = String(deviceESN) + "/init";

  topicPing     = String(deviceESN) + "/ping";;
  topicPingResp = String(deviceESN) + "/ping/resp";

  topicSensorData = String(deviceESN) + "/SensorData";

  topicRequestThreshold = String(deviceESN) + "/RequestThresholds";
  topicThresholds       = String(deviceESN) + "/Thresholds";

  topicRequestContacts  = String(deviceESN) + "/RequestContacts";
  topicContacts         = String(deviceESN) + "/Contacts";

  topicRequestDateTime  = String(deviceESN) + "/RequestDateTime";
  topicGetDateTime      = String(deviceESN) + "/DateTime";
  
}


void saveHeartbeatInterval(uint32_t interval) {

  if (interval == 0 || interval > 3600000) {  // Set a default if the value is invalid
    interval = 60000;                         // Default to 60 seconds
    EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, interval);
    Serial.println(F("[CLI] Invalid value. Default heartbeat interval set to 60000 ms."));
  } else {
    EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, interval);
    Serial.print(F("[CLI] Heartbeat interval saved: "));
    Serial.print(interval);
    Serial.println(F(" ms"));
  }
  EEPROM.commit();

}

uint32_t loadHeartbeatInterval() {
  uint32_t interval;
  EEPROM.get(EEPROM_SEND_INTERVAL_ADDR, interval);
  if (interval == 0 || interval > 3600000) {  // Set a default if the value is invalid
    interval = 60000;                         // Default to 60 seconds
    EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, interval);
    EEPROM.commit();
    Serial.println(F("[CLI] Invalid stored value. Default heartbeat interval restored to 60000 ms."));
  }else{
    Serial.print(F("[CLI] Loaded heartbeat interval: "));
    Serial.print(interval);
    Serial.println(F(" ms"));
  }
  return interval;
}



void saveWiFiCredsToEEPROM(const WiFiCreds_t& creds) {
  EEPROM.put(EEPROM_WIFI_CRED_ADDR, creds);
  EEPROM.commit();
}

WiFiCreds_t loadWiFiCredsFromEEPROM() {
  WiFiCreds_t creds;
  EEPROM.get(EEPROM_WIFI_CRED_ADDR, creds);
  return creds;
}


void saveNetworkMode(NetworkMode mode) {
  EEPROM.write(EEPROM_NETWORK_MODE_ADDR, (uint8_t)mode);
  EEPROM.commit();
}

NetworkMode loadNetworkMode() {
  uint8_t val = EEPROM.read(EEPROM_NETWORK_MODE_ADDR);
  if (val > 2) return GSM_ONLY;  // fallback
  return static_cast<NetworkMode>(val);
}

// Read `sendInterval` from EEPROM
uint32_t readSendIntervalFromEEPROM() {
  uint32_t interval;
  EEPROM.get(EEPROM_SEND_INTERVAL_ADDR, interval);
  if (interval == 0 || interval > 3600000) {  // Set a default if the value is invalid
    interval = 60000;                         // Default to 60 seconds
    EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, interval);
    EEPROM.commit();
  }
  return interval;
}

void writeSendIntervalToEEPROM(uint32_t interval) {
  EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, interval);
  EEPROM.commit();
}

// ---- Implementation ----
static const uint8_t TH_MAGIC = 0xA5;

bool saveThresholdsToEEPROM(const Thresholds_t& in)
{
  // Layout: [magic:1][normal:2][warning:2][alert:2][critical:2]
  int addr = EEPROM_THRESHOLDS_ADDR;

  EEPROM.write(addr++, TH_MAGIC);

  // write in compact int16 form
  int16_t v;

  v = in.normal;   EEPROM.put(addr, v); addr += sizeof(int16_t);
  v = in.warning;  EEPROM.put(addr, v); addr += sizeof(int16_t);
  v = in.alert;    EEPROM.put(addr, v); addr += sizeof(int16_t);
  v = in.critical; EEPROM.put(addr, v); addr += sizeof(int16_t);

  EEPROM.commit();
  return true;
}

bool loadThresholdsFromEEPROM(Thresholds_t& out)
{
  int addr = EEPROM_THRESHOLDS_ADDR;

  uint8_t magic = EEPROM.read(addr++);
  if (magic != TH_MAGIC) {
    // not initialized yet -> caller decides defaults
    return false;
  }

  int16_t v;

  EEPROM.get(addr, v); addr += sizeof(int16_t); out.normal   = v;
  EEPROM.get(addr, v); addr += sizeof(int16_t); out.warning  = v;
  EEPROM.get(addr, v); addr += sizeof(int16_t); out.alert    = v;
  EEPROM.get(addr, v); addr += sizeof(int16_t); out.critical = v;

  return true;
}
