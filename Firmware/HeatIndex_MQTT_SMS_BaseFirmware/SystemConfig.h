#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <Arduino.h>
#include "NetworkMqttManager.h"
//==============================================================================
//                              DEBUG FLAGS
//==============================================================================
// Uncomment to enable serial debug output for each subsystem
 #define DEBUG_WIFI
 #define DEBUG_MQTT
 #define DEBUG_SENSOR
 #define DEBUG_TH
 #define DEBUG_CONTACTS
 #define DEBUG_SMS


//==============================================================================
//                               HARDWARE PINS
//==============================================================================


// Master enable / data lines (e.g., for GSM or other radio)
#define MASTER_RO 26  // Master Read Only line
#define MASTER_DI 25  // Master Data In line
#define MASTER_EN 27  // Master Enable line

#define GSM_PWR 4

// LEDs
#define NETWORK_INDICATOR 12     // Server Connectivity LED
#define ACTIVE_INDICATOR 13  // IDLE Indicator LED
#define LED_ON HIGH
#define LED_OFF LOW

//==============================================================================
//                             STRUCTURES
//==============================================================================
typedef struct {
  char mqttServer[64];
  char mqttUser[32];
  char mqttPassword[32];
  int mqttPort;
} MQTTConfig_t;

typedef struct {
  char ssid[32];
  char password[64];
} WiFiCreds_t;

typedef struct{
  int normal;
  int warning;
  int alert;
  int critical;
} Thresholds_t;

typedef struct{
  bool green;
  bool blue;
  bool orange;
  bool red;
} TowerLight_t;

enum TowerLightState {
  STATE_GREEN,
  STATE_BLUE,
  STATE_ORANGE,
  STATE_RED,
  STATE_INIT
};

#define DEFAUL_NETWORK_MODE GSM_ONLY
//==============================================================================
//                             EEPROM SETTINGS
//==============================================================================
// Total EEPROM size (bytes)
#define EEPROM_SIZE 1024

#define DEVICE_ESN_MAX_LEN 32
#define SEND_INTERVAL_SIZE  sizeof(uint32_t)
#define MQTT_CONFIG_SIZE    sizeof(MQTTConfig_t)
#define WIFI_CREDS_SIZE     sizeof(WiFiCreds_t)
#define NETWORK_MODE_SIZE   sizeof(uint8_t)

// Contacts storage
#define MAX_CONTACTS        35
#define CONTACT_MAX_LEN     16         // "+639..." up to 15 chars + NUL
#define CONTACTS_BLOCK_SIZE (1 + (MAX_CONTACTS * CONTACT_MAX_LEN)) // count + data

// ---- Thresholds block (4 x int16 + 1 magic) ----
#define THRESHOLDS_COUNT        4
#define THRESHOLD_FIELD_SIZE    sizeof(int16_t)   // store compactly
#define THRESHOLDS_BLOCK_SIZE   (1 + THRESHOLDS_COUNT * THRESHOLD_FIELD_SIZE) // magic + 4x int16


// Addresses within EEPROM
#define EEPROM_ESN_ADDR               0  // Start address for storing ESN
#define EEPROM_SEND_INTERVAL_ADDR    (EEPROM_ESN_ADDR + DEVICE_ESN_MAX_LEN)
#define EEPROM_MQTT_CONFIG_ADDR      (EEPROM_SEND_INTERVAL_ADDR + SEND_INTERVAL_SIZE) + 10
#define EEPROM_WIFI_CRED_ADDR        (EEPROM_MQTT_CONFIG_ADDR + MQTT_CONFIG_SIZE)
#define EEPROM_NETWORK_MODE_ADDR     (EEPROM_WIFI_CRED_ADDR + WIFI_CREDS_SIZE) 
#define EEPROM_CONTACTS_ADDR         (EEPROM_NETWORK_MODE_ADDR + NETWORK_MODE_SIZE)
#define EEPROM_THRESHOLDS_ADDR        (EEPROM_CONTACTS_ADDR + CONTACTS_BLOCK_SIZE)


#define EEPROM_RESERVED_ADDR           (EEPROM_THRESHOLDS_ADDR + THRESHOLDS_BLOCK_SIZE)



//==============================================================================
//                           DEVICE IDENTIFIERS
//==============================================================================
// Device Electronic Serial Number


extern char deviceESN[DEVICE_ESN_MAX_LEN];
extern MQTTConfig_t mqttConfig;

//==============================================================================
//                           MQTT CONFIGURATION
//==============================================================================
// Configurable MQTT parameters
extern MQTTConfig_t mqttConfig;
extern char deviceESN[DEVICE_ESN_MAX_LEN];

extern Thresholds_t thresholds;
extern TowerLight_t towerLights;

#define TH_HYSTERESIS 0.5

extern String willTopic;
extern String willMessage;
extern String initTopic;
extern String intervalConfig;
extern String mqttConfiguration;

extern String topicPing;
extern String topicPingResp;

extern String topicSensorData;

extern String topicRequestThreshold;
extern String topicThresholds;

extern String topicRequestContacts;
extern String topicContacts;

extern uint32_t hbInterval;

// Wi-Fi credentials
extern const char* wifiSSID;
extern const char* wifiPass;

extern const char* topicInit;


// Timeout for detecting broker disconnect (milliseconds)
#define MQTT_DISCONNECT_TIMEOUT_MS 300000

//==============================================================================
//                         FUNCTION PROTOTYPES
//==============================================================================
// Load and save ESN from EEPROM into deviceESN[]
void loadDeviceESNFromEEPROM(void);
void saveDeviceESNToEEPROM(const char* esn);

// Load and save MQTT Parameters from EEPROM
void loadMQTTConfigFromEEPROM();
void saveMQTTConfigToEEPROM(const MQTTConfig_t& config);

// Load and save heartbeat interval from EEPROM
uint32_t loadHeartbeatInterval();
void saveHeartbeatInterval(uint32_t interval);

// Load dynamic MQTT topics derived from Device ESN
void initializeDynamicTopics();

// Load and Save WiFi Credentials
void saveWiFiCredsToEEPROM(const WiFiCreds_t& creds);
WiFiCreds_t loadWiFiCredsFromEEPROM();

void saveNetworkMode(NetworkMode mode);
NetworkMode loadNetworkMode();

uint32_t readSendIntervalFromEEPROM();
void writeSendIntervalToEEPROM(uint32_t interval);

bool saveThresholdsToEEPROM(const Thresholds_t& in);
bool loadThresholdsFromEEPROM(Thresholds_t& out);


#endif  // SYSTEM_CONFIG_H
