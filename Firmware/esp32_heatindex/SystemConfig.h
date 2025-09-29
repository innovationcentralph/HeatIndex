#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <Arduino.h>

// Pin definitions
#define WDT_DONE 13
#define SDA_PIN 21
#define SCL_PIN 22
#define OUT_CTRL_DIN     23
#define OUT_CTRL_CLK     25
#define OUT_CTRL_CS      26
#define BIT_OFF 1
#define BIT_ON  0

//#define STATUS_LED 13
//#define LED_ON  HIGH
//#define LED_OFF LOW


// Debug flags
#define DEBUG_WIFI
#define DEBUG_MQTT
//#define DEBUG_SENSOR

#define EEPROM_SIZE 512
#define EEPROM_SEND_INTERVAL_ADDR   0
#define EEPROM_MQTT_CONFIG_ADDR     100 
#define EEPROM_ESN_ADDR             300
#define DEVICE_ESN_MAX_LEN          32
#define EEPROM_THRESHOLD_ADDR       400 //addded

// Watchdog timer timeout (in milliseconds)
#define WDT_TIME_MS 30000
#define MQTT_DISCONNECT_TIMEOUT_MS 250000



typedef struct {
    char mqttServer[64];
    char mqttUser[32];
    char mqttPassword[32];
    int  mqttPort;
} MQTTConfig_t;

// Threshold config structure
typedef struct {
    int heatIndexCritical;
    int heatIndexAlert;
    int heatIndexWarning;
    int heatIndexNormal;
} ThresholdConfig_t;


// Configurable MQTT parameters
extern MQTTConfig_t mqttConfig;
extern ThresholdConfig_t thresholdConfig;
extern char deviceESN[DEVICE_ESN_MAX_LEN];
extern String willTopic;
extern String willMessage;
extern String initTopic;
extern String intervalConfig;
extern String mqttConfiguration;
extern String thresholdConfiguration;


// Functions
void loadMQTTConfigFromEEPROM();
void saveMQTTConfigToEEPROM(const MQTTConfig_t& config);
void loadDeviceESNFromEEPROM();
void saveDeviceESNToEEPROM(const char* esn);
void loadThresholdConfigFromEEPROM();
void saveThresholdConfigToEEPROM(const ThresholdConfig_t& config);
void initializeDynamicTopics();

#endif // SYSTEM_CONFIG_H
