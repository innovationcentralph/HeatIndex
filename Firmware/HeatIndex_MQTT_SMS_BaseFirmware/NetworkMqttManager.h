#ifndef NETWORK_MQTT_MANAGER_H
#define NETWORK_MQTT_MANAGER_H

#include <Arduino.h>
#include <PubSubClient.h>

// ================== Modes ==================
typedef enum {
  WIFI_ONLY,
  GSM_ONLY,
  WIFI_GSM
} NetworkMode;

extern NetworkMode currentNetworkMode;

// Start network manager (creates RTOS tasks)
void startNetworkMqttManagerTask(int powerPin, int ledPin = -1, NetworkMode mode = WIFI_GSM);

// ================== External States & Clients ==================
extern bool isMQTTConnectedWiFi;
extern bool isMQTTConnectedGSM;
extern PubSubClient mqttWiFi;
extern PubSubClient mqttGSM;

#define WIFI_CONNECTED  1
#define GSM_CONNECTED   2

// ================== SMS API ==================
// Non-blocking; queues an SMS to be sent by the SMS task.
bool enqueueSms(const char* number, const char* text);

// ================== Hooks (set from your .ino) ==================
// Called after every successful (re)connect to let your sketch subscribe.
typedef void (*MqttOnConnectHandler)(PubSubClient& client, bool isWifi);
void setMqttOnConnectHandler(MqttOnConnectHandler handler);

// MQTT message callback (compatible with PubSubClient)
typedef void (*MqttMessageCallback)(char* topic, byte* payload, unsigned int length);
void setMqttMessageCallback(MqttMessageCallback cb);

#endif // NETWORK_MQTT_MANAGER_H
