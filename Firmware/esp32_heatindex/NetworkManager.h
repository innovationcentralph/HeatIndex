#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <WiFiManager.h>
#include <Arduino.h>

// Struct to hold network information
typedef struct {
    bool wifiConnected;       // Connection status
    String SSID;              // Current SSID
    String password;          // Password (optional, based on use case)
    int RSSI;                 // Signal strength (RSSI)
    bool mqttConnected;
} NetworkInfo_t;

// RTOS Task Handle
extern TaskHandle_t xTaskHandle_NetworkMonitor;

// Declare the global NetworkInfo object as extern
extern NetworkInfo_t networkInfo;

// Function Prototypes
void NetworkMonitorTask(void* pvParameters);
void startNetworkMonitorTask();

#endif // NETWORK_MANAGER_H