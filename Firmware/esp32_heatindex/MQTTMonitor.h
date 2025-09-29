#ifndef MQTT_MONITOR_H
#define MQTT_MONITOR_H

#include <Arduino.h>
#include "MQTTHandler.h"
#include "SystemConfig.h"

// Define RTOS task handle for MQTT monitoring
extern TaskHandle_t xTaskHandle_MQTTMonitor;
extern SemaphoreHandle_t xSemaphore_LocalTime;

extern String localTime;
extern unsigned long epochTime;

extern uint32_t sendInterval;

// Function prototypes
void MQTTMonitor_Routine(void* pvParameters);
void startMQTTMonitorTask();

#endif // MQTT_MONITOR_H