#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H
#include <Arduino.h>

// Struct to hold sensor data
typedef struct {
    float temperature; // Example: Temperature reading
    float humidity;    // Example: Humidity reading
    float heatIndex;  // heat index
    bool isValid;      // true if the data is stable
} SensorData_t;

// RTOS Task Handle
extern TaskHandle_t xTaskHandle_SensorManager;
extern SemaphoreHandle_t xSemaphore_Sensor;

// Global Sensor Data
extern SensorData_t sensorData;

// Function Prototypes
void startSensorManagerTask();  // Function to start the task
void SensorManagerTask(void* pvParameters); // Task function

#endif // SENSOR_MANAGER_H