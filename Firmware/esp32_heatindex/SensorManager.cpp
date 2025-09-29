#include "SensorManager.h"
#include "SystemConfig.h"
#include "SHT2x.h"
//#define DEBUG_SENSOR  // test
SHT2x sht20(&Wire);
#define MAX_SENSOR_HISTORY 3
#define MAX_TEMP_DIFF_ALLOWED 15.0  // If temp jumps more than this, consider it a spike 5
#define MAX_HUM_DIFF_ALLOWED 30.0  // For humidity 10

#define HEAT_INDEX_DANGER_THRESHOLD 30.0f  // °C

// Define the task handle
TaskHandle_t xTaskHandle_SensorManager = NULL;
SemaphoreHandle_t xSemaphore_Sensor = NULL;

// Define the global sensor data
SensorData_t sensorData = { 0.0, 0.0 };

// Circular buffer for filtering
float tempHistory[MAX_SENSOR_HISTORY] = { 0 };
float humHistory[MAX_SENSOR_HISTORY] = { 0 };
uint8_t historyIndex = 0;
bool historyFilled = false;

float computeAverage(float* buffer, uint8_t size) {
  float sum = 0;
  for (uint8_t i = 0; i < size; i++) {
    sum += buffer[i];
  }
  return sum / size;
}

// Compute heat index based on temperature (°C) and humidity (%)
float computeHeatIndex(float temperatureC, float humidity) {
  // Convert temperature to Fahrenheit for calculation
  float T = temperatureC * 1.8 + 32;
  float R = humidity;

  float HI = -42.379 + 2.04901523 * T + 10.14333127 * R + -0.22475541 * T * R + -0.00683783 * T * T + -0.05481717 * R * R + 0.00122874 * T * T * R + 0.00085282 * T * R * R + -0.00000199 * T * T * R * R;

  // Convert back to Celsius
  return (HI - 32) * 5.0 / 9.0;
}


// Timer interval for reading sensors (in milliseconds)
const uint32_t SENSOR_READ_INTERVAL_MS = 5000;

void SensorManagerTask(void* pvParameters);

void startSensorManagerTask() {

  xSemaphore_Sensor = xSemaphoreCreateMutex();
  if (xSemaphore_Sensor == NULL) {
    Serial.println("Failed to create mutex for sensor");
    return;
  }

  xTaskCreatePinnedToCore(
    SensorManagerTask,           // Task function
    "Sensor Manager",            // Task name
    4096,                        // Stack size
    NULL,                        // Task parameters
    1,                           // Task priority
    &xTaskHandle_SensorManager,  // Task handle
    1                            // Core ID
  );
}

void SensorManagerTask(void* pvParameters) {
  Serial.print("Sensor Manager running on core ");
  Serial.println(xPortGetCoreID());

  // Initialize sensor peripherals here
  Serial.println("Initializing sensors...");

  uint32_t lastReadTime = millis();

  // SHT2x Init
  Wire.begin(SDA_PIN, SCL_PIN);
  sht20.begin();
  uint8_t stat = sht20.getStatus();
  Serial.print("Sensor status: 0x");
  Serial.println(stat, HEX);

  // Give sensor time to stabilize
  vTaskDelay(pdMS_TO_TICKS(2000));

  float initTemp = 0.0f, initHum = 0.0f;
  bool validInit = false;

  // Try up to 5 times to get a valid initial reading
  for (int attempt = 0; attempt < 5; attempt++) {
    sht20.read();
    initTemp = sht20.getTemperature();
    initHum = sht20.getHumidity();

    bool tempValid = initTemp > -40.0f && initTemp < 85.0f;
    bool humValid = initHum >= 0.0f && initHum <= 100.0f;

    if (tempValid && humValid) {
      validInit = true;
      break;
    }

    Serial.println("⚠️ Invalid initial sensor reading. Retrying...");
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  if (validInit) {
    for (int i = 0; i < MAX_SENSOR_HISTORY; i++) {
      tempHistory[i] = initTemp;
      humHistory[i] = initHum;
    }
    historyFilled = true;

    sensorData.temperature = initTemp;
    sensorData.humidity = initHum;
    sensorData.heatIndex = computeHeatIndex(initTemp, initHum);
    sensorData.isValid = true;

    Serial.println("✅ Sensor warm-up complete.");
  } else {
    Serial.println("❌ Failed to get valid initial sensor data. Marking as invalid.");
    sensorData.isValid = false;
  }




  // Main task loop
  for (;;) {
    if (millis() - lastReadTime >= SENSOR_READ_INTERVAL_MS) {
      // Time to read sensors
      Serial.println("Reading sensors...");

      if (xSemaphoreTake(xSemaphore_Sensor, portMAX_DELAY) == pdTRUE) {
        sht20.read();
        float newTemp = sht20.getTemperature();
        float newHum = sht20.getHumidity();
        bool tempOK = true;
        bool humOK = true;

        if (historyFilled) {
          float avgTemp = computeAverage(tempHistory, MAX_SENSOR_HISTORY);
          float avgHum = computeAverage(humHistory, MAX_SENSOR_HISTORY);

          tempOK = fabs(newTemp - avgTemp) <= MAX_TEMP_DIFF_ALLOWED;
          humOK = fabs(newHum - avgHum) <= MAX_HUM_DIFF_ALLOWED;
        }

        if (tempOK && humOK) {
          tempHistory[historyIndex] = newTemp;
          humHistory[historyIndex] = newHum;
          historyIndex = (historyIndex + 1) % MAX_SENSOR_HISTORY;
          historyFilled = true;

          sensorData.temperature = computeAverage(tempHistory, MAX_SENSOR_HISTORY);
          sensorData.humidity = computeAverage(humHistory, MAX_SENSOR_HISTORY);
          sensorData.heatIndex = computeHeatIndex(sensorData.temperature, sensorData.humidity);  // for heat index
          sensorData.isValid = true;

#ifdef DEBUG_SENSOR
          Serial.print("Filtered Temperature: ");
          Serial.println(sensorData.temperature);
          Serial.print("Filtered Humidity: ");
          Serial.println(sensorData.humidity);
          Serial.print("Heat Index (°C): ");     // for heat index
          Serial.println(sensorData.heatIndex);  // for heat index
          if (sensorData.heatIndex >= HEAT_INDEX_DANGER_THRESHOLD) {
            Serial.print("HOT");  // for heat index
          }


#endif
        } else {
          Serial.println("⚠️ Spike detected, ignoring this sample");
          sensorData.isValid = false;
        }

        xSemaphoreGive(xSemaphore_Sensor);
      }

      // Reset timer
      lastReadTime = millis();
    }

    // Task delay to yield execution
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
