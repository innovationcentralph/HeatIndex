#include "ThresholdMonitor.h"

// Externals
extern SensorData_t sensorData;
extern ThresholdConfig_t thresholdConfig;
extern ShiftRegister OUTPUT_CONTROL_PORT;

// Optional: Simulate values without real sensors
// Uncomment to use simulation for testing
// #define DEBUG_THRESHOLD

#ifdef DEBUG_THRESHOLD
float simulatedHI = 25.0;
#endif

// Core logic that checks HI and updates relay outputs
void checkHeatIndexAndTriggerRelays(const SensorData_t& sensorData, ShiftRegister& outputPort) {
  float hi;

#ifdef DEBUG_THRESHOLD
  // Simulate HI value cycling from 25 to 55
  static float simulatedHI = 20.0;
  hi = simulatedHI;
  simulatedHI += 1.0;
  if (simulatedHI > 55.0) simulatedHI = 25.0;
  Serial.println("[DEBUG] Simulated HI: " + String(hi));
#else
  if (!sensorData.isValid) return;
  hi = sensorData.heatIndex;
#endif

  // Turn OFF all relays first
  outputPort.setAll(BIT_OFF);
  outputPort.updateRegisters();

  // Load dynamic thresholds
  float normal   = thresholdConfig.heatIndexNormal;
  float warning  = thresholdConfig.heatIndexWarning;
  float alert    = thresholdConfig.heatIndexAlert;
  float critical = thresholdConfig.heatIndexCritical;

  // Relay logic based on heat index range
  if (hi >= critical) {
    outputPort.setBit(1, BIT_ON);
  //  Serial.println("🔴 HI ≥ Critical → Relay 4 (Red)");
  } else if (hi >= alert && hi < critical) {
    outputPort.setBit(2, BIT_ON);
   // Serial.println("🟠 HI in [Alert, Critical) → Relay 3 (Orange)");
  } else if (hi >= warning && hi < alert) {
    outputPort.setBit(3, BIT_ON);
  //  Serial.println("🔵 HI in [Warning, Alert) → Relay 2 (Blue)");
  } else if (hi >= normal && hi < warning) {
    outputPort.setBit(4, BIT_ON);
 //   Serial.println("🟢 HI in [Normal, Warning) → Relay 1 (Green)");
  } else {
  //  Serial.println("⚪ HI < Normal → All relays OFF");
  }

  // Apply the final output state
  outputPort.updateRegisters();
}


// RTOS task that continuously checks heat index
void ThresholdTask(void* parameter) {
  for (;;) {
    checkHeatIndexAndTriggerRelays(sensorData, OUTPUT_CONTROL_PORT);
    vTaskDelay(pdMS_TO_TICKS(1000));  // Check every 1 second
  }
}

// Called from setup() to launch the RTOS task
void startThresholdMonitorTask() {
  xTaskCreatePinnedToCore(
    ThresholdTask,            // Task function
    "ThresholdMonitor",       // Task name
    2048,                     // Stack size
    NULL,                     // Task parameters
    1,                        // Priority
    NULL,                     // Task handle (optional)
    1                         // Core 1 (ESP32 has 2 cores: 0 & 1)
  );
}
