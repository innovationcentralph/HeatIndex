#pragma once

#include "SensorManager.h"     // For SensorData_t
#include "SystemConfig.h"      // For ThresholdConfig_t
#include "ShiftRegister.h"     // For controlling relays

void startThresholdMonitorTask();
void checkHeatIndexAndTriggerRelays(const SensorData_t& sensorData, ShiftRegister& outputPort);
