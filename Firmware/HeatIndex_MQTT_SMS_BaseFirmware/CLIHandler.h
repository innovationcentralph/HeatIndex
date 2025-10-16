#pragma once
#include <Arduino.h>
#include "SystemConfig.h"

void startCliTask();
void cliTask(void* pvParameters);
void handleCLICommand(String cmd);