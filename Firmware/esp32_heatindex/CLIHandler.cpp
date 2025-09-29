#include "CLIHandler.h"
#include "MQTTHandler.h"
#include "EEPROM.h"
#include "SystemConfig.h"
#include "ThresholdMonitor.h"

// Correct external variable declarations
extern MQTTConfig_t mqttConfig;
extern uint32_t sendInterval;
extern char deviceESN[DEVICE_ESN_MAX_LEN];  // ✅ Fix here
extern ThresholdConfig_t thresholdConfig;   // ✅ Fix here

// External function declarations
extern void saveMQTTConfigToEEPROM(const MQTTConfig_t& config);
extern void saveDeviceESNToEEPROM(const char* esn);
extern void writeSendIntervalToEEPROM(uint32_t interval);

void handleCLICommand(String command) {
  command.trim();

  if (command.startsWith("SETMQTT ")) {
    int firstSpace = command.indexOf(' ');
    int secondSpace = command.indexOf(' ', firstSpace + 1);
    int thirdSpace = command.indexOf(' ', secondSpace + 1);
    int fourthSpace = command.indexOf(' ', thirdSpace + 1);

    if (firstSpace > 0 && secondSpace > firstSpace && thirdSpace > secondSpace && fourthSpace > thirdSpace) {
      String server = command.substring(firstSpace + 1, secondSpace);
      String user = command.substring(secondSpace + 1, thirdSpace);
      String pass = command.substring(thirdSpace + 1, fourthSpace);
      String portStr = command.substring(fourthSpace + 1);
      int port = portStr.toInt();

      if (port > 0 && port <= 65535) {
        MQTTConfig_t newConfig;
        server.toCharArray(newConfig.mqttServer, sizeof(newConfig.mqttServer));
        user.toCharArray(newConfig.mqttUser, sizeof(newConfig.mqttUser));
        pass.toCharArray(newConfig.mqttPassword, sizeof(newConfig.mqttPassword));
        newConfig.mqttPort = port;

        mqttConfig = newConfig;
        saveMQTTConfigToEEPROM(mqttConfig);
        Serial.println("MQTT credentials and port saved.");
      } else {
        Serial.println("Invalid port number. Use a number between 1 and 65535.");
      }
    } else {
      Serial.println("Invalid format. Use: SETMQTT <server> <user> <pass> <port>");
    }
  }

  else if (command.startsWith("SETINTERVAL ")) {
    String valStr = command.substring(12);
    uint32_t val = valStr.toInt();
    if (val >= 10000 && val <= 3600000) {
      sendInterval = val;
      writeSendIntervalToEEPROM(val);
      Serial.println("Send interval updated.");
    } else {
      Serial.println("Invalid interval. Use between 10000 and 3600000 ms.");
    }
  }

  else if (command == "SHOWCONFIG") {
    Serial.println("Current Configuration:");
    Serial.println("------------------------");
    Serial.println("MQTT Server : " + String(mqttConfig.mqttServer));
    Serial.println("MQTT User   : " + String(mqttConfig.mqttUser));
    Serial.println("MQTT Pass   : " + String(mqttConfig.mqttPassword));
    Serial.println("MQTT Port   : " + String(mqttConfig.mqttPort));
    Serial.println("Send Interval: " + String(sendInterval) + " ms");
    Serial.println("Device ESN  : " + String(deviceESN));
    Serial.println("------------------------");
    Serial.println("Heat Index Thresholds:");
    Serial.println("  Normal   : " + String(thresholdConfig.heatIndexNormal));
    Serial.println("  Warning  : " + String(thresholdConfig.heatIndexWarning));
    Serial.println("  Alert    : " + String(thresholdConfig.heatIndexAlert));
    Serial.println("  Critical : " + String(thresholdConfig.heatIndexCritical));
    Serial.println("------------------------");
  }


  else if (command.startsWith("SETESN ")) {
    String newESN = command.substring(7);
    newESN.trim();

    if (newESN.length() < DEVICE_ESN_MAX_LEN) {
      saveDeviceESNToEEPROM(newESN.c_str());
      Serial.println("Device ESN updated to: " + newESN);
      Serial.println("Reboot to apply new ESN.");
    } else {
      Serial.println("ESN too long.");
    }
  }

  else if (command == "RESTART") {
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

  else if (command.startsWith("SETTHRESHOLD ")) {
    int normal, warning, alert, critical;
    int parsed = sscanf(command.c_str(), "SETTHRESHOLD %d %d %d %d", &normal, &warning, &alert, &critical);

    if (parsed == 4) {
      thresholdConfig.heatIndexNormal = normal;
      thresholdConfig.heatIndexWarning = warning;
      thresholdConfig.heatIndexAlert = alert;
      thresholdConfig.heatIndexCritical = critical;

      EEPROM.put(EEPROM_THRESHOLD_ADDR, thresholdConfig);
      EEPROM.commit();

      Serial.println("Heat Index Thresholds Saved:");
      Serial.println("  Normal   : " + String(normal));
      Serial.println("  Warning  : " + String(warning));
      Serial.println("  Alert    : " + String(alert));
      Serial.println("  Critical : " + String(critical));
    } else {
      Serial.println("Invalid format. Use:");
      Serial.println("SETTHRESHOLD <normal> <warning> <alert> <critical>");
    }
  }

  else {
    Serial.println("Unknown command.");
  }
}
