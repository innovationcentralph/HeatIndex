#include "MQTTMonitor.h"

// Include external dependencies
#include "MQTTHandler.h"
#include "NetworkManager.h"
#include "WiFi.h"
#include "EEPROM.h"



// Define task handle
TaskHandle_t xTaskHandle_MQTTMonitor = NULL;
SemaphoreHandle_t xSemaphore_LocalTime = NULL;



// Create an instance of the MQTT handler
extern MQTTHandler mqttHandler;  // Assuming the instance is declared in the main sketch

String localTime = "";
unsigned long epochTime = 0;

// Function prototypes
String getTime12HourFormat(unsigned long epoch, int gmtOffset);
void writeSendIntervalToEEPROM(uint32_t interval);


// Function to start the MQTT monitor task
void startMQTTMonitorTask() {

  xSemaphore_LocalTime = xSemaphoreCreateMutex();
  if (xSemaphore_LocalTime == NULL) {
    Serial.println("Failed to create mutex for localTime");
    return;
  }

  xTaskCreatePinnedToCore(
    MQTTMonitor_Routine,       // Task function
    "MQTT Monitor",            // Task name
    4096,                      // Stack size
    NULL,                      // Task parameters
    1,                         // Task priority
    &xTaskHandle_MQTTMonitor,  // Task handle
    0                          // Core ID
  );
}

// MQTT monitoring routine
void MQTTMonitor_Routine(void* pvParameters) {
  Serial.print("MQTT monitoring running on core ");
  Serial.println(xPortGetCoreID());


  for (;;) {
    if (networkInfo.wifiConnected == true) {

      // Check MQTT connectivity
      if (mqttHandler.checkConnectivity()) {
        //Serial.println("MQTT Connected");
        networkInfo.mqttConnected = true;
      } else {
        //Serial.println("MQTT Disconnected");
        networkInfo.mqttConnected = false;
      }

      // Check for incoming messages
      if (mqttHandler.messageAvailable()) {
        String incomingTopic = mqttHandler.getMessageTopic();
        String incomingPayload = mqttHandler.getMessagePayload();

        // Process the message

        Serial.print("Received message on topic: ");
        Serial.println(incomingTopic);
        Serial.print("Message payload: ");
        Serial.println(incomingPayload);

 // Threshold downlink
if (incomingTopic.equals("HI/Thresholds")) {
  String payload = incomingPayload;
  payload.replace(" ", "");  // Sanitize input by removing spaces

  // Lambda to extract values
  auto getValue = [](const String& json, const String& key) -> int {
    int keyIndex = json.indexOf("\"" + key + "\"");
    if (keyIndex == -1) return -1;

    int colonIndex = json.indexOf(":", keyIndex);
    if (colonIndex == -1) return -1;

    int quoteStart = json.indexOf("\"", colonIndex);
    if (quoteStart == -1) return -1;

    int quoteEnd = json.indexOf("\"", quoteStart + 1);
    if (quoteEnd == -1) return -1;

    String valueStr = json.substring(quoteStart + 1, quoteEnd);
    return valueStr.toInt();
  };

  int normal   = getValue(payload, "normal");
  int warning  = getValue(payload, "warning");
  int alert    = getValue(payload, "alert");
  int critical = getValue(payload, "critical");

  if (normal >= 0 && warning >= 0 && alert >= 0 && critical >= 0) {
    thresholdConfig.heatIndexNormal   = normal;
    thresholdConfig.heatIndexWarning  = warning;
    thresholdConfig.heatIndexAlert    = alert;
    thresholdConfig.heatIndexCritical = critical;

    saveThresholdConfigToEEPROM(thresholdConfig);

    Serial.println("✅ Thresholds updated from MQTT:");
    Serial.println("  Normal   : " + String(normal));
    Serial.println("  Warning  : " + String(warning));
    Serial.println("  Alert    : " + String(alert));
    Serial.println("  Critical : " + String(critical));
  } else {
    Serial.println("❌ Invalid or missing threshold values in payload.");
  }
}









      // Process change in sending interval
      if (incomingTopic.equals(intervalConfig.c_str())) {
        // Parse the payload for a new interval
        uint32_t newInterval = incomingPayload.toInt();
        if (newInterval > 10000 && newInterval <= 3600000) {  // Ensure valid interval > 10s
          sendInterval = newInterval;
          writeSendIntervalToEEPROM(newInterval);  // Save to EEPROM
          Serial.println("Updated send interval: " + String(sendInterval) + " ms");
        }
      }



      // Process MQTT credential Change
      if (incomingTopic.equals(mqttConfiguration.c_str())) {
        int firstComma = incomingPayload.indexOf(',');
        int secondComma = incomingPayload.indexOf(',', firstComma + 1);
        int thirdComma = incomingPayload.indexOf(',', secondComma + 1);

        if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma) {
          String newServer = incomingPayload.substring(0, firstComma);
          String newUser = incomingPayload.substring(firstComma + 1, secondComma);
          String newPass = incomingPayload.substring(secondComma + 1, thirdComma);
          String portStr = incomingPayload.substring(thirdComma + 1);
          int newPort = portStr.toInt();

          MQTTConfig_t originalConfig = mqttConfig;
          MQTTConfig_t trialConfig;

          newServer.toCharArray(trialConfig.mqttServer, sizeof(trialConfig.mqttServer));
          newUser.toCharArray(trialConfig.mqttUser, sizeof(trialConfig.mqttUser));
          newPass.toCharArray(trialConfig.mqttPassword, sizeof(trialConfig.mqttPassword));
          trialConfig.mqttPort = (newPort > 0 && newPort <= 65535) ? newPort : originalConfig.mqttPort;

          Serial.println("Trying new MQTT credentials:");
          Serial.println("Server: " + newServer);
          Serial.println("User: " + newUser);
          Serial.println("Password: " + newPass);
          Serial.println("Port: " + String(trialConfig.mqttPort));

          mqttHandler.init(
            trialConfig.mqttServer,
            trialConfig.mqttPort,
            trialConfig.mqttUser,
            trialConfig.mqttPassword,
            deviceESN,
            willTopic.c_str(),
            willMessage.c_str());

          if (mqttHandler.connect()) {
            Serial.println("Connection successful. Saving new credentials.");
            mqttConfig = trialConfig;
            saveMQTTConfigToEEPROM(mqttConfig);
          } else {
            Serial.println("Connection failed. Reverting to original credentials.");

            mqttHandler.init(
              originalConfig.mqttServer,
              originalConfig.mqttPort,
              originalConfig.mqttUser,
              originalConfig.mqttPassword,
              deviceESN,
              willTopic.c_str(),
              willMessage.c_str());

            mqttHandler.connect();
          }

        } else {
          Serial.println("Invalid format. Use: server,user,password,port");
        }
      }


      // Clear the message flag
      mqttHandler.clearMessageFlag();
    }
  }

  // Yield to other tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
}

// Write `sendInterval` to EEPROM
void writeSendIntervalToEEPROM(uint32_t interval) {
  EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, interval);
  EEPROM.commit();
}