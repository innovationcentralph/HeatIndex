#include "NetworkManager.h"
#include "SensorManager.h"
#include "MQTTMonitor.h"
#include "EEPROM.h"
#include "SystemConfig.h"
#include "ShiftRegister.h"
#include "ThresholdMonitor.h"
#include "CLIHandler.h"

MQTTHandler mqttHandler;
ShiftRegister OUTPUT_CONTROL_PORT(OUT_CTRL_DIN, OUT_CTRL_CS, OUT_CTRL_CLK, 1);

bool configTimeFlag = false;
uint32_t sendInterval = 10000;
uint32_t lastMQTTConnectionTime = 0;


// Function Prototypes
uint32_t readSendIntervalFromEEPROM();
void writeSendIntervalToEEPROM(uint32_t interval);
void handleCLICommand(String command);

unsigned long lastDay = 0;  // Track the last recorded day (epoch)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  pinMode(WDT_DONE, OUTPUT);
  digitalWrite(WDT_DONE, LOW);

  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Last Reset Reason: ");
  Serial.println(reason);

  EEPROM.begin(EEPROM_SIZE);

  // ✅ Load thresholds from EEPROM
  EEPROM.get(EEPROM_THRESHOLD_ADDR, thresholdConfig);

  // Read `sendInterval` from EEPROM
  sendInterval = readSendIntervalFromEEPROM();

  if (sendInterval < 10000 || sendInterval >= 3600000) {  // Minimum Sending time is 10s
    sendInterval = 10000;
    EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, sendInterval);
    EEPROM.commit();
  }

  Serial.println("Sending Interval: " + String(sendInterval));


  // Start the Network Monitor Task
  startNetworkMonitorTask();

  // Start the Sensor Manager Task
  startSensorManagerTask();
  // Start the new Threshold Monitor Task
  startThresholdMonitorTask();

  // get saved MQTT credentials
  loadMQTTConfigFromEEPROM();
  loadDeviceESNFromEEPROM();
  loadThresholdConfigFromEEPROM();
  initializeDynamicTopics();

  // Initialize MQTT
  mqttHandler.init(
    mqttConfig.mqttServer,
    mqttConfig.mqttPort,
    mqttConfig.mqttUser,
    mqttConfig.mqttPassword,
    deviceESN,

    willTopic.c_str(),
    willMessage.c_str());


  // Set the initial message to publish on connect
  mqttHandler.setInitialMessage(initTopic.c_str(), "Device is now connected");

  // // Add subscription topics
  mqttHandler.addSubscriptionTopic(intervalConfig.c_str());
  mqttHandler.addSubscriptionTopic(mqttConfiguration.c_str());
  mqttHandler.addSubscriptionTopic("HI/Thresholds");
  // Start the MQTT Monitor Task
  startMQTTMonitorTask();

  unsigned long mqttWaitStart = millis();
  while (!networkInfo.mqttConnected && millis() - mqttWaitStart < 5000) {
    delay(100);
  }

  if (networkInfo.mqttConnected) {
    mqttHandler.publish("HI/RequestThresholds", "reqTH");
    Serial.println("Requested threshold update on startup.");
  } else {
    Serial.println("MQTT not connected; threshold request skipped.");
  }

  OUTPUT_CONTROL_PORT.setBitOrder(LSBFIRST);
  OUTPUT_CONTROL_PORT.setAll(BIT_OFF);
  OUTPUT_CONTROL_PORT.updateRegisters();
}

uint32_t lastSendTime = 0;
bool initMessageSent = false;


uint32_t ledMillis = 0;

String inputCommand = "";


void loop() {
  // put your main code here, to run repeatedly:


  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      inputCommand.trim();
      if (inputCommand.length() > 0) {
        handleCLICommand(inputCommand);
        inputCommand = "";
      }
    } else {
      inputCommand += ch;
    }
  }

  // Check Serrver Connectivity
  if (networkInfo.wifiConnected && networkInfo.mqttConnected) {
    lastMQTTConnectionTime = millis();
  } else {

    // Check if the ESP has been disconnected for too long
    if (millis() - lastMQTTConnectionTime > MQTT_DISCONNECT_TIMEOUT_MS) {
      Serial.println("MQTT disconnected for too long. Restarting ESP...");
      vTaskDelay(100);
      ESP.restart();  // Restart the ESP
    }
  }


  //Send data to server

  if (millis() - lastSendTime > sendInterval && sensorData.isValid) {
    lastSendTime = millis();
    // String topic = String("RHT/") + deviceESN;
    //String payload = "{\"TEMP\":\"" + String(sensorData.temperature, 1) + "\", \"RH\":\"" + String(sensorData.humidity, 1) + "\", \"SD\":\"1\", \"STAT\":\"Online\"}";
    //added
    String topic = "HI/SensorData";
    String payload = "{\"hi\":\"" + String(sensorData.heatIndex, 0) + "\", \"t\":\"" + String(sensorData.temperature, 0) + "\", \"h\":\"" + String(sensorData.humidity, 0) + "\"}";


    if (networkInfo.mqttConnected) {
      mqttHandler.publish(topic.c_str(), payload.c_str());
      mqttHandler.publish("HI/RequestThresholds", "reqTH");
#ifdef DEBUG_MQTT
      Serial.println("Sent heartbeat message.");
      Serial.println("Payload: " + payload);
      Serial.println("Requested threshold update.");
#endif
    }
  }


  // Trigger Watchdog
  if (networkInfo.mqttConnected) {
    digitalWrite(WDT_DONE, HIGH);
    vTaskDelay(500);
    digitalWrite(WDT_DONE, LOW);
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}



// Read `sendInterval` from EEPROM
uint32_t readSendIntervalFromEEPROM() {
  uint32_t interval;
  EEPROM.get(EEPROM_SEND_INTERVAL_ADDR, interval);
  if (interval == 0 || interval > 3600000) {  // Set a default if the value is invalid
    interval = 60000;                         // Default to 60 seconds
    EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, interval);
    EEPROM.commit();
  }
  return interval;
}

