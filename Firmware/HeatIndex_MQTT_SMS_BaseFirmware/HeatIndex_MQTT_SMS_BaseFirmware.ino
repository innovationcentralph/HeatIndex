#include <ModbusRTU.h>

#include "SystemConfig.h"
#include "EEPROM.h"
#include "CLIHandler.h"
#include "NetworkMqttManager.h"

#include "SensorManager.h"

#include <ArduinoJson.h>

uint32_t lastMQTTConnectionTime = 0;

uint32_t heartbeatMillis = 0;

uint32_t sendInterval = 0;
uint32_t lastSendTime = 0;

String currentDateTime = "";

TowerLightState currentState = STATE_GREEN;
TowerLightState prevState = STATE_INIT;


String contacts[35];
int contactCount = 0;


// Modbus Object
ModbusRTU modbus;

// Function Prototypes
void initializeModbus();
void modbusTask(void* pvParameters);

void rtcTask(void* pvParameters);

bool saveContactsToEEPROM(const String inContacts[], uint8_t inCount);
bool loadContactsFromEEPROM(String outContacts[], uint8_t maxOut, uint8_t& outCount);

void myOnMqttConnect(PubSubClient& client, bool isWifi) {

  String _topic = "";

  client.subscribe(topicPing.c_str());
  client.subscribe(topicContacts.c_str());
  client.subscribe(topicThresholds.c_str());
  client.subscribe(topicGetDateTime.c_str());

  client.publish(initTopic.c_str(), isWifi ? "subscribed via WIFI" : "subscribed via WIFI");

  // get  latest contacts
  // Get contacts

  client.publish(topicRequestContacts.c_str(), "reqContact");
  //vTaskDelay(100);
  client.publish(topicRequestThreshold.c_str(), "reqTH");
}

void myMessageHandler(char* topic, byte* payload, unsigned int len) {
  Serial.print("RX ");
  Serial.print(topic);
  Serial.print(": ");
  for (unsigned i = 0; i < len; i++) Serial.print((char)payload[i]);
  Serial.println();

  // Null-terminate payload
  payload[len] = '\0';

  // Check if the topic matches thresholds topic
  if (String(topic) == topicThresholds) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload, len);
    if (error) {
      Serial.print(F("JSON parse failed: "));
      Serial.println(error.f_str());
      return;
    }

    thresholds.normal = doc["normal"].as<int>();
    thresholds.warning = doc["warning"].as<int>();
    thresholds.alert = doc["alert"].as<int>();
    thresholds.critical = doc["critical"].as<int>();

#ifdef DEBUG_TH
    Serial.println(F("[Thresholds Updated]"));
    Serial.print("Normal: ");
    Serial.println(thresholds.normal);
    Serial.print("Warning: ");
    Serial.println(thresholds.warning);
    Serial.print("Alert: ");
    Serial.println(thresholds.alert);
    Serial.print("Critical: ");
    Serial.println(thresholds.critical);
#endif

    Thresholds_t thSave = {
      (int16_t)thresholds.normal,
      (int16_t)thresholds.warning,
      (int16_t)thresholds.alert,
      (int16_t)thresholds.critical
    };
    saveThresholdsToEEPROM(thSave);

  }

  // Check if the topic matches contacts topic
  else if (String(topic) == topicContacts) {
    StaticJsonDocument<1024> doc;  // Larger document for contact array
    DeserializationError error = deserializeJson(doc, payload, len);
    if (error) {
      Serial.print(F("Contacts JSON parse failed: "));
      Serial.println(error.f_str());
      return;
    }

    // Reset contact count
    contactCount = 0;

    // Check if it's a single string or array
    if (doc.is<const char*>()) {
      // Single contact number
      contacts[0] = doc.as<const char*>();
      contactCount = 1;
    } else if (doc.is<JsonArray>()) {
      // Array of contact numbers
      JsonArray arr = doc.as<JsonArray>();
      int maxContacts = arr.size();
      if (maxContacts > 20) {
        maxContacts = 20;
      }
      contactCount = maxContacts;

      for (int i = 0; i < contactCount; i++) {
        contacts[i] = arr[i].as<const char*>();
      }
    }

    saveContactsToEEPROM(contacts, (uint8_t)contactCount);

#ifdef DEBUG_CONTACTS
    Serial.println(F("[Contacts Updated]"));
    Serial.print("Number of contacts: ");
    Serial.println(contactCount);
    for (int i = 0; i < contactCount; i++) {
      Serial.print("Contact ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(contacts[i]);
    }
#endif
  }

  else if (String(topic) == topicGetDateTime) {
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, payload, len);
    if (error) {
      Serial.print(F("DateTime JSON parse failed: "));
      Serial.println(error.f_str());
      return;
    }

    const char* dt = doc["datetime"];
    if (dt) {
      currentDateTime = String(dt);  // ✅ Save globally
      Serial.print(F("[DateTime Updated] "));
      Serial.println(currentDateTime);
    }
  }
}

bool onWriteDone(Modbus::ResultCode rc, uint16_t tid, void* data) {
  // Serial.print(F("[Modbus] writeCoil result="));
  // Serial.print((int)rc);
  // Serial.print(F(", tid="));
  // Serial.println(tid);
  // // Return true to keep the transaction object (or false to auto-free if lib uses it that way)
  return true;
}

void sendTowerLightsMulti(uint8_t unitId, uint16_t baseCoil) {
  // Order must match your slave map: base+0..3
  bool coils[4];
  coils[0] = towerLights.red;     // base + 0
  coils[1] = towerLights.orange;  // base + 1
  coils[2] = towerLights.blue;    // base + 2
  coils[3] = towerLights.green;   // base + 3

  // Overload: writeCoil(TYPEID id, uint16_t offset, bool* value, uint16_t numregs, cbTransaction cb = nullptr, uint8_t unit=MODBUSIP_UNIT)
  modbus.writeCoil(unitId, baseCoil, coils, (uint16_t)4, onWriteDone);
  // if (!tid) {
  //   Serial.println(F("[Modbus] writeCoil enqueue failed"));
  // }
}


void setup() {
  Serial.begin(9600);  // Initialize Serial

  pinMode(NETWORK_INDICATOR, OUTPUT);
  digitalWrite(NETWORK_INDICATOR, LED_ON);
  delay(200);
  digitalWrite(NETWORK_INDICATOR, LED_OFF);
  delay(200);
  digitalWrite(NETWORK_INDICATOR, LED_ON);
  delay(200);
  digitalWrite(NETWORK_INDICATOR, LED_OFF);


  Serial.println("Heat Index Initialized.");

  EEPROM.begin(EEPROM_SIZE);

  // load thresholds
  Thresholds_t thTmp;
  if (loadThresholdsFromEEPROM(thTmp)) {
    thresholds.normal = thTmp.normal;
    thresholds.warning = thTmp.warning;
    thresholds.alert = thTmp.alert;
    thresholds.critical = thTmp.critical;
#ifdef DEBUG_TH
    Serial.println(F("[EEPROM] Thresholds loaded."));
    Serial.printf("N:%d W:%d A:%d C:%d\n",
                  thresholds.normal, thresholds.warning,
                  thresholds.alert, thresholds.critical);
#endif
  } else {
    // defaults (tune to your app)
    thresholds.normal = 27;
    thresholds.warning = 33;
    thresholds.alert = 42;
    thresholds.critical = 52;

    // Save defaults so the block becomes initialized
    Thresholds_t thInit = {
      (int16_t)thresholds.normal,
      (int16_t)thresholds.warning,
      (int16_t)thresholds.alert,
      (int16_t)thresholds.critical
    };
    saveThresholdsToEEPROM(thInit);
#ifdef DEBUG_TH
    Serial.println(F("[EEPROM] Thresholds not found; defaults applied and saved."));
#endif
  }

  uint8_t loadedCount = 0;
  loadContactsFromEEPROM(contacts, 20, loadedCount);
  contactCount = loadedCount;

#ifdef DEBUG_CONTACTS
  Serial.print(F("[EEPROM] Loaded contacts: "));
  Serial.println(contactCount);
  for (int i = 0; i < contactCount; i++) {
    Serial.print("#");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(contacts[i]);
  }
#endif

  startCliTask();


  hbInterval = loadHeartbeatInterval();

  sendInterval = readSendIntervalFromEEPROM();

  Serial.println("Sending Interval: " + String(hbInterval));

  initializeModbus();

  // Create a task to handle Modbus communication
  xTaskCreatePinnedToCore(
    modbusTask,    // Function to run
    "ModbusTask",  // Task name
    4096,          // Stack size (in bytes)
    NULL,          // Task parameter
    1,             // Priority
    NULL,          // Task handle
    1              // Core ID (1 for ESP32 dual-core)
  );

  xTaskCreatePinnedToCore(
    rtcTask,     // Function to run
    "RTC Task",  // Task name
    4096,        // Stack size (in bytes)
    NULL,        // Task parameter
    1,           // Priority
    NULL,        // Task handle
    1            // Core ID (1 for ESP32 dual-core)
  );

  bool coil_on[4] = { 1, 1, 1, 1 };
  bool coil_off[4] = { 0, 0, 0, 0 };

  modbus.writeCoil(1, 0, coil_on, (uint16_t)4, onWriteDone);
  vTaskDelay(500);
  modbus.writeCoil(1, 0, coil_off, (uint16_t)4, onWriteDone);
  vTaskDelay(500);
  modbus.writeCoil(1, 0, coil_on, (uint16_t)4, onWriteDone);
  vTaskDelay(500);
  modbus.writeCoil(1, 0, coil_off, (uint16_t)4, onWriteDone);

  currentNetworkMode = loadNetworkMode();

  vTaskDelay(1000);

  Serial.println(" Initializing Network...");

  setMqttOnConnectHandler(myOnMqttConnect);  // <-- your subscriptions live here
  setMqttMessageCallback(myMessageHandler);  // <-- optional override of message callback


  startNetworkMqttManagerTask(GSM_PWR, NETWORK_INDICATOR, currentNetworkMode);
  vTaskDelay(5000);

  // Start the Sensor Manager Task
  startSensorManagerTask();


  // Get thresholds
  if (isMQTTConnectedGSM) {
    mqttGSM.publish(topicRequestThreshold.c_str(), "reqTH");
  } else if (isMQTTConnectedWiFi) {
    mqttWiFi.publish(topicRequestThreshold.c_str(), "reqTH");
  }

  vTaskDelay(1000);

  // Get contacts
  if (isMQTTConnectedGSM) {
    mqttGSM.publish(topicRequestContacts.c_str(), "reqContact");
  } else if (isMQTTConnectedWiFi) {
    mqttWiFi.publish(topicRequestContacts.c_str(), "reqContact");
  }


  // Initialize Modbus
  Serial2.begin(9600, SERIAL_8N1, MASTER_RO, MASTER_DI);
  modbus.begin(&Serial2, MASTER_EN);
  modbus.setBaudrate(9600);
  modbus.master();

  Serial.println("Modbus Master Initialized");

  vTaskDelay(1000);
}

void loop() {

  // N  W  A  C   -> 43
  //27 33 42 52

  // Decide new state
  if (sensorData.heatIndex >= (thresholds.critical + TH_HYSTERESIS)) {  //hi > 52.5
    currentState = STATE_RED;
  } else if (sensorData.heatIndex >= (thresholds.alert + TH_HYSTERESIS) && sensorData.heatIndex < (thresholds.critical - TH_HYSTERESIS)) {  //hi >= 42.5 && hi < 51.5
    currentState = STATE_ORANGE;
  } else if (sensorData.heatIndex >= (thresholds.warning + TH_HYSTERESIS) && sensorData.heatIndex < (thresholds.alert - TH_HYSTERESIS)) {  // hi >= 33.5 && hi < 41.5
    currentState = STATE_BLUE;
  } else if (sensorData.heatIndex >= (thresholds.normal + TH_HYSTERESIS) && sensorData.heatIndex < (thresholds.warning - TH_HYSTERESIS)) {  // hi >= 27.5 && hi < 32.5
    currentState = STATE_GREEN;
  } else if (sensorData.heatIndex < thresholds.normal) {  //   hi < 27
    currentState = STATE_INIT;
  }

  // Only update lights if state changed
  if (currentState != prevState) {
    String _payload = "";
    switch (currentState) {
      case STATE_RED:
        towerLights.red = true;
        towerLights.orange = towerLights.blue = towerLights.green = false;

        // Update Modbus Relay
        sendTowerLightsMulti(1, 0);

        // Update web
        _payload = "{\"hi\":\"" + String(sensorData.heatIndex, 2) + "\", \"t\":\"" + String(sensorData.temperature, 2) + "\", \"h\":\"" + String(sensorData.humidity, 2) + "\"}";
        if (isMQTTConnectedGSM) {
          mqttGSM.publish(topicSensorData.c_str(), _payload.c_str());
          vTaskDelay(pdMS_TO_TICKS(500));

          mqttGSM.publish(topicRequestThreshold.c_str(), "reqTH");
          vTaskDelay(pdMS_TO_TICKS(500));
          mqttGSM.publish(topicRequestDateTime.c_str(), "SAMPLE");
        } else if (isMQTTConnectedWiFi) {
          mqttWiFi.publish(topicSensorData.c_str(), _payload.c_str());
          vTaskDelay(pdMS_TO_TICKS(500));

          mqttWiFi.publish(topicRequestThreshold.c_str(), "reqTH");
          vTaskDelay(pdMS_TO_TICKS(500));
          mqttWiFi.publish(topicRequestDateTime.c_str(), "SAMPLE");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));

        if (prevState == STATE_ORANGE || prevState == STATE_BLUE || prevState == STATE_GREEN || prevState == STATE_INIT) {
          Serial.println("Sending SMS Blast");
          for (int x = 0; x < contactCount; x++) {

            //[Time and Date]Heat Index 52°C Critical!Heat stroke risk. Stay in shaded or cool areas.

            String msg = "[" + currentDateTime + "] Heat Index " + String(sensorData.heatIndex, 1) + " C! Critical! Heat stroke risk. Stay in shaded or cool areas. \r\n";
            enqueueSms(contacts[x].c_str(), msg.c_str());
            vTaskDelay(pdMS_TO_TICKS(100));
          }
        }
        break;
      case STATE_ORANGE:
        towerLights.orange = true;
        towerLights.red = towerLights.blue = towerLights.green = false;

        // Update Modbus Relay
        sendTowerLightsMulti(1, 0);

        // Update web
        _payload = "{\"hi\":\"" + String(sensorData.heatIndex, 2) + "\", \"t\":\"" + String(sensorData.temperature, 2) + "\", \"h\":\"" + String(sensorData.humidity, 2) + "\"}";
        if (isMQTTConnectedGSM) {
          mqttGSM.publish(topicSensorData.c_str(), _payload.c_str());
          vTaskDelay(pdMS_TO_TICKS(500));

          mqttGSM.publish(topicRequestThreshold.c_str(), "reqTH");
          vTaskDelay(pdMS_TO_TICKS(500));
          mqttGSM.publish(topicRequestDateTime.c_str(), "SAMPLE");
        } else if (isMQTTConnectedWiFi) {
          mqttWiFi.publish(topicSensorData.c_str(), _payload.c_str());
          vTaskDelay(pdMS_TO_TICKS(500));

          mqttWiFi.publish(topicRequestThreshold.c_str(), "reqTH");
          vTaskDelay(pdMS_TO_TICKS(500));
          mqttWiFi.publish(topicRequestDateTime.c_str(), "SAMPLE");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));

        if (prevState == STATE_BLUE || prevState == STATE_GREEN || prevState == STATE_INIT) {
          Serial.println("Sending SMS Blast");
          for (int x = 0; x < contactCount; x++) {
            String msg = "[" + currentDateTime + "] Heat Index " + String(sensorData.heatIndex, 1) + " C! Alert! Fatigue possible. Drink water and rest. \r\n";
            enqueueSms(contacts[x].c_str(), msg.c_str());
            vTaskDelay(pdMS_TO_TICKS(100));
          }
        }
        break;
      case STATE_BLUE:
        towerLights.blue = true;
        towerLights.red = towerLights.orange = towerLights.green = false;

        // Update Modbus Relay
        sendTowerLightsMulti(1, 0);

        // Update web
        _payload = "{\"hi\":\"" + String(sensorData.heatIndex, 2) + "\", \"t\":\"" + String(sensorData.temperature, 2) + "\", \"h\":\"" + String(sensorData.humidity, 2) + "\"}";
        if (isMQTTConnectedGSM) {
          mqttGSM.publish(topicSensorData.c_str(), _payload.c_str());
          vTaskDelay(pdMS_TO_TICKS(500));

          mqttGSM.publish(topicRequestThreshold.c_str(), "reqTH");
        } else if (isMQTTConnectedWiFi) {
          mqttWiFi.publish(topicSensorData.c_str(), _payload.c_str());
          vTaskDelay(pdMS_TO_TICKS(500));

          mqttWiFi.publish(topicRequestThreshold.c_str(), "reqTH");
        }

        // for (int x = 0; x < contactCount; x++) {
        //   String msg = "Heat Index " + String(sensorData.heatIndex, 1) + " C. Medyo mainit na! Iwas bilad, magpayong at uminom ng maraming tubig. \r\n";
        //   enqueueSms(contacts[x].c_str(), msg.c_str());
        //   vTaskDelay(pdMS_TO_TICKS(100));
        // }
        break;
      case STATE_GREEN:
        towerLights.green = true;
        towerLights.red = towerLights.orange = towerLights.blue = false;

        // Update Modbus Relay
        sendTowerLightsMulti(1, 0);

        // Update web
        _payload = "{\"hi\":\"" + String(sensorData.heatIndex, 2) + "\", \"t\":\"" + String(sensorData.temperature, 2) + "\", \"h\":\"" + String(sensorData.humidity, 2) + "\"}";
        if (isMQTTConnectedGSM) {
          mqttGSM.publish(topicSensorData.c_str(), _payload.c_str());
          vTaskDelay(pdMS_TO_TICKS(500));

          mqttGSM.publish(topicRequestThreshold.c_str(), "reqTH");
        } else if (isMQTTConnectedWiFi) {
          mqttWiFi.publish(topicSensorData.c_str(), _payload.c_str());
          vTaskDelay(pdMS_TO_TICKS(500));

          mqttWiFi.publish(topicRequestThreshold.c_str(), "reqTH");
        }

        // for (int x = 0; x < contactCount; x++) {
        //   String msg = "Heat Index " + String(sensorData.heatIndex, 1) + " C. Presko pa! Ligtas sa labas pero tubig-tubig din para iwas uhaw. \r\n";
        //   enqueueSms(contacts[x].c_str(), msg.c_str());
        //   vTaskDelay(pdMS_TO_TICKS(100));
        // }
        break;
      case STATE_INIT:
        towerLights.green = towerLights.red = towerLights.orange = towerLights.blue = false;

        // Update Modbus Relay
        sendTowerLightsMulti(1, 0);

        // Update web
        _payload = "{\"hi\":\"" + String(sensorData.heatIndex, 2) + "\", \"t\":\"" + String(sensorData.temperature, 2) + "\", \"h\":\"" + String(sensorData.humidity, 2) + "\"}";
        if (isMQTTConnectedGSM) {
          mqttGSM.publish(topicSensorData.c_str(), _payload.c_str());
          vTaskDelay(pdMS_TO_TICKS(500));

          mqttGSM.publish(topicRequestThreshold.c_str(), "reqTH");
        } else if (isMQTTConnectedWiFi) {
          mqttWiFi.publish(topicSensorData.c_str(), _payload.c_str());
          vTaskDelay(pdMS_TO_TICKS(500));

          mqttWiFi.publish(topicRequestThreshold.c_str(), "reqTH");
        }

        // for (int x = 0; x < contactCount; x++) {
        //   enqueueSms(contacts[x].c_str(), "NO ALERT \r\n");
        //   vTaskDelay(pdMS_TO_TICKS(100));
        // }
        break;
    }

    // Debug log
    Serial.print(F("HeatIndex changed state → "));
    Serial.println(currentState);

    // Update last state
    prevState = currentState;
  }

  // Check Server Connectivity -> Restart when disconnected for too long
  if (isMQTTConnectedGSM || isMQTTConnectedWiFi) {
    lastMQTTConnectionTime = millis();
  } else {
    // Check if the ESP has been disconnected for too long
    if (millis() - lastMQTTConnectionTime > MQTT_DISCONNECT_TIMEOUT_MS) {
      Serial.println("MQTT disconnected for too long. Restarting ESP...");
      vTaskDelay(100);
      ESP.restart();  // Restart the ESP
    }
  }



  if (millis() - heartbeatMillis > hbInterval) {

    //     //enqueueSms("+639260694581", "Test SMS from Device\r\n");

    //     String _topic = String(deviceESN) + "/Heartbeat";
    //     String _payload = "Heartbeat";

    //     if (isMQTTConnectedGSM) {
    //       mqttGSM.publish(_topic.c_str(), _payload.c_str());
    //     } else if (isMQTTConnectedWiFi) {
    //       mqttWiFi.publish(_topic.c_str(), _payload.c_str());
    //     }

    // #ifdef DEBUG_MQTT
    //     Serial.println("Sent heartbeat message.");
    // #endif

    // #ifdef DEBUG_SMS
    //     // Serial.println("Sending SMS Blast");
    //     // for(int x = 0; x < contactCount; x++){
    //     //   enqueueSms(contacts[x].c_str(), "Test SMS from Heat Index Project \r\n");
    //     //   vTaskDelay(pdMS_TO_TICKS(100));
    //     // }
    // #endif

    // Update Modbus Slave
    sendTowerLightsMulti(1, 0);

    heartbeatMillis = millis();
  }


  if (millis() - lastSendTime > sendInterval && sensorData.isValid) {
    lastSendTime = millis();


    String _payload = "{\"hi\":\"" + String(sensorData.heatIndex, 2) + "\", \"t\":\"" + String(sensorData.temperature, 2) + "\", \"h\":\"" + String(sensorData.humidity, 2) + "\"}";


    if (isMQTTConnectedGSM) {
      mqttGSM.publish(topicSensorData.c_str(), _payload.c_str());
      vTaskDelay(pdMS_TO_TICKS(500));

      mqttGSM.publish(topicRequestThreshold.c_str(), "reqTH");
    } else if (isMQTTConnectedWiFi) {
      mqttWiFi.publish(topicSensorData.c_str(), _payload.c_str());
      vTaskDelay(pdMS_TO_TICKS(500));

      mqttWiFi.publish(topicRequestThreshold.c_str(), "reqTH");
    }

    if (isMQTTConnectedGSM || isMQTTConnectedWiFi) {
#ifdef DEBUG_MQTT
      Serial.println("Sent heartbeat message.");
      Serial.println("Payload: " + _payload);
      Serial.println("Requested threshold update.");
#endif
    }

    // Update Modbus Slave
    sendTowerLightsMulti(1, 0);
  }
}

// Keep only leading '+' and digits; cap to CONTACT_MAX_LEN-1
static void sanitizeToFixedSlot(const String& in, char out[CONTACT_MAX_LEN]) {
  memset(out, 0, CONTACT_MAX_LEN);
  int oi = 0;
  int si = 0;

  if (in.length() > 0 && in[0] == '+') {  // keep one leading '+'
    out[oi++] = '+';
    si = 1;
  }
  while (si < (int)in.length() && oi < CONTACT_MAX_LEN - 1) {
    char c = in[si++];
    if (c >= '0' && c <= '9') out[oi++] = c;
  }
  out[oi] = '\0';
}

// Save caller-provided list into EEPROM contacts block
bool saveContactsToEEPROM(const String inContacts[], uint8_t inCount) {
  if (inContacts == nullptr) return false;

  // Clamp to layout
  uint8_t count = inCount;
  if (count > MAX_CONTACTS) count = MAX_CONTACTS;

  // Write count
  EEPROM.write(EEPROM_CONTACTS_ADDR, count);

  // Write fixed slots (pad with NULs after count entries)
  const int base = EEPROM_CONTACTS_ADDR + 1;
  for (uint8_t i = 0; i < MAX_CONTACTS; ++i) {
    char slot[CONTACT_MAX_LEN] = { 0 };
    if (i < count) sanitizeToFixedSlot(inContacts[i], slot);

    // Store slot bytes
    for (int j = 0; j < CONTACT_MAX_LEN; ++j) {
      EEPROM.write(base + i * CONTACT_MAX_LEN + j, slot[j]);
    }
  }
  EEPROM.commit();
  return true;
}

// Load EEPROM contacts block into caller buffers
bool loadContactsFromEEPROM(String outContacts[], uint8_t maxOut, uint8_t& outCount) {
  if (outContacts == nullptr || maxOut == 0) {
    outCount = 0;
    return false;
  }

  uint8_t storedCount = EEPROM.read(EEPROM_CONTACTS_ADDR);
  if (storedCount > MAX_CONTACTS) storedCount = 0;  // basic validation

  const int base = EEPROM_CONTACTS_ADDR + 1;
  uint8_t copyCount = (storedCount > maxOut) ? maxOut : storedCount;

  // Load only up to maxOut for the caller; skip the rest
  for (uint8_t i = 0; i < copyCount; ++i) {
    char slot[CONTACT_MAX_LEN];
    for (int j = 0; j < CONTACT_MAX_LEN; ++j) {
      slot[j] = EEPROM.read(base + i * CONTACT_MAX_LEN + j);
    }
    slot[CONTACT_MAX_LEN - 1] = '\0';
    outContacts[i] = String(slot);
  }

  // Clear any remaining caller entries (optional hygiene)
  for (uint8_t i = copyCount; i < maxOut; ++i) {
    outContacts[i] = "";
  }

  outCount = copyCount;
  return true;
}





// Initialize Modbus
void initializeModbus() {
  Serial2.begin(9600, SERIAL_8N1, MASTER_RO, MASTER_DI);
  modbus.begin(&Serial2, MASTER_EN);
  modbus.setBaudrate(9600);
  modbus.master();
}

// Modbus Task Function
void modbusTask(void* pvParameters) {
  for (;;) {
    modbus.task();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void rtcTask(void* pvParameters) {
  for (;;) {
    if (isMQTTConnectedGSM) {
      mqttGSM.publish(topicRequestDateTime.c_str(), "SAMPLE");
    } else if (isMQTTConnectedWiFi) {
      mqttWiFi.publish(topicRequestDateTime.c_str(), "SAMPLE");
    }

    vTaskDelay(30000);
  }
}
