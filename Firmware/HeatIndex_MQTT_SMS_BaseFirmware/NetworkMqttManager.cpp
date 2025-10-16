#include "freertos/projdefs.h"
#include <WiFi.h>
#include "NetworkMqttManager.h"
#include "SystemConfig.h"        // expects: mqttConfig, willTopic, willMessage, initTopic, load*FromEEPROM(), initializeDynamicTopics(), WiFiCreds_t, etc.
#include "esp_task_wdt.h"

// ================== FreeRTOS ==================
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// ================== GSM / TinyGSM ==================
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>

// ================== Serials ==================
#define SerialMon   Serial
#define GSMSerial   Serial1

// ================== GSM Credentials ==================
static const char apn[]     = "Internet";
static const char gprsUser[] = "";
static const char gprsPass[] = "";

// ================== Globals ==================
bool isMQTTConnectedWiFi = false;
bool isMQTTConnectedGSM  = false;
bool wifiConnected       = false;
bool gsmConnected        = false;

// Some projects define this in SystemConfig.
// Keep compatibility if it's there, else default to WIFI_GSM.
#ifndef DEFAUL_NETWORK_MODE
#define DEFAUL_NETWORK_MODE WIFI_GSM
#endif
NetworkMode currentNetworkMode = DEFAUL_NETWORK_MODE;

WiFiCreds_t creds;

// Clients
TinyGsm       modem(GSMSerial);
TinyGsmClient gsmClient(modem);
WiFiClient    wifiClient;
PubSubClient  mqttWiFi(wifiClient);
PubSubClient  mqttGSM(gsmClient);

static int ledPin   = -1;
static int powerPin = -1;

// ================== Modem Guard + SMS Queue ==================
static SemaphoreHandle_t g_modemMutex = nullptr;
static QueueHandle_t     g_smsQueue   = nullptr;

struct SmsMsg {
  char number[24];   // +63xxxxxxxxxx etc.
  char text[161];    // 160 chars + NUL
};

static inline bool lockModem(uint32_t timeout_ms) {
  if (!g_modemMutex) return false;
  return xSemaphoreTake(g_modemMutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
static inline void unlockModem() {
  if (g_modemMutex) xSemaphoreGive(g_modemMutex);
}

// ================== User Hooks ==================
static MqttOnConnectHandler  g_onConnect = nullptr;
static MqttMessageCallback   g_userMsgCb = nullptr;

void setMqttOnConnectHandler(MqttOnConnectHandler handler) { g_onConnect = handler; }
void setMqttMessageCallback(MqttMessageCallback cb)        { g_userMsgCb = cb; }

// ================== Forward Decls ==================
static void wifiTask(void* pvParameters);
static void gsmTask(void* pvParameters);
static void mqttTask(void* pvParameters);
static void smsTask(void* pvParameters);
static void internalMqttCallback(char* topic, uint8_t* payload, unsigned int len);

// ================== MQTT Helpers ==================
static bool mqttConnect(PubSubClient& mqttClient, uint8_t connectivity);

// ================== Internal MQTT Callback ==================
static void internalMqttCallback(char* topic, uint8_t* payload, unsigned int len) {
  if (g_userMsgCb) {
    g_userMsgCb(topic, payload, len);
  } else {
    // Minimal fallback logger
    SerialMon.printf("MQTT [%s] ", topic);
    SerialMon.write(payload, len);
    SerialMon.println();
  }
}

// ================== GSM Helpers ==================
static bool connectGPRS() {
  SerialMon.println("Connecting to GPRS...");

  // Ensure serial is up (RX=17, TX=16 — change if your wiring differs)
  GSMSerial.begin(9600, SERIAL_8N1, 16, 17);

  if (!lockModem(5000)) {
    SerialMon.println("connectGPRS(): modem lock timeout");
    return false;
  }

  bool ok = false;
  do {
    if (!modem.waitForNetwork(10000)) {  // 10s
      SerialMon.println("Failed to connect to GSM network.");
      break;
    }
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println("Failed to connect to GPRS.");
      break;
    }
    ok = true;
  } while (0);

  unlockModem();

  if (ok) SerialMon.println("Connected to GPRS.");
  return ok;
}

// ================== WiFi Task ==================
static void wifiTask(void* pvParameters) {
  Serial.printf("WiFi Task Core %d\n", xPortGetCoreID());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  vTaskDelay(pdMS_TO_TICKS(1000));

  creds = loadWiFiCredsFromEEPROM();
  //WiFi.begin(String(creds.ssid).c_str(), String(creds.password).c_str());
  WiFi.begin("PLDTinnov", "Password12345!");
  Serial.println("Attempting WiFi connection...");

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiConnected) {
        Serial.println("WiFi Connected!");
        wifiConnected = true;

        mqttWiFi.setClient(wifiClient);
        mqttWiFi.setServer(mqttConfig.mqttServer, mqttConfig.mqttPort);
        mqttWiFi.setCallback(internalMqttCallback);
      }
    } else {
      if (wifiConnected) {
        Serial.println("WiFi Disconnected! Retrying...");
        wifiConnected = false;
        WiFi.disconnect();
        WiFi.begin(String(creds.ssid).c_str(), String(creds.password).c_str());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ================== GSM Task ==================
static void gsmTask(void* pvParameters) {
  if (powerPin != -1) pinMode(powerPin, OUTPUT);

  while (true) {
    Serial.println("📶 Attempting GSM Connection...");

    GSMSerial.begin(9600);
    vTaskDelay(pdMS_TO_TICKS(1000));

    bool alive = false;
    if (lockModem(3000)) { alive = modem.testAT(); unlockModem(); }

    if (!alive) {
      // Toggle power key sequence (adjust to your board requirement)
      for (int attempt = 0; attempt < 3 && !alive; attempt++) {
        SerialMon.println("Toggling GSM Power Key...");
        digitalWrite(powerPin, LOW);   vTaskDelay(pdMS_TO_TICKS(2000));
        digitalWrite(powerPin, HIGH);  vTaskDelay(pdMS_TO_TICKS(2000));
        digitalWrite(powerPin, LOW);   vTaskDelay(pdMS_TO_TICKS(2000));
        if (lockModem(3000)) { alive = modem.testAT(); unlockModem(); }
      }
    }

    if (!connectGPRS()) {
      Serial.println("🚨 GSM Connection Failed! Retrying in 20s...");
      vTaskDelay(pdMS_TO_TICKS(20000));
      continue;
    }

    Serial.println("✅ GSM Connected!");
    gsmConnected = true;

    mqttGSM.setClient(gsmClient);
    mqttGSM.setServer(mqttConfig.mqttServer, mqttConfig.mqttPort);
    mqttGSM.setCallback(internalMqttCallback);

    // Keep-alive loop; reconnection handled in mqttTask
    while (true) { vTaskDelay(pdMS_TO_TICKS(5000)); }
  }
}

// ================== MQTT Task ==================
static void mqttTask(void* pvParameters) {
  uint32_t lastReconnectAttemptWiFi = millis();
  uint32_t lastReconnectAttemptGSM  = millis();

  loadMQTTConfigFromEEPROM();
  loadDeviceESNFromEEPROM();
  initializeDynamicTopics();

  while (true) {
    uint32_t now = millis();

    // ---- WiFi MQTT (no modem lock needed) ----
    if (wifiConnected) {
      if (!mqttWiFi.connected()) {
        isMQTTConnectedWiFi = false;
        if (now - lastReconnectAttemptWiFi > 10000) {
          lastReconnectAttemptWiFi = now;
          if (mqttConnect(mqttWiFi, WIFI_CONNECTED)) {
            isMQTTConnectedWiFi = true;
          }
        }
      } else {
        isMQTTConnectedWiFi = true;
        lastReconnectAttemptWiFi = now;
        mqttWiFi.loop();
      }
    }

    // ---- GSM MQTT (guard with modem mutex) ----
    if (gsmConnected) {
      if (!mqttGSM.connected()) {
        isMQTTConnectedGSM = false;
        if (now - lastReconnectAttemptGSM > 10000) {
          lastReconnectAttemptGSM = now;
          if (mqttConnect(mqttGSM, GSM_CONNECTED)) {
            isMQTTConnectedGSM = true;
          }
        }
      } else {
        isMQTTConnectedGSM = true;
        lastReconnectAttemptGSM = now;
        if (lockModem(2000)) { mqttGSM.loop(); unlockModem(); }
      }
    }

    // LED state
    if (isMQTTConnectedGSM || isMQTTConnectedWiFi) {
      if (ledPin != -1) digitalWrite(ledPin, HIGH);
    } else {
      if (ledPin != -1) digitalWrite(ledPin, LOW);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ================== MQTT Connect Helper ==================
static bool mqttConnect(PubSubClient& mqttClient, uint8_t connectivity) {
  mqttClient.setKeepAlive(120);
  mqttClient.setSocketTimeout(60);

  const bool isWifi = (connectivity == WIFI_CONNECTED);

  if (isWifi) {
    SerialMon.println("Connecting to MQTT broker via WiFi...");
    if (mqttClient.connect(deviceESN, mqttConfig.mqttUser, mqttConfig.mqttPassword,
                           willTopic.c_str(), 1, false, willMessage.c_str())) {
      SerialMon.println("Connected to MQTT via WiFi");
      //mqttClient.publish(initTopic.c_str(), "Wi-Fi MQTT Client Started");

      // Let the sketch (re)subscribe
      if (g_onConnect) g_onConnect(mqttClient, true);
      return true;
    }
    SerialMon.println("Failed to connect to MQTT via WiFi");
    return false;
  } else {
    SerialMon.println("Connecting to MQTT broker via GSM...");
    if (!lockModem(10000)) {
      SerialMon.println("mqttConnect(GSM): modem lock timeout");
      return false;
    }
    bool ok = mqttClient.connect(deviceESN, mqttConfig.mqttUser, mqttConfig.mqttPassword,
                                 willTopic.c_str(), 1, false, willMessage.c_str());
    unlockModem();

    if (ok) {
      SerialMon.println("Connected to MQTT via GSM");
      //mqttClient.publish(initTopic.c_str(), "GSM MQTT Client Started");

      // Re-subscribe under modem lock to serialize AT ops
      if (g_onConnect) {
        if (lockModem(3000)) { g_onConnect(mqttClient, false); unlockModem(); }
      }
      return true;
    }
    SerialMon.println("Failed to connect to MQTT via GSM");
    return false;
  }
}

// ================== SMS Task ==================
static void smsTask(void* pvParameters) {
  // Configure text mode
  if (lockModem(5000)) {
    modem.sendAT("+CMGF=1");             modem.waitResponse(1000);
    modem.sendAT("+CNMI=2,1,0,0,0");     modem.waitResponse(1000);
    unlockModem();
  }

  SmsMsg msg;
  for (;;) {
    if (xQueueReceive(g_smsQueue, &msg, portMAX_DELAY) == pdTRUE) {
      bool sent = false;
      if (lockModem(15000)) {
        Serial.println("Sending SMS");
        Serial.print("MSG: "); Serial.println(msg.text);
        Serial.print("NUM: "); Serial.println(msg.number);
        sent = modem.sendSMS(msg.number, msg.text);
        vTaskDelay(pdMS_TO_TICKS(1000));
        unlockModem();
      }
      // Report status through whichever MQTT is up
      if (isMQTTConnectedWiFi) {
        mqttWiFi.publish("device/sms/status", sent ? "OK" : "FAIL");
      } else if (isMQTTConnectedGSM) {
        if (lockModem(2000)) { mqttGSM.publish("device/sms/status", sent ? "OK" : "FAIL"); unlockModem(); }
      }
    }
  }
}

// ================== Public API ==================
bool enqueueSms(const char* number, const char* text) {
  if (!g_smsQueue || !number || !text) return false;
  SmsMsg m{}; strlcpy(m.number, number, sizeof m.number); strlcpy(m.text, text, sizeof m.text);
  return xQueueSend(g_smsQueue, &m, 0) == pdPASS;
}

void startNetworkMqttManagerTask(int pwrPin, int led, NetworkMode mode) {
  powerPin = pwrPin;
  ledPin   = led;

  if (ledPin != -1) { pinMode(ledPin, OUTPUT); digitalWrite(ledPin, LOW); }

  Serial.println("Starting Network MQTT Manager...");

  // Create mutex & queue before tasks
  if (!g_modemMutex) g_modemMutex = xSemaphoreCreateMutex();
  if (!g_smsQueue)   g_smsQueue   = xQueueCreate(35, sizeof(SmsMsg));

  // SMS worker
  xTaskCreatePinnedToCore(smsTask, "SMS Task", 4096, NULL, 2, NULL, 1);

  if (mode == WIFI_ONLY || mode == WIFI_GSM) {
    Serial.println("📡 WiFi Mode Enabled");
    xTaskCreatePinnedToCore(wifiTask, "WiFi Task", 8192, NULL, 2, NULL, 0);
  }
  if (mode == GSM_ONLY || mode == WIFI_GSM) {
    Serial.println("📶 GSM Mode Enabled");
    xTaskCreatePinnedToCore(gsmTask, "GSM Task", 8192, NULL, 2, NULL, 1);
  }

  Serial.println("⚙️ Starting MQTT Task...");
  xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 16384, NULL, 1, NULL, 1);
}
