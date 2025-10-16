#include "CLIHandler.h"


//=============================================================================
//                              CLI Task Setup
//=============================================================================
static String cliBuffer = "";  // Buffer to accumulate characters from Serial input

void startCliTask() {

  xTaskCreatePinnedToCore(
    cliTask,
    "CliTask",
    4096,
    NULL,
    1,
    NULL,
    1);
}

//=============================================================================
//                              CLI Task Loop
//=============================================================================

void cliTask(void* pvParameters) {

  for (;;) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        cliBuffer.trim();
        if (cliBuffer.length() > 0) {
          handleCLICommand(cliBuffer);
          cliBuffer = "";
        }
      } else {
        cliBuffer += c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

//=============================================================================
//                            Command Handler
//=============================================================================
void handleCLICommand(String cmd) {
  cmd.trim();

  //-------------------------------------------------------------------------
  //                             HELP COMMAND
  //-------------------------------------------------------------------------
  if (cmd == "AT+HELP" || cmd == "?") {
    Serial.println(F("Available AT Commands:"));
    Serial.println(F("  AT+HELP or ?                    - Show this help message"));
    Serial.println(F("  AT+SETESN=<esn>                 - Set the device ESN"));
    Serial.println(F("  AT+ESN?                         - Show saved ESN"));
    Serial.println(F("  AT+SETMQTT=s,u,p,port           - Set MQTT (server,user,pass,port)"));
    Serial.println(F("  AT+MQTT?                        - Show current MQTT config"));
    Serial.println(F("  AT+SET_HB_INTERVAL=<ms>         - Set heartbeat interval (ms)"));
    Serial.println(F("  AT+GET_HB_INTERVAL              - Show current heartbeat interval"));
    Serial.println(F("  AT+SET_SENSOR_INTERVAL          - Set sensor sending interval (ms)"));
    Serial.println(F("  AT+GET_SENSOR_INTERVAL          - Show sensor sending interval"));
    Serial.println(F("  AT+SETWIFI=ssid,pass            - Save WiFi credentials to EEPROM"));
    Serial.println(F("  AT+WIFI?                        - Show saved WiFi credentials"));
    Serial.println(F("  AT+SETNET=wifi|gsm              - Set network mode (WiFi or GSM)"));
    Serial.println(F("  AT+NET?                         - Show current network mode"));
    Serial.println(F("  AT+RESTART                      - Restart the device"));
    return;
  }



  else if (cmd.startsWith("AT+SETESN=")) {
    String esn = cmd.substring(String("AT+SETESN=").length());
    esn.trim();
    if (esn.length() < sizeof(deviceESN)) {
      esn.toCharArray(deviceESN, sizeof(deviceESN));
      saveDeviceESNToEEPROM(deviceESN);
      Serial.print(F("Device ESN updated to: "));
      Serial.println(deviceESN);
    } else {
      Serial.println(F("ESN too long."));
    }
  }


  else if (cmd == "AT+ESN?") {
    loadDeviceESNFromEEPROM();  // Refresh the ESN from EEPROM in case it's not loaded
    Serial.print(F("[CLI] Saved Device ESN: "));
    Serial.println(deviceESN);
  }


  else if (cmd.startsWith("AT+SETMQTT=")) {
    String params = cmd.substring(String("AT+SETMQTT=").length());
    int firstComma = params.indexOf(',');
    int secondComma = params.indexOf(',', firstComma + 1);
    int thirdComma = params.indexOf(',', secondComma + 1);

    if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma) {
      String newServer = params.substring(0, firstComma);
      String newUser = params.substring(firstComma + 1, secondComma);
      String newPass = params.substring(secondComma + 1, thirdComma);
      int newPort = params.substring(thirdComma + 1).toInt();

      newServer.toCharArray(mqttConfig.mqttServer, sizeof(mqttConfig.mqttServer));
      newUser.toCharArray(mqttConfig.mqttUser, sizeof(mqttConfig.mqttUser));
      newPass.toCharArray(mqttConfig.mqttPassword, sizeof(mqttConfig.mqttPassword));
      mqttConfig.mqttPort = (newPort > 0 && newPort <= 65535) ? newPort : mqttConfig.mqttPort;

      saveMQTTConfigToEEPROM(mqttConfig);
      Serial.println(F("MQTT configuration updated. Please restart"));
    } else {
      Serial.println(F("Invalid format. Use: server,user,password,port"));
    }
  }


  else if (cmd == "AT+MQTT?") {
    Serial.println("MQTT Server: " + String(mqttConfig.mqttServer));
    Serial.println("MQTT User:   " + String(mqttConfig.mqttUser));
    Serial.println("MQTT Pass:   " + String(mqttConfig.mqttPassword));
    Serial.println("MQTT Port:   " + String(mqttConfig.mqttPort));
  }

  else if (cmd.startsWith("AT+SET_HB_INTERVAL=")) {
    String valueStr = cmd.substring(String("AT+SET_HB_INTERVAL=").length());
    uint32_t interval = valueStr.toInt();
    saveHeartbeatInterval(interval);
  }


  else if (cmd == "AT+GET_HB_INTERVAL") {
    uint32_t interval = loadHeartbeatInterval();
    Serial.println("[CLI] Heartbeat Interval: " + String(interval));
  }

  else if (cmd.startsWith("AT+SET_SENSOR_INTERVAL=")) {
    String valueStr = cmd.substring(String("AT+SET_SENSOR_INTERVAL=").length());
    uint32_t interval = valueStr.toInt();
    writeSendIntervalToEEPROM(interval);
  }


  else if (cmd == "AT+GET_SENSOR_INTERVAL") {
    uint32_t interval = readSendIntervalFromEEPROM();
    Serial.println("[CLI] Sensor Interval: " + String(interval));
  }


  else if (cmd == "AT+RESTART") {
    Serial.println(F("[CLI] Restarting device..."));
    delay(100);  // Allow serial print to complete
    ESP.restart();
  }


  else if (cmd.startsWith("AT+SETWIFI=")) {
    String paramStr = cmd.substring(String("AT+SETWIFI=").length());
    int commaIndex = paramStr.indexOf(',');
    if (commaIndex > 0) {
      String ssid = paramStr.substring(0, commaIndex);
      String pass = paramStr.substring(commaIndex + 1);

      WiFiCreds_t creds;
      ssid.toCharArray(creds.ssid, sizeof(creds.ssid));
      pass.toCharArray(creds.password, sizeof(creds.password));

      saveWiFiCredsToEEPROM(creds);
      Serial.println(F("[CLI] WiFi credentials saved to EEPROM."));
      Serial.println(F("Please reboot or reconnect manually."));
    } else {
      Serial.println(F("Invalid format. Use: AT+SETWIFI=ssid,password"));
    }
  }


  else if (cmd == "AT+WIFI?") {
    WiFiCreds_t creds = loadWiFiCredsFromEEPROM();
    Serial.println(F("Stored WiFi Credentials:"));
    Serial.print(F("SSID: "));
    Serial.println(creds.ssid);
    Serial.print(F("PASS: "));
    Serial.println(creds.password);
  }

  else if (cmd.startsWith("AT+SETNET=")) {
    String param = cmd.substring(String("AT+SETNET=").length());
    param.toLowerCase();
    if (param == "wifi") {
      currentNetworkMode = WIFI_ONLY;
      saveNetworkMode(currentNetworkMode);
      Serial.println("[CLI] Network mode set to WIFI_ONLY");
    } else if (param == "gsm") {
      currentNetworkMode = GSM_ONLY;
      saveNetworkMode(currentNetworkMode);
      Serial.println("[CLI] Network mode set to GSM_ONLY");
    } else {
      Serial.println(F("Invalid option. Use: wifi, gsm, or both"));
    }
  }

  else if (cmd == "AT+NET?") {
    Serial.print(F("[CLI] Current Network Mode: "));
    switch (currentNetworkMode) {
      case WIFI_ONLY: Serial.println("WIFI_ONLY"); break;
      case GSM_ONLY: Serial.println("GSM_ONLY"); break;
      case WIFI_GSM: Serial.println("WIFI_GSM"); break;
      default: break;
    }
  }


  else {
    Serial.println(F("Unknown AT command. Type AT+HELP for a list of available commands."));
  }
}
