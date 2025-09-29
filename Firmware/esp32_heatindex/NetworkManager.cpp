#include "NetworkManager.h"
#include "SystemConfig.h"

TaskHandle_t xTaskHandle_NetworkMonitor = NULL; // Define the task handle
NetworkInfo_t networkInfo = {false, "", "", 0, false};


void startNetworkMonitorTask() {
    xTaskCreatePinnedToCore(
        NetworkMonitorTask,          // Task function
        "Network Monitor",           // Task name
        4096,                        // Stack size
        NULL,                        // Task parameters
        1,                           // Task priority
        &xTaskHandle_NetworkMonitor, // Task handle
        1                            // Core ID
    );
}

void NetworkMonitorTask(void* pvParameters) {
    Serial.print("Network Monitoring running on core ");
    Serial.println(xPortGetCoreID());

    vTaskDelay(1000);
    WiFiManager wm;

    // reset settings - wipe stored credentials for testing
     //wm.resetSettings();

    bool res;
    wm.setConfigPortalTimeout(240);
    wm.setConnectTimeout(15);
    wm.setWiFiAutoReconnect(true);
    
    res = wm.autoConnect(deviceESN, "innovation");
    

    if (!res) {
#ifdef DEBUG_WIFI
        Serial.println("Failed to connect \r\n Restarting device now . . . ");
#endif
        ESP.restart();
    } 
    else {
#ifdef DEBUG_WIFI
        Serial.println("Connected to: " + WiFi.SSID());
#endif

      networkInfo.wifiConnected = true;
      networkInfo.SSID = wm.getWiFiSSID(true);
      networkInfo.password = wm.getWiFiPass(true);
      networkInfo.RSSI = WiFi.RSSI();

    }

    for (;;) {

      networkInfo.wifiConnected = WiFi.status() == WL_CONNECTED ?  true : false;
      networkInfo.RSSI = WiFi.RSSI();
      
      vTaskDelay(1); // Keep the task alive

    }
}