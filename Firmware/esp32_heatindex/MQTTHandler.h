#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>


class MQTTHandler {
  public:
    MQTTHandler();
    void init(const char* mqttServer, int mqttPort, const char* mqttUser, const char* mqttPassword, const char* deviceESN, const char* willTopic, const char* willMessage);
    boolean connect();
    boolean checkConnectivity();

    void setInitialMessage(const char* topic, const char* message); // Set dynamic initial message
    void addSubscriptionTopic(const char* topic); // Add topics to subscribe dynamically


    void subscribe(const char* topic);
    void publish(const char* topic, const char* payload);
    boolean messageAvailable(); // Check if there is an incoming message
    String getMessageTopic();   // Get the topic of the incoming message
    String getMessagePayload(); // Get the payload of the incoming message
    void clearMessageFlag();    // Method to clear the flag indicating an incoming message
    
  private:
    WiFiClient _espClient;
    PubSubClient _mqttClient;
    String _incomingTopic;
    String _incomingPayload;
    bool _messageAvailable; // Flag to indicate if there is an incoming message


    const char* _mqttServer;
    int _mqttPort;
    const char* _mqttUser;
    const char* _mqttPassword;
    const char* _deviceESN;
    const char* _willTopic;
    const char* _willMessage;

    String _initialMessageTopic;
    String _initialMessagePayload;
    std::vector<String> _subscriptionTopics; // List of topics to subscribe to

    void callback(char* topic, byte* payload, unsigned int length);
    void subscribeToTopics(); // Internal function to subscribe to all topics
};

#endif