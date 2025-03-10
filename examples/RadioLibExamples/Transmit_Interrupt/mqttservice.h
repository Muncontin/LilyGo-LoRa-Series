#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "DANIELS_PC_2300";
const char* password = "%g3D2085";
const char* mqtt_server = "192.168.0.130";  // Replace with your broker's IP

WiFiClient espClient;
PubSubClient client(espClient);

bool ledState = false;

void setupWifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

void mqttsetup() {
    setupWifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

// Loop use case
// void loop() {
//     if (!client.connected()) {
//         reconnect();
//     }
//     client.loop();
// }

void setupWifi() {
    Serial.printf("Connecting to Wifi on %s %s", ssid, password);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}

void publishMessage(String message) {
    // **NEW: Publish the same payload over MQTT**
    client.publish("ttgo/network", message.c_str());  // Sends to MQTT topic
}

// Receive subscribed messages
void callback(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, "ttgo/network") == 0) {
        Serial.printf("Waiting on message received");
    }
}

void reconnect() {
    while (!client.connected()) {
        if (client.connect("ttgo/network")) {
            client.subscribe("ttgo/network");
        } else {
            delay(5000);
        }
    }
}
