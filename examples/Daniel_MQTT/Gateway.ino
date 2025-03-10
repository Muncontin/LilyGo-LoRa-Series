#include "LoRaBoards.h"
#include <painlessMesh.h>
//#include <WiFiClient.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define MESH_PREFIX "dan-geon mesh-i"
#define MESH_PASSWORD "password"
#define MESH_PORT 5555

// Prototypes
//void receivedMeshCallback( const uint32_t &from, const String &msg);
void receivedMqttCallback(char* topic, byte* payload, unsigned int length);

// External MQTT Broker Info
IPAddress mqttBroker(192,168,137,1);    // declares MQTT broker IP
//const char* mqtt_server = "192.168.1.100";  // Local MQTT broker IP
//const int mqtt_port = 1883;
const char* mqtt_user = "your_username";  // Optional
const char* mqtt_password = "your_password";  // Optional

unsigned long previousMillis = 0;
unsigned long interval = 30000;

IPAddress gatewayIp(192,168,137,10);

Scheduler userScheduler;
painlessMesh mesh;
WiFiClient espClient;
PubSubClient mqttClient(mqttBroker, 1883, receivedMqttCallback, espClient);

// Callback function for receiving mesh messages
void receivedMeshCallback(const uint32_t from, const String &msg) {
    Serial.printf("Mesh message from %u: %s\n", from, msg.c_str());
    
    // Forward mesh message to MQTT topic
    mqttClient.publish("mesh/data", msg.c_str());
}

// Callback for receiving MQTT messages
void receivedMqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg = "";
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }
    Serial.printf("MQTT received: [%s] %s\n", topic, msg.c_str());

    // Broadcast message to Mesh
    mesh.sendBroadcast(msg);
}

void initWifi() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin("DANIELS_PC_2300", "%g3D2085");
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
}

void reconnectWifi(unsigned long currentMillis) {
    // checks if the wifi has disconnected and forces a reconnect
    while ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
        Serial.print(millis());
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
        previousMillis = currentMillis;
    }
}

// MQTT Reconnection Logic
void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("LilyGO_Gateway")) { // if need to, add username and pw
            Serial.println("connected!");
            mqttClient.publish("mesh/fromMesh", "HELLO!");
            mqttClient.subscribe("mesh/toMesh/#"); // Subscribe to mesh commands
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" retrying in 5 seconds...");
            delay(5000);
        }
    }
}

IPAddress getlocalIP() {
    return IPAddress(mesh.getStationIP());
}

void setup() {
    Serial.begin(115200);
    initWifi();

    Serial.println("Starting mesh connection");
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedMeshCallback);
    mesh.setRoot(true);
    mesh.setContainsRoot(true);

    //mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(receivedMqttCallback);
}

void loop() {
    unsigned long currentMillis = millis();
    mesh.update();
    mqttClient.loop();

    //Serial.println(WiFi.status());
    reconnectWifi(currentMillis);

    if(gatewayIp != getlocalIP()){
        gatewayIp = getlocalIP();
        Serial.println("Set IP to " + gatewayIp.toString());

        if (!mqttClient.connected()) {
            //reconnectMQTT();
        }
    }
}