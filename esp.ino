
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "secrets.h"
#include <ArduinoJson.h>
#include <math.h>

#define RXp2 16
#define TXp2 17

// WiFi & AWS IoT Client
#define AWS_IOT_PUBLISH_TOPIC "esp32/mpu6050"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"
#define WIFI_SSID "Threat"
#define WIFI_PASSWORD "ambrosia123"

WiFiClientSecure net;
PubSubClient client(net);

#define EARTHQUAKE_THRESHOLD 2  // Define an appropriate threshold for earthquake-like vibrations

// Calculate Instrumental Intensity Function (IIF)
float calculateIfF(float af) {
    if (af > 0) {
        return (2.0 * log10(af) + 0.94);
    }
    return 0;
}

// WiFi Connection
void connectToWiFi() {
    Serial.print("[WiFi] Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);

        if (millis() - startAttemptTime > 20000) { // Timeout after 20 sec
            Serial.println("\n[ERROR] WiFi Connection Timeout!");
            return;
        }
    }

    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
}

// AWS IoT MQTT Callback
void messageHandler(char* topic, byte* payload, unsigned int length) {
    Serial.print("[AWS IoT] Message received on topic: ");
    Serial.println(topic);

    Serial.print("[AWS IoT] Payload: ");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println("\n[AWS IoT] Processing command...");
}

// AWS IoT Connection
void connectAWS() {
    Serial.println("[AWS IoT] Setting up secure certificates...");
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);

    Serial.println("[AWS IoT] Connecting to AWS IoT...");
    unsigned long startAttemptTime = millis();

    while (!client.connect("ESP32_MPU6050")) {
        Serial.print(".");
        delay(1000);

        if (millis() - startAttemptTime > 20000) { // Timeout after 20 sec
            Serial.println("\n[ERROR] AWS IoT Connection Timeout!");
            return;
        }
    }

    Serial.println("\n[AWS IoT] Connected!");
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    Serial.print("[AWS IoT] Subscribed to: ");
    Serial.println(AWS_IOT_SUBSCRIBE_TOPIC);
}

// Read MPU6050 Data from Arduino
String readMPU6050Data() {
    if (Serial2.available()) {
        String data = Serial2.readStringUntil('\n'); 
        data.trim();
        Serial.print("[DEBUG] Received from Arduino: ");
        Serial.println(data);
        return data;
    }
    return "";
}

// Send MPU6050 Data to AWS
void sendMPUData(String data) {
    StaticJsonDocument<256> doc;

    DeserializationError error = deserializeJson(doc, data);
    if (error) {
        Serial.print("[ERROR] JSON Parsing Failed: ");
        Serial.println(error.c_str());
        return;
    }

    if (doc.containsKey("acc_x") && doc.containsKey("acc_y") && doc.containsKey("acc_z")) {
        // Calculate af as the magnitude of acceleration vector
        float acc_x = doc["acc_x"];
        float acc_y = doc["acc_y"];
        float acc_z = doc["acc_z"];
        
        float af = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

        // Calculate IIF
        float iif = calculateIfF(af);

        if (iif >= EARTHQUAKE_THRESHOLD) {  // Send data only if IIF exceeds the threshold
            StaticJsonDocument<128> payloadDoc;
            payloadDoc["af"] = af;
            payloadDoc["iif"] = iif;  // Use IIF
            payloadDoc["data_from"] = "esp32 Threat";

            String payload;
            serializeJson(payloadDoc, payload);

            Serial.print("[AWS IoT] Sending: ");
            Serial.println(payload);

            if (!client.publish(AWS_IOT_PUBLISH_TOPIC, payload.c_str())) {
                Serial.println("[ERROR] MQTT Publish Failed!");
            }
        } else {
            Serial.println("[INFO] Vibration below earthquake threshold, skipping transmission.");
        }
    } else {
        Serial.println("[ERROR] Missing acceleration components in MPU6050 JSON data!");
    }
}

// Setup Function
void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2); // Use appropriate RX/TX pins

    Serial.println("\n=== ESP32 AWS IoT MPU6050 Data Logger ===");
    
    connectToWiFi();
    connectAWS();
}

// Main Loop
void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Lost Connection! Reconnecting...");
        connectToWiFi();
    }

    if (!client.connected()) {
        Serial.println("[AWS IoT] MQTT Disconnected! Reconnecting...");
        connectAWS();
    }

    client.loop();  // Keep MQTT running

    String data = readMPU6050Data();
    if (data.length() > 0) {
        sendMPUData(data);
    }

    delay(500);
}
