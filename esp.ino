
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "secrets.h"
#include <ArduinoJson.h>
#include <math.h>
#include <1euroFilter.h>

#define RXp2 16
#define TXp2 17

// WiFi & AWS IoT Client
#define AWS_IOT_PUBLISH_TOPIC "esp32/mpu6050"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

#define WIFI_SSID "Threat"
#define WIFI_PASSWORD "ambrosia123"

#define SAMPLE_RATE 10  // 10Hz sampling rate
#define HIGH_PASS_CUTOFF 0.2  // 0.2Hz high-pass filter
#define LOW_PASS_CUTOFF 5.0  // 5Hz low-pass filter

#define BUFFER_SIZE 10  // Buffer size for sliding window
float buffer[BUFFER_SIZE];  
int bufferIndex = 0; 

// High-pass and Low-pass Filters
OneEuroFilter highPassFilter(SAMPLE_RATE, HIGH_PASS_CUTOFF, 1.0, 0.0);
OneEuroFilter lowPassFilter(SAMPLE_RATE, LOW_PASS_CUTOFF, 1.0, 0.0);

WiFiClientSecure net;
PubSubClient client(net);

// Calculate Instrumental Intensity Function (IIF)
float calculateIfF(float af) {
    if (af > 0) 
        return (2.0 * log10(af) + 0.94);
    return 0;
}

// Calculate Richter scale magnitude correctly
float calculateRichter(float amplitude) {
    const float S = 0.001; // Standard earthquake amplitude in mm
    const float B = 0.0;   // Distance correction factor (assumed 0 for now)
    if (amplitude > 0) {
        float M = log10(amplitude / S) + B;
        return max(M, 0.0f); // Ensure no negative values
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

        if (millis() - startAttemptTime > 20000) {
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

        if (millis() - startAttemptTime > 20000) {
            Serial.println("\n[ERROR] AWS IoT Connection Timeout!");
            return;
        }
    }

    Serial.println("\n[AWS IoT] Connected!");
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
}

// Read MPU6050 Data from Arduino
String readMPU6050Data() {
    if (Serial2.available()) {
        String data = Serial2.readStringUntil('\n'); 
        data.trim();
        Serial.println("[MPU6050] Data Received: " + data);
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
        float acc_x = doc["acc_x"].as<float>();
        float acc_y = doc["acc_y"].as<float>();
        float acc_z = doc["acc_z"].as<float>() - 9.8; // Remove gravity effect
        
        float af = sqrt(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2));

        // Apply high-pass and low-pass filtering
        af = highPassFilter.filter(af);
        af = lowPassFilter.filter(af);

        float iif = calculateIfF(af);
        float richter = calculateRichter(af);

        Serial.print("[MPU6050] Filtered Acceleration (af): ");
        Serial.println(af);
        Serial.print("[MPU6050] Instrumental Intensity Function (iif): ");
        Serial.println(iif);
        Serial.print("[MPU6050] Richter Magnitude: ");
        Serial.println(richter);
        Serial.println("[MPU6050] Location: my location");

        // Publish JSON data to AWS IoT
        StaticJsonDocument<128> payloadDoc;
        payloadDoc["richter"] = richter;
        payloadDoc["iif"] = iif;
        payloadDoc["af"] = af;
        payloadDoc["location"] = "my location";

        String payload;
        serializeJson(payloadDoc, payload);
        client.publish(AWS_IOT_PUBLISH_TOPIC, payload.c_str());
    }
}


// Setup
void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
    connectToWiFi();
    connectAWS();
}

// Main Loop
void loop() {
    if (!client.connected()) connectAWS();
    client.loop();
    String data = readMPU6050Data();
    if (!data.isEmpty()) sendMPUData(data);
    delay(2000);
}
