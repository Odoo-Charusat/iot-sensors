#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(9600);
    while (!Serial)
        delay(10); // Pause for serial to open

    Serial.println("Initializing MPU6050...");

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip!");
        while (1) { delay(10); }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Format data as JSON
    Serial.print("{\"acc_x\":");
    Serial.print(a.acceleration.x);
    Serial.print(", \"acc_y\":");
    Serial.print(a.acceleration.y);
    Serial.print(", \"acc_z\":");
    Serial.print(a.acceleration.z);
    Serial.print(", \"gyro_x\":");
    Serial.print(g.gyro.x);
    Serial.print(", \"gyro_y\":");
    Serial.print(g.gyro.y);
    Serial.print(", \"gyro_z\":");
    Serial.print(g.gyro.z);
    Serial.print(", \"temp\":");
    Serial.print(temp.temperature);
    Serial.println("}");

    delay(5000); // Send data every 500ms
}
