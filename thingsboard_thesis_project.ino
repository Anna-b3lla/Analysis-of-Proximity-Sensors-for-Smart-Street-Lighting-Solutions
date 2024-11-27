#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "Galaxy A13 5G 00E5";       // Replace with your WiFi SSID
const char* password = "tniw3807";             // Replace with your WiFi Password

// ThingsBoard configuration
const char* mqttServer = "demo.thingsboard.io";
const int mqttPort = 1883;
const char* accessToken = "12jlczjq86v3m8c7ct29"; // Replace with your device's access token

WiFiClient espClient;
PubSubClient client(espClient);

// Define RD-03 Radar Sensor pins
#define RD03_DIGITAL_PIN 18 // OT1_PIN (Digital output)
#define RD03_ANALOG_PIN 19  // Analog pin for radar sensor reading (change as needed)

// Define Ultrasonic pins
#define ULTRASONIC_TRIG 12
#define ULTRASONIC_ECHO 14

// Define Laser Ping Sensor pin
#define LASERPING_PIN 13  // SIG pin connected to analog input 13

// Define PIR Sensor pin
#define PIR_PIN 16

// Function to connect to WiFi
void setup_wifi() {
    delay(10);
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
}

// Function to connect to ThingsBoard
void reconnect() {
    while (!client.connected()) {
        Serial.println("Connecting to ThingsBoard...");
        if (client.connect("ESP32Client", accessToken, NULL)) {
            Serial.println("Connected to ThingsBoard!");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" Try again in 5 seconds");
            delay(5000);
        }
    }
}

// Function to read Ultrasonic sensor
float readUltrasonic() {
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    float duration = pulseIn(ULTRASONIC_ECHO, HIGH, 60000); // 60ms timeout

    // Convert to distance in cm
    if (duration == 0) {
        return -1; // No echo detected
    }
    return duration * 0.034 / 2;
}

// Function to read Laser Ping Sensor
float readLaserPing(int& rawValue, float& voltage) {
    rawValue = analogRead(LASERPING_PIN);
    voltage = rawValue * (5.0 / 1023.0);
    if (voltage < 0.4) {
        return 0.0; // Out of range
    } else if (voltage > 2.5) {
        return 200.0; // Clamp to max range
    }
    return (voltage - 0.4) * 100.0;
}

// Function to read PIR sensor (output 1 for motion, 0 for no motion)
int readPIR() {
    return digitalRead(PIR_PIN) == HIGH ? 1 : 0;
}

// Function to read RD-03 Radar Sensor Analog Output
int readRadarAnalog() {
    return analogRead(RD03_ANALOG_PIN);  // Read analog value from radar sensor pin
}

// Function to read RD-03 Radar Sensor (Digital Output Only)
String readRD03Digital() {
    int digitalValue = digitalRead(RD03_DIGITAL_PIN);
    return digitalValue == HIGH ? "Motion Detected" : "No Motion";
}

void setup() {
    Serial.begin(115200);

    // Setup pins
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(LASERPING_PIN, INPUT);
    pinMode(PIR_PIN, INPUT);
    pinMode(RD03_DIGITAL_PIN, INPUT);
    pinMode(RD03_ANALOG_PIN, INPUT);  // Make sure the radar analog pin is set up correctly

    // Connect to WiFi and ThingsBoard
    setup_wifi();
    client.setServer(mqttServer, mqttPort);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Read sensors
    float ultrasonicDistance = readUltrasonic();
    int laserRawValue = 0;
    float laserVoltage = 0.0;
    float laserDistance = readLaserPing(laserRawValue, laserVoltage);
    String radarDigitalStatus = readRD03Digital();
    int pirMotionDetected = readPIR();  // Get motion detected (1 or 0)
    int radarAnalogValue = readRadarAnalog();  // Read the radar sensor's analog value

    // Prepare payload
    String payload = "{";
    payload += "\"ultrasonicDistance\": " + String(ultrasonicDistance, 2) + ",";
    payload += "\"laserDistance\": " + String(laserDistance, 2) + ",";
    payload += "\"radarDigitalStatus\": \"" + radarDigitalStatus + "\","; // Digital: "Motion Detected" or "No Motion"
    payload += "\"radarAnalogValue\": " + String(radarAnalogValue);        // Analog: Integer reading
    payload += "\"motionDetected\": " + String(pirMotionDetected) + ",";
    payload += "}";

    // Publish data to ThingsBoard
    Serial.println("Publishing payload:");
Serial.println(payload);
client.publish("v1/devices/me/telemetry", payload.c_str());

    delay(250); // Wait for 1 second before the next reading
}
