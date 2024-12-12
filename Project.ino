// Sensor pins
#define RD03_DIGITAL_PIN 18
#define RD03_ANALOG_PIN 19
#define ULTRASONIC_TRIG 12
#define ULTRASONIC_ECHO 14
#define LASERPING_PIN 13
#define PIR_PIN 16

// Thresholds
#define CAR_DISTANCE_THRESHOLD 5.0     // Meters for detecting cars
#define HUMAN_DISTANCE_MIN 5.0         // Minimum distance for human detection
#define HUMAN_DISTANCE_MAX 10.0        // Maximum distance for human detection

// Function to read Ultrasonic Sensor
float readUltrasonic() {
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    float duration = pulseIn(ULTRASONIC_ECHO, HIGH, 60000); // 60ms timeout
    if (duration == 0) {
        return -1; // No echo detected
    }
    float distance = duration * 0.034 / 2;
    return (distance > 400 || distance < 0) ? -1 : distance; // Filter invalid readings
}

// Function to read LaserPing Sensor
float readLaserPing() {
    int rawValue = analogRead(LASERPING_PIN);  // Read raw ADC value
    float voltage = rawValue * (3.3 / 4095.0);  // Convert to voltage (ESP32)

    if (voltage < 0.2) {
        return 0.0;  // Too close or out of range
    } else if (voltage > 2.7) {
        return 200.0;  // Max range clamp
    }
    return (voltage - 0.2) * 100.0;  // Convert voltage to distance
}

// Function to read PIR Motion Sensor
int readPIR() {
    return digitalRead(PIR_PIN) == HIGH ? 1 : 0;
}

// Function to read RD03 Radar Digital Pin
int readRD03Digital() {
    return digitalRead(RD03_DIGITAL_PIN);
}

// Function to read RD03 Radar Analog Pin
int readRadarAnalog() {
    return analogRead(RD03_ANALOG_PIN);
}

void setup() {
    Serial.begin(115200);

    // Initialize pins
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    pinMode(LASERPING_PIN, INPUT);
    pinMode(PIR_PIN, INPUT);
    pinMode(RD03_DIGITAL_PIN, INPUT);
    pinMode(RD03_ANALOG_PIN, INPUT);
}

void loop() {
    // Read sensor values
    float ultrasonicDistance = readUltrasonic();
    float laserDistance = readLaserPing();
    int radarDigitalStatus = readRD03Digital();
    int pirMotionDetected = readPIR();
    int radarAnalogValue = readRadarAnalog();

    // Print sensor readings, comma-separated without extra characters
    Serial.print(ultrasonicDistance);  // Ultrasonic Distance
    Serial.print(",");
    Serial.print(laserDistance);       // Laser Distance
    Serial.print(",");
    Serial.print(radarDigitalStatus);  // Radar Digital Status
    Serial.print(",");
    Serial.print(radarAnalogValue);    // Radar Analog Value
    Serial.print(",");
    Serial.println(pirMotionDetected); // PIR Motion Detected

    delay(60); // Short delay to avoid sensor overlap
}
