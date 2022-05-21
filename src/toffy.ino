#include <SD.h>
#include <TimeLib.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

#define LED2_PIN 2
#define LED1_PIN 3
#define BUZZER_PIN 4
#define VOLTAGE_PIN 20

#define R1_OHM 2000
#define R2_OHM 1250

#define gpsSerial Serial3
#define lora Serial2

TinyGPSPlus gps;

String lastCmd = "N/A";

char fileName[100];

void blink(const unsigned int& count, const unsigned int& delay_ms = 100) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(LED1_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(LED1_PIN, LOW);
        delay(delay_ms);
    }
}

void beep(const unsigned int& count, const unsigned int& delay_ms = 100) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(BUZZER_PIN, LOW);
        delay(delay_ms);
    }
}

void blinkbeep(const unsigned int& count, const unsigned int& delay_ms = 100) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(LED1_PIN, HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(LED1_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
        delay(delay_ms);
    }
}

struct Packet {
    char time[9] = "xx:xx:xx";
    char gpsTime[9] = "xx:xx:xx";
    float gpsLat;
    float gpsLng;
    float gpsAlt;
    float voltage;
    int gpsSat;

    String combine() {
        return String(gpsLat, 6) + " " + String(gpsLng, 6) + " " + String(gpsAlt) + " " + String(gpsSat) + "$";
    }
}(packet);

time_t getTeensy3Time() {
    return Teensy3Clock.get();
}

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600);
    lora.begin(9600);

    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(VOLTAGE_PIN, INPUT);

    if (SD.begin(10))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("❌ FAILED - SD card reader initialization failed!");
        blinkbeep(5);
    }

    setSyncProvider(getTeensy3Time);

    int fileIndex = 0;
    do {
        fileIndex++;
        String("R_" + String(fileIndex) + ".txt").toCharArray(fileName, 100);
    } while (SD.exists(fileName));
    Serial.print("Selected file name: ");
    Serial.println(fileName);
    delay(1000);
    blinkbeep(3);
    Serial.println("gpsTime, gpsLat, gpsLng, gpsAlt, gpsSat, state");
}

void getGPSData() {
    packet.gpsLat = gps.location.lat();
    packet.gpsLng = gps.location.lng();
    sprintf(packet.gpsTime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    packet.gpsAlt = gps.altitude.meters();
    packet.gpsSat = gps.satellites.value();
}

void getBattery() {
    float apparentVoltage = analogRead(VOLTAGE_PIN) * 3.3 / 1023.0;
    packet.voltage = apparentVoltage * (R1_OHM + R2_OHM) / R2_OHM;
}

uint32_t lastExecuted = 0, lastSent = 0, lastKradik = 0;
bool kradikBool = false;
void loop() {
    while (gpsSerial.available())
        gps.encode(gpsSerial.read());

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        if (cmd == "PARAON") {
        } else if (cmd == "PARAOFF") {
        }
    }

    if (lora.available()) {
        String cmd = lora.readStringUntil('$');
        lastCmd = cmd;
        lora.print("Got: " + cmd + "$");
    }

    if (millis() - lastExecuted > 250) {
        lastExecuted = millis();
        sprintf(packet.time, "%02d:%02d:%02d", hour(), minute(), second());
        getBattery();
        getGPSData();

        String str = packet.combine();
        Serial.println(str);

        File file = SD.open(fileName, FILE_WRITE);
        if (file) {
            file.println(str);
            file.close();
        }
    }
    if (millis() - lastSent > 2000) {
        lastSent = millis();
        lora.println(packet.combine());
    }
}