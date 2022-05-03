#define VERSION_NUM "0.1"
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
// #include <Adafruit_LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

// CONFIG - START

#define LED1_PIN 19
#define LED2_PIN 6
#define LED3_PIN 9

#define VOLTAGE_PIN 18
#define R1_OHM 2000.0F
#define R2_OHM 1250.0F

#define xbee Serial2
#define gpsSerial Serial3

// CONFIG - END

#define SEALEVELPRESSURE_HPA 1013.25
#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

TinyGPSPlus gps;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BME280 bme;
LiquidCrystal_I2C lcd(0x27, 20, 2);

float gyro_r = 0;
float gyro_p = 0;
float gyro_y = 0;
float accel_r = 0;
float accel_p = 0;
float accel_y = 0;
float mag_r = 0;
float mag_p = 0;
float mag_y = 0;
float temp = 0;
float altitude = 0;
float voltage = 0;

char fileName[100];

void blink(int count, unsigned int delay_ms = 100) {
    for (int i = 0; i < count; i++) {
        digitalWrite(LED1_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(LED1_PIN, LOW);
        delay(delay_ms);
    }
}

void setup() {
    Serial.begin(115200);
    xbee.begin(115200);
    gpsSerial.begin(115200);
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);
    pinMode(VOLTAGE_PIN, INPUT);

    Serial.println("Initilizing LCD...");
    lcd.init();        // initialize the lcd
    lcd.begin(20, 4);  // set the size of the display
    // Print a message to the LCD.
    lcd.backlight();
    lcd.setCursor(3, 0);
    lcd.print("GS Board v" + String(VERSION_NUM));
    lcd.setCursor(0, 2);
    lcd.print("Starting...");

    bool bmeSuccess = true, bnoSuccess = true, sdSuccess = true;
    if (bme.begin(0x76))
        Serial.println("✔ SUCCEED: BME280");
    else {
        Serial.println("[FAILED] Unable to set up BME280!");
        bmeSuccess = false;
        blink(1);
        delay(500);
    }
    if (bno.begin())
        Serial.println("✔ SUCCEED: BNO055");
    else {
        Serial.println("[FAILED] Unable to set up BNO055!");
        bnoSuccess = false;
        blink(2);
        delay(500);
    }
    bno.setExtCrystalUse(true);

    if (SD.begin(10))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        sdSuccess = false;
        blink(5);
    }

    if (!bmeSuccess || !bnoSuccess || !sdSuccess) {
        lcd.setCursor(0, 2);
        lcd.print("Errors:");
        if (!bmeSuccess) lcd.print(" BME");
        if (!bnoSuccess) lcd.print(" BNO");
        if (!sdSuccess) lcd.print(" SD");
        delay(4000);
    } else {
        lcd.setCursor(0, 2);
        lcd.print("Succeed! No errors.");
        delay(1000);
    }

    lcd.clear();

    int fileIndex = 0;
    do {
        fileIndex++;
        String("GS_" + String(fileIndex) + ".txt").toCharArray(fileName, 100);
    } while (SD.exists(fileName));
    Serial.print("Selected file name: ");
    Serial.println(fileName);
    blink(3);
}

void getBMEData() {
    temp = bme.readTemperature();
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

void getBNOData() {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    gyro_r = gyro.x();
    gyro_p = gyro.y();
    gyro_y = gyro.z();
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    accel_r = accel.x();
    accel_p = accel.y();
    accel_y = accel.z();
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    mag_r = mag.x();
    mag_p = mag.y();
    mag_y = mag.z();
}

float latitude, longitude, gpsAltitude;
int gpsSatellite;
char gpsTime[32] = "xx:xx:xx";

void getGPS() {
    latitude = gps.location.lat();
    longitude = gps.location.lng();

    sprintf(gpsTime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    gpsAltitude = gps.altitude.meters();
    gpsSatellite = gps.satellites.value();
}

void getBattery() {
    float apparentVoltage = analogRead(VOLTAGE_PIN) * 3.3 / 1023.0;
    voltage = apparentVoltage * (R1_OHM + R2_OHM) / R2_OHM;
    if (voltage < 5.3) {
        blink(10, 25);
    }
}

// byte batteryIcon[] = {
//     B00000,
//     B01110,
//     B11111,
//     B10001,
//     B11111,
//     B11111,
//     B11111,
//     B00000};
byte batteryIcon[] = {
    B00000,
    B01110,
    B11111,
    B10001,
    B10001,
    B10001,
    B11111,
    B00000};

unsigned long lastSendSelf = 0, lastStatus = 0, count = 0;
bool didResetStatus = true;
void loop() {
    while (gpsSerial.available())
        gps.encode(gpsSerial.read());
    getBMEData();
    getBNOData();
    getGPS();
    if (Serial.available()) {
        String command = Serial.readStringUntil('\r');
        lcd.setCursor(9, 3);
        lcd.print("Sending cmd");
        lastStatus = millis();
        didResetStatus = false;
        xbee.print(command + "\r");
    }
    if (xbee.available()) {
        String telemetry = xbee.readStringUntil('\r');
        Serial.print(telemetry + "\r");
        lcd.setCursor(0, 0);
        lcd.print("PKG #");
        lcd.print(++count);
        File file = SD.open(fileName, FILE_WRITE);
        if (file) {
            file.println(telemetry);
            file.close();
        }
    }
    if (millis() - lastSendSelf > 1000) {
        lastSendSelf = millis();
        // Serial.print(String(altitude) + ',' +
        //              temp + ',' + String(voltage) + ',' + gyro_r + ',' + gyro_p + ',' + gyro_y +
        //              ',' + accel_r + ',' + accel_p + ',' + accel_y + ',' + mag_r + ',' +
        //              mag_p + ',' + mag_y + ',' + latitude + ',' + longitude + ',' + gpsAltitude + '\r');

        lcd.createChar(0, batteryIcon);
        lcd.setCursor(0, 3);
        lcd.write(0);
        lcd.setCursor(2, 3);
        lcd.print(voltage);
        lcd.print("V");
    }
    if (!didResetStatus && millis() - lastStatus > 1000) {
        didResetStatus = true;
        lcd.setCursor(7, 3);
        lcd.print("             ");
    }
}