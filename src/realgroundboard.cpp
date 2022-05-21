#define VERSION_NUM "0.1"
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
// #include <Adafruit_LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

#include <vector>

#include "LiquidCrystal_I2C.h"

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
float alt = 0;
float voltage = 0;

float c_lat, c_lon, c_alt;

char fileName[100];

class GNSS {
    float lat_g, lng_g, alt_g;
    float lat_c, lng_c, alt_c;
    float x;
    float y;
    float z;

    float dlon, dlat, alt;

   public:
    GNSS(float lat_g, float lng_g, float alt_g, float lat_c, float lng_c, float alt_c, float x, float y, float z) {
        updateCoordinates(lat_g, lng_g, alt_g, lat_c, lng_c, alt_c, x, y, z);
    }

    void updateCoordinates(float lat_g, float lng_g, float alt_g, float lat_c, float lng_c, float alt_c, float x, float y, float z) {
        this->lat_g = radians(lat_g);
        this->lat_c = radians(lat_c);
        this->lng_g = radians(lng_g);
        this->lng_c = radians(lng_c);
        this->dlon = this->lng_c - this->lng_g;
        this->dlat = this->lat_c - this->lat_g;

        this->alt_g = alt_g;
        this->alt_c = alt_c;
        this->alt = this->alt_c - this->alt_g;

        this->x = x;
        this->y = y;
        this->z = z;
    }

    // return arc of the 2 given point
    float arc() {
        float a = pow(sin(dlat / 2), 2);
        a += (cos(lat_g) * cos(lat_c) * (pow((sin(dlon / 2)), 2)));
        float x = sqrt(a);
        float y = sqrt(1 - a);
        float c = 2 * atan2(x, y);
        return c;
    }

    // return ground distance in meters
    float ground_distance() {
        int R0 = 6371000;
        return R0 * arc();
    }

    // return line of sight distance in meters
    float line_of_sight() {
        int R0 = 6371000;
        float R = R0 + alt_g;
        float baselength = 2 * R * cos((PI - arc()) / 2);
        float heightlength = alt;

        float los = pow(baselength, 2) + pow(heightlength, 2) - 2 * baselength * heightlength * cos((PI + arc()) / 2);
        los = abs(los);
        los = sqrt(los);

        return los;
    }

    // return azimuth in degree for locating cansat
    float azimuth() {
        float a = sin(dlon) * cos(lat_c);
        float b = cos(lat_g) * sin(lat_c) - sin(lat_g) * cos(lat_c) * cos(dlon);
        float theta = atan2(a, b);
        theta = degrees(theta);
        return theta;
    }

    // return heading in degree for locating cansat
    float heading() {
        float ht = azimuth() - z;
        ht = int(ht) % 360 + (int(ht * 1000) % 1000) / 1000;
        return ht;
    }

    // return elevation in degree for locating cansat
    float elevation() {
        double elev = atan2(alt, ground_distance());
        elev = degrees(elev);
        elev = elev - x;
        elev = int(elev) % 360 + (int(elev * 1000) % 1000) / 1000;
        return elev;
    }

    float roll() {
        float ty = y;
        ty = int(ty) % 360 + (int(ty * 1000) % 1000) / 1000;
        return ty;
    }

    // find distance of the remaining pythagorus
    float pythagorus() {
        float pyx = atan2((lng_c - lng_g), (lat_c - lat_g));
        return pyx * 180 / PI;
    }
};

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
    alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
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

float lat, lng, gpsAltitude;
int gpsSatellite;
char gpsTime[32] = "xx:xx:xx";

void getGPS() {
    lat = gps.location.lat();
    lng = gps.location.lng();

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

int separate(
    String str,
    char **p,
    int size) {
    int n;
    char s[100];

    strcpy(s, str.c_str());

    *p++ = strtok(s, " ");
    for (n = 1; NULL != (*p++ = strtok(NULL, " ")); n++)
        if (size == n)
            break;

    return n;
}

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }

    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

std::vector<String> split(String str, char separator) {
    std::vector<String> internal;
    String val = "";
    for (char c : str) {
        if (c == separator) {
            internal.push_back(val);
            val = "";
        } else {
            val += c;
        }
    }
    if (val != "") {
        internal.push_back(val);
    }
    return internal;
}

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

        //  TEAM_ID, MISSION_TIME, PACKET_COUNT, PACKET_TYPE, MODE, TP_RELEASED, ALTITUDE, TEMP, VOLTAGE, GPS_TIME, GPS_LATITUDE, GPS_LONGITUDE, GPS_ALTITUDE, GPS_SATS, SOFTWARE_STATE, CMD_ECHO,

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