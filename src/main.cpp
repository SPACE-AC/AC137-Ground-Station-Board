#define VERSION_NUM "1.2"
#include <Adafruit_AHRS.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <math.h>

#include <queue>
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

#define DISPLAY_INTERVAL_MS 500
#define USE_BNO055

// CONFIG - END

#define SEALEVELPRESSURE_HPA 1013.25
#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

#define UPDOWN 2
#define LEFTRIGHT 3
#define UP 4
#define BLANK 12

TinyGPSPlus gps;
#ifdef USE_BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
#else
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_NXPSensorFusion filter;
#endif
Adafruit_BME280 bme;
LiquidCrystal_I2C lcd(0x27, 20, 2);

float roll = 0, pitch = 0, yaw = 0;
float temp = 0;
float alt = 0;
float voltage = 0;

float c_lat = 0, c_lng = 0, c_alt = 0;

char fileName[100];

int mod(int x, int y) {
    return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}

// A class of coordinate consisting of latitude, longitude, altitude
struct Coordinate {
    float latitude = 0;
    float longitude = 0;
    float altitude = 0;

    Coordinate(float latitude = 0, float longitude = 0, float altitude = 0) : latitude(latitude), longitude(longitude), altitude(altitude) {}

    // def __bool__(self):
    //     return self.__len__() > 0

    // def __len__(self):
    //     return len([x for x in [self.latitude, self.longitude, self.altitude] if x])
};

class Gis {
    Coordinate here;
    Coordinate there;
    float gx, gy, gz;
    float lat1, lat2, lon1, lon2;
    float dlon, dlat;
    float alt;

   public:
    /**
     *   A Gis object consisting of coordinates and physical variables
     *   @param here Coordinate of home location
     *   @param there Coordinate of target location
     *   @param gx Orientation X (Roll)
     *   @param gy Orientation Y (Pitch)
     *   @param gz Orientation Z (Yaw)
     */
    Gis(const Coordinate& here, const Coordinate& there, float gx, float gy, float gz) : here(here), there(there), gx(gx), gy(gy), gz(gz) {
        lat1 = radians(here.latitude);

        lat2 = radians(there.latitude);
        lon1 = radians(here.longitude);
        lon2 = radians(there.longitude);
        dlon = lon2 - lon1;
        dlat = lat2 - lat1;

        float a1 = here.altitude;
        float a2 = there.altitude;
        alt = a2 - a1;
        gx = float(gx);
        gy = float(gy);
        gz = float(gz);
    }

    /**
     * @brief Calculate arc angle in unit system : radians
     *
     * @return Arc angle in radians
     */
    float getArcRadians() {
        float a = pow(sin(dlat / 2), 2);
        a += (cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2));
        float y = sqrt(a);
        float x = sqrt(1 - a);
        float c = 2 * atan2(y, x);
        return c;
    }

    /**
     * @brief Calculate arc length in default: meters respect to earth radius
     *
     * @param R0 Radius
     * @return Arc length
     */
    float getArcLength(int R0 = 6371000) {
        float d = R0 * getArcRadians();
        return d;
    }

    /**
     * @brief  Calculate line of sight in default : meters respect to earth radius
     *
     * @param R0 Radius
     * @return Line of sight distance
     */
    float getLineOfSight(float R0 = 6371000) {
        float R = R0 + here.altitude;
        float baselength = 2 * R * cos((PI - getArcRadians()) / 2);
        float heightlength = alt;
        float los = pow(baselength, 2) + pow(heightlength, 2) - 2 * baselength * heightlength * cos((PI + getArcRadians()) / 2);
        los = abs(los);
        los = sqrt(los);

        return los;
    }
    /**
     * @brief Calculate azimuth angle in degrees
     *
     * @return Azimuth angle in degrees
     */
    float getAzimuth() {
        float a = sin(dlon) * cos(lat2);
        float b = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
        float theta = atan2(a, b);
        theta = degrees(theta);
        return mod(theta, 360);
    }

    /**
     * @brief Calculate heading angle in degrees
     *
     * @return Heading angle in degrees
     */
    int getHeading() {
        float ht = getAzimuth() - gz;
        if (ht < -180) ht += 360;
        // ht = ht % 360;
        return round(ht);
    }

    // /**
    //  * @brief Get pitch angle in degrees
    //  *
    //  * @return Pitch angle in degrees
    //  */
    // int getPitch() {
    //     return degrees(asin(getElevationApprox() / getLineOfSight())) - gy;
    // }

    /**
     * @brief Get elevation angle in degrees
     *
     * @param delta Delta pitch in degrees
     *
     * @return Elevation angle in degrees
     */
    float getElevationApprox(bool delta = false) {
        float elev = atan2(alt, getArcLength());
        elev = degrees(elev);
        if (delta) elev = elev - gy;
        return elev;
    }

    /**
     * @brief Get roll angle in degrees
     *
     * @return Roll angle in degrees
     */
    float getRoll() {
        float ty = gx;
        // ty = ty % 360;
        return ty;
    }

    /**
     * @brief Approximate arc angle from Pythagorean equivalence in radians
     *
     * @return Approximated arc angle in radians
     */
    float getArcAngleTotalApprox() {
        float pyx = atan2((lon2 - lon1), (lat2 - lat1));
        return pyx * 180 / PI;
    }
};

float bearing(float lat, float lon, float lat2, float lon2) {
    float teta1 = radians(lat);
    float teta2 = radians(lat2);
    float delta1 = radians(lat2 - lat);
    float delta2 = radians(lon2 - lon);

    //==================Heading Formula Calculation================//

    float y = sin(delta2) * cos(teta2);
    float x = cos(teta1) * sin(teta2) - sin(teta1) * cos(teta2) * cos(delta2);
    float brng = atan2(y, x);
    brng = degrees(brng);  // radians to degrees
    brng = (((int)brng + 360) % 360);

    Serial.print("Heading GPS: ");
    Serial.println(brng);

    return brng;
}

class SMA {
    int size;
    int count;
    std::queue<float> buffer;
    float sum;

   public:
    SMA(int size) {
        this->size = size;
        count = 0;
        sum = 0;
    }

    void add(float value) {
        if (count < size) {
            buffer.push(value);
            sum += value;
            count++;
        } else {
            sum -= buffer.front();
            buffer.pop();
            buffer.push(value);
            sum += value;
        }
    }

    float getAverage() {
        return sum / count;
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

byte batteryIcon[] = {
    B00000,
    B01110,
    B11111,
    B10001,
    B10001,
    B10001,
    B11111,
    B00000};

byte updown[] = {
    B00100,
    B01110,
    B11111,
    B00100,
    B00100,
    B11111,
    B01110,
    B00100};

byte leftright[] = {
    B00000,
    B00000,
    B01010,
    B11111,
    B01010,
    B00000,
    B00000,
    B00000};

byte up[] = {
    B00100,

    B01110,
    B11111,
    B00100,
    B00100,
    B00100,
    B00100,
    B00100};

byte degreeLeft[] = {
    B11100,
    B10100,
    B11100,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000};

byte degreeRight[] = {
    B00111,
    B00101,
    B00111,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000};

byte vertical[] = {
    B00100,
    B00100,
    B00000,
    B00100,
    B00100,
    B00000,
    B00100,
    B00100};

byte blank[] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000};

void setup() {
    Serial.begin(115200);
    xbee.begin(115200);
    gpsSerial.begin(9600);
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);
    pinMode(VOLTAGE_PIN, INPUT);

    Serial.println("Initilizing LCD...");
    // int startTime = millis();
    lcd.init();        // initialize the lcd
    lcd.begin(20, 4);  // set the size of the display
    lcd.backlight();
    lcd.setCursor(3, 0);
    lcd.print("GS Board v" + String(VERSION_NUM));
    lcd.setCursor(6, 1);
    lcd.print("By Omsin");
    lcd.setCursor(0, 3);
    lcd.print("Starting...");
    // lcd.print((millis() - startTime) / 1000.0F);

    bool bmeSuccess = true, bnoSuccess = true, sdSuccess = true;
    if (bme.begin(0x76, &Wire1))
        Serial.println("✔ SUCCEED: BME280");
    else {
        Serial.println("[FAILED] Unable to set up BME280!");
        bmeSuccess = false;
        blink(1);
        // delay(500);
    }
#ifdef USE_BNO055
    if (bno.begin())
        Serial.println("✔ SUCCEED: BNO055");
    else {
        Serial.println("[FAILED] Unable to set up BNO055!");
        bnoSuccess = false;
        blink(2);
        // delay(500);
    }
    bno.setExtCrystalUse(true);
#else
    if (gyro.begin((uint8_t)33U, &Wire1))
        Serial.println("✔ SUCCEED: gyro");
    else {
        Serial.println("[FAILED] Unable to set up gyro!");
        blink(2);
        // delay(500);
    }
    if (accelmag.begin((uint8_t)31U, &Wire1))
        Serial.println("✔ SUCCEED: accelmag");
    else {
        Serial.println("[FAILED] Unable to set up accelmag!");
        blink(2);
        // delay(500);
    }
#endif

    if (SD.begin(10))
        Serial.println("✔ SUCCEED: SD card reader");
    else {
        Serial.println("[FAILED] SD card reader initialization failed!");
        sdSuccess = false;
        blink(5);
    }

    if (!bmeSuccess || !bnoSuccess || !sdSuccess) {
        lcd.setCursor(0, 3);
        lcd.print("Errors:");
        if (!bmeSuccess) lcd.print(" BME");
#ifdef USE_BNO055
        if (!bnoSuccess) lcd.print(" BNO");
#else
        if (!bnoSuccess) lcd.print(" NXP");
#endif
        if (!sdSuccess) lcd.print(" SD");
        delay(2000);
    } else {
        lcd.setCursor(0, 3);
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

    lcd.createChar(0, batteryIcon);
    lcd.createChar(1, degreeLeft);
    lcd.createChar(2, updown);
    lcd.createChar(3, leftright);
    lcd.createChar(4, up);
    lcd.createChar(6, degreeRight);
    lcd.createChar(7, vertical);

    lcd.setCursor(0, 0);
    lcd.print('#');
}

void getBMEData() {
    temp = bme.readTemperature();
    alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

#ifdef USE_BNO055
void getIMUData() {
    sensors_event_t event;
    bno.getEvent(&event);
    roll = event.orientation.x;
    pitch = event.orientation.y;
    yaw = event.orientation.z;
}
#else
void getIMUData() {
    sensors_event_t gyro_event, accel_event, mag_event;
    gyro.getEvent(&gyro_event);
    accelmag.getEvent(&accel_event, &mag_event);

    // gyro_r = gyro_event.gyro.x;
    // gyro_p = gyro_event.gyro.y;
    // gyro_y = gyro_event.gyro.z;
    // accel_r = accel_event.acceleration.x;
    // accel_p = accel_event.acceleration.y;
    // accel_y = accel_event.acceleration.z;
    // mag_r = mag_event.magnetic.x;
    // mag_p = mag_event.magnetic.y;
    // mag_y = mag_event.magnetic.z;
}
#endif

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

float getBattery() {
    float apparentVoltage = analogRead(VOLTAGE_PIN) * 3.3 / 1023.0;
    voltage = apparentVoltage * (R1_OHM + R2_OHM) / R2_OHM;
    return voltage;
    if (voltage < 5.3) {
        blink(10, 25);
    }
}

int separate(
    String str,
    char** p,
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

unsigned long lastSendSelf = 0, lastStatus = 0, lastPacketCount = 0, count = 0;
bool didResetStatus = true;
SMA batterySma(100), tempSma(100), losSma(5);
float lastBattery = 1000, lastTemp = 1000, lastAzimuth = 1000, lastElevation = 1000, lastGpsStatus = 1000, lastDistance = 1000;
void loop() {
    while (gpsSerial.available())
        gps.encode(gpsSerial.read());
    if (Serial.available()) {
        String command = Serial.readStringUntil('\r');
        lcd.setCursor(12, 0);
        lcd.print("Sending");
        lcd.write(UP);
        lastStatus = millis();
        didResetStatus = false;
        xbee.print(command + "\r");
    }
    if (xbee.available()) {
        String telemetry = xbee.readStringUntil('\r');
        Serial.print(telemetry + "\r");

        //  TEAM_ID, MISSION_TIME, PACKET_COUNT, PACKET_TYPE, MODE, TP_RELEASED, ALTITUDE, TEMP, VOLTAGE, GPS_TIME, GPS_LATITUDE, GPS_LONGITUDE, GPS_ALTITUDE, GPS_SATS, SOFTWARE_STATE, CMD_ECHO,
        File file = SD.open(fileName, FILE_WRITE);
        if (file) {
            file.println(telemetry);
            file.flush();
        }
        count++;

        std::vector<String> splitted = split(telemetry, ',');
        if (splitted.size() >= 12 && splitted[3] == "C") {
            c_lat = splitted[10].toFloat();
            c_lng = splitted[11].toFloat();
            c_alt = splitted[6].toFloat();
        }
    }
    if (millis() - lastPacketCount > 1000) {
        lastPacketCount = millis();
        lcd.setCursor(1, 0);
        // lcd.print("#");
        lcd.print(count);
    }
    if (millis() - lastSendSelf > DISPLAY_INTERVAL_MS) {
        lastSendSelf = millis();
        // Serial.print(String(altitude) + ',' +
        //              temp + ',' + String(voltage) + ',' + gyro_r + ',' + gyro_p + ',' + gyro_y +
        //              ',' + accel_r + ',' + accel_p + ',' + accel_y + ',' + mag_r + ',' +
        //              mag_p + ',' + mag_y + ',' + latitude + ',' + longitude + ',' + gpsAltitude + '\r');
        getGPS();
        getBMEData();
        getIMUData();
        tempSma.add(temp);
        batterySma.add(getBattery());

        byte gpsStatus = 0;
        if (c_lat == 0 && c_lng == 0)
            gpsStatus += 1;
        if (lat == 0 && lng == 0)
            gpsStatus += 2;

        unsigned long start = millis();
        if (gpsStatus != lastGpsStatus) {
            if (gpsStatus == 3) {
                lcd.setCursor(0, 1);
                lcd.print(" No remote GPS data");
                lcd.setCursor(0, 2);
                lcd.print("    No GPS data");
            } else if (gpsStatus == 1) {
                lcd.setCursor(0, 1);
                lcd.print(" No remote GPS data");
                lcd.setCursor(0, 2);
                lcd.print("                   ");
            } else if (gpsStatus == 2) {
                lcd.setCursor(0, 1);
                lcd.print("    No GPS data    ");
                lcd.setCursor(0, 2);
                lcd.print("                   ");
            }
        }
        if (gpsStatus == 0) {
            if (lastGpsStatus != 0) {
                // lcd.setCursor(0, 1);
                // lcd.print("                   ");
                lcd.setCursor(0, 1);
                lcd.print("Distance           ");
                lcd.setCursor(0, 2);
                lcd.print("                   ");
                lcd.setCursor(9, 1);
                lcd.write(7);
                lcd.setCursor(9, 2);
                lcd.write(7);
                lcd.setCursor(11, 1);
                lcd.write(LEFTRIGHT);
                lcd.print(' ');
                lcd.setCursor(11, 2);
                lcd.write(UPDOWN);
                lcd.print(' ');
            }
            lastGpsStatus = gpsStatus;
            Gis gis(Coordinate(lat, lng, 0), Coordinate(c_lat, c_lng, c_alt), pitch, -yaw, roll);

            losSma.add(gis.getLineOfSight());
            if (lastDistance != losSma.getAverage()) {
                lcd.setCursor(0, 2);
                lcd.print(String(losSma.getAverage(), 1));
                lcd.print(" m  ");
                lastDistance = losSma.getAverage();
            }
            // lcd.print("Heading: ");
            short heading = round(gis.getAzimuth());
            short elev = round(gis.getElevationApprox(false));
            if (lastAzimuth != heading) {
                lcd.setCursor(13, 1);
                lcd.print(heading);
                lcd.write(6);
                lastAzimuth = heading;
            }
            if (lastElevation != elev) {
                lcd.setCursor(13, 2);
                lcd.print(elev);
                lcd.write(6);
                lcd.print("  ");
                lastElevation = elev;
            }
        }
        lastGpsStatus = gpsStatus;

        if (abs(batterySma.getAverage() - lastBattery) >= 0.009F) {
            lcd.setCursor(0, 3);
            lcd.write(0);
            lcd.print(' ');
            lcd.print(batterySma.getAverage());
            lcd.print("V");
            lastBattery = batterySma.getAverage();
        }
        if (abs(tempSma.getAverage() - lastTemp) >= 0.009F) {
            lcd.setCursor(13, 3);
            lcd.print(tempSma.getAverage());
            lcd.write(6);
            lcd.print("C");
            lastTemp = tempSma.getAverage();
        }
        // Serial.print("Single loop took ");
        // Serial.print(millis() - start);
        // Serial.println(" ms");
    }

    if (!didResetStatus && millis() - lastStatus > 1000) {
        didResetStatus = true;
        lcd.setCursor(12, 0);
        lcd.print("        ");
    }
}