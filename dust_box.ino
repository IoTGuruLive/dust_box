#include <IoTGuru.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
#endif

#ifdef ESP32
  #include <WiFi.h>
#endif

const char* ssid      = "ssid";
const char* password  = "password";

WiFiClient client;

String userShortId    = "uuuuuuuuuuuuuuuuuuuuuu";
String deviceShortId  = "dddddddddddddddddddddd";
String deviceKey      = "kkkkkkkkkkkkkkkkkkkkkk";
IoTGuru iotGuru = IoTGuru(userShortId, deviceShortId, deviceKey);

String nodeShortId    = "nnnnnnnnnnnnnnnnnnnnnn";
const char* ota_version = "sds011-mqtt-1.0.0";

volatile int PIN_SDS_RX  = 4;
volatile int PIN_SDS_TX  = 5;
volatile int PIN_SDA     = 0;
volatile int PIN_SCL     = 2;
volatile int PIN_POWER   = 14;

#include "SdsDustSensor.h"
SdsDustSensor sds(PIN_SDS_RX, PIN_SDS_TX);

union unionForm {
  byte inBytes[2];
  unsigned int inInt;
} sdsDeviceId;

#include <BME280_MOD-1022.h>
#include <Wire.h>

volatile double temp;
volatile double humidity;
volatile double pressure;

void setup() {
    Serial.begin(115200);
    delay(10);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(50);
        Serial.print(".");
    }
    Serial.println("");

    /**
     * Set the callback function.
     */
    iotGuru.setCallback(&callback);
    /**
     * Set check in duration.
     */
    iotGuru.setCheckDuration(60000);
    /**
     * Set the debug printer (optional).
     */
    iotGuru.setDebugPrinter(&Serial);
    /**
     * Set the network client.
     */
    iotGuru.setNetworkClient(&client);
    /**
     * Check new firmware and try to update during the clean boot.
     */
    iotGuru.firmwareUpdate(ota_version);

    sds.begin();
    delay(10);
    sds.wakeup();
    delay(10);
    Serial.println(sds.queryFirmwareVersion().toString());
    Serial.println(sds.setQueryReportingMode().toString());
}

volatile unsigned long nextWakeup = 0;
volatile unsigned long nextSend = 0;
volatile unsigned long readAttempt = 12;

void loop() {
    if (iotGuru.check(ota_version)) {
        ESP.restart();
    }

    iotGuru.loop();

    if (nextWakeup < millis()) {
        Serial.println("Turn on the sensor");
        sds.wakeup();
        nextWakeup = millis() + 120000;
        nextSend = millis() + 30000;
        readAttempt = 12;
    }

    if (nextSend < millis()) {
        if (readAttempt == 0) {
            nextSend = nextWakeup + 30000;
            readAttempt = 12;
            Serial.println("Turn off the sensor");
            sds.sleep();
            return;
        }

        nextSend = millis() + 2500;
        readAttempt--;

        PmResult pm = sds.queryPm();
        if (pm.isOk()) {

            sdsDeviceId.inBytes[0] = pm.deviceId()[0];
            sdsDeviceId.inBytes[1] = pm.deviceId()[1];
            Serial.println("DeviceId: " + String(sdsDeviceId.inInt) + "(" + String(pm.deviceId()[0]) + "/" + String(pm.deviceId()[1]) + ")");

            Serial.print("Raw measurement PM2.5 = ");
            Serial.print(pm.pm25);
            Serial.print(", PM10 = ");
            Serial.println(pm.pm10);

            if (readBME280() == 1) {
                Serial.println("Compensate the dust measurement with the humidity");
                pm.pm25 = pm.pm25 / (1.0 + 0.48756 * pow((humidity / 100.0), 8.60068));
                pm.pm10 = pm.pm10 / (1.0 + 0.81559 * pow((humidity / 100.0), 5.83411));

                Serial.print("Compensated measurement PM2.5 = ");
                Serial.print(pm.pm25);
                Serial.print(", PM10 = ");
                Serial.println(pm.pm10);

                iotGuru.sendMqttValue(nodeShortId, "temperature", temp);
                iotGuru.sendMqttValue(nodeShortId, "humidity", humidity);
                iotGuru.sendMqttValue(nodeShortId, "pressure", pressure);
            } else {
                Serial.println("Could not read values from BME280 sensor, do not compensate");
            }

            iotGuru.sendMqttValue(nodeShortId, "pm25", pm.pm25);
            iotGuru.sendMqttValue(nodeShortId, "pm10", pm.pm10);

            readAttempt = 0;
        } else {
            Serial.print("Could not read values from sensor, reason: ");
            Serial.println(pm.statusToString());
        }
    }
}

void callback(const char* nodeShortId, const char* fieldName, const char* message) {
    Serial.print(nodeShortId);Serial.print(" - ");Serial.print(fieldName);Serial.print(": ");Serial.println(message);
}

int readBME280() {
    digitalWrite(PIN_POWER, HIGH);
    delay(5);

    Wire.begin(PIN_SDA, PIN_SCL);
    uint8_t chipID = BME280.readChipId();

    Serial.print("ChipID = 0x");
    Serial.println(chipID, HEX);

    if (chipID != 0x60) {
        digitalWrite(PIN_POWER, LOW);
        return -1;
    }

    BME280.readCompensationParams();
    BME280.writeOversamplingPressure(os16x);
    BME280.writeOversamplingTemperature(os16x);
    BME280.writeOversamplingHumidity(os16x);

    BME280.writeMode(smForced);

    int result = repeatedBMERead();

    BME280.writeMode(smNormal);

    digitalWrite(PIN_POWER, LOW);

    if (result == 0) {
        Serial.println("Cannot read valid measurements...");
        return 0;
    }

    Serial.println("Temperature  " + String(temp));
    Serial.println("Humidity     " + String(humidity));
    Serial.println("Pressure     " + String(pressure));

    return 1;
}

int repeatedBMERead() {
    for (int i = 0; i < 10; i++) {
        Serial.println("Measuring...");
        while (BME280.isMeasuring()) {
            delay(1);
        }

        noInterrupts();
        BME280.readMeasurements();
        interrupts();

        temp = BME280.getTemperature();
        humidity = BME280.getHumidity();
        pressure = BME280.getPressure();

        BME280.writeStandbyTime(tsb_0p5ms);
        BME280.writeFilterCoefficient(fc_16);

        if (pressure > 800 && pressure < 1200) {
            return 1;
        }

        delay(10);
    }

    return 0;
}
