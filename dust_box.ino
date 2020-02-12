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

String sdsNodeShortId    = "ssssssssssssssssssssss";
String bmeNodeShortId    = "bbbbbbbbbbbbbbbbbbbbbb";
const char* ota_version = "sds011-bme680-mqtt-1.0.0";

volatile int PIN_SDA     = 4;
volatile int PIN_SCL     = 5;
volatile int PIN_SDS_RX  = 0;
volatile int PIN_SDS_TX  = 2;

#include "SdsDustSensor.h"
SdsDustSensor sds(PIN_SDS_RX, PIN_SDS_TX);

union unionForm {
  byte inBytes[2];
  unsigned int inInt;
} sdsDeviceId;

#define BME_SCK 14
#define BME_MOSI 12
#define BME_MISO 5
#define BME_CS 4

#include "Adafruit_BME680.h"
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

volatile double temp;
volatile double humidity;
volatile double pressure;
volatile double gas;

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

    if (bme.begin())
    {
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150);
    } else {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
    }
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

            if (bme.performReading()) {
                temp = bme.temperature / 1.0;
                humidity = bme.humidity / 1.0;
                pressure = bme.pressure / 100.0;
                gas = bme.gas_resistance / 1000.0;

                Serial.print("Environmental measurement temperature = ");
                Serial.print(temp);
                Serial.print(", humidity = ");
                Serial.print(humidity);
                Serial.print(", pressure = ");
                Serial.print(pressure);
                Serial.print(", gas = ");
                Serial.println(gas);

                Serial.println("Compensate the dust measurement with the humidity");
                pm.pm25 = pm.pm25 / (1.0 + 0.48756 * pow((humidity / 100.0), 8.60068));
                pm.pm10 = pm.pm10 / (1.0 + 0.81559 * pow((humidity / 100.0), 5.83411));

                Serial.print("Compensated measurement PM2.5 = ");
                Serial.print(pm.pm25);
                Serial.print(", PM10 = ");
                Serial.println(pm.pm10);

                iotGuru.sendMqttValue(bmeNodeShortId, "temperature", temp);
                iotGuru.sendMqttValue(bmeNodeShortId, "humidity", humidity);
                iotGuru.sendMqttValue(bmeNodeShortId, "pressure", pressure);
                if (millis() > 900000) {
                    iotGuru.sendMqttValue(bmeNodeShortId, "gas", gas);
                } else {
                    Serial.println("Skipping VOC gas sensor sending after cold boot (15 mins)");
                }
            } else {
                Serial.println("Could not read values from BME280 sensor, do not compensate");
            }

            iotGuru.sendMqttValue(sdsNodeShortId, "pm25", pm.pm25);
            iotGuru.sendMqttValue(sdsNodeShortId, "pm10", pm.pm10);

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
