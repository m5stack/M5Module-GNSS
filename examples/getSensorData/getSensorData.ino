/**
 * @file getSensorData.ino
 * @author SeanKwok (shaoxiang@m5stack.com)
 * @brief M5Module GNSS Get Sensor Data Demo.
 * @version 0.2
 * @date 2023-09-15
 *
 *
 * @Hardwares:M5Module GNSS
 * @Platform Version: Arduino M5Stack Board Manager v2.0.7
 * @Dependent Library:
 * M5Module_GNSS: https://github.com/m5stack/M5Module-GNSS
 * Adafruit BMP280 Library: https://github.com/adafruit/Adafruit_BMP280_Library
 */

#include "M5Module_GNSS.h"
#include <Adafruit_BMP280.h>

#define BIM270_SENSOR_ADDR 0x68
#define BMP280_SENSOR_ADDR 0x76

BMI270::BMI270 bmi270;
Adafruit_BMP280 bmp(&Wire);

void setup() {
    // put your setup code here, to run once:

    Serial.begin(115200);
    Wire.begin(21, 22, 100000);
    while (!Serial)
        ;

    unsigned status = bmp.begin(BMP280_SENSOR_ADDR);
    if (!status) {
        Serial.println(
            F("Could not find a valid BMP280 sensor, check wiring or "
              "try a different address!"));
        Serial.print("SensorID was: 0x");
        Serial.println(bmp.sensorID(), 16);
        while (1) delay(10);
    }

    bmi270.init(I2C_NUM_0, BIM270_SENSOR_ADDR);
}

void loop() {
    // put your main code here, to run repeatedly:
    float x, y, z;

    if (bmi270.accelerationAvailable()) {
        bmi270.readAcceleration(x, y, z);

        Serial.print("accel: \t");
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.print(z);
        Serial.println();
    }

    if (bmi270.gyroscopeAvailable()) {
        bmi270.readGyroscope(x, y, z);

        Serial.print("gyro: \t");
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.print(z);
        Serial.println();
    }

    if (bmi270.magneticFieldAvailable()) {
        int16_t mx, my, mz = 0;
        bmi270.readMagneticField(mx, my, mz);

        Serial.print("mag: \t");
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.print(z);
        Serial.println();
    }

    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
    delay(500);
}
