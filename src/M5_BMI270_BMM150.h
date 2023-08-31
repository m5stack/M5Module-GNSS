#ifndef _M5_BMI270_BMM150_H
#define _M5_BMI270_BMM150_H

#include <Arduino.h>
#include <Wire.h>
#include "utilities/BMI270-Sensor-API/bmi270.h"
#include "utilities/BMM150-Sensor-API/bmm150.h"

struct dev_info {
    TwoWire* _wire;
    uint8_t dev_addr;
};

class M5_BMI270_BMM150 {
   public:
    M5_BMI270_BMM150(TwoWire* wire = &Wire);
    ~M5_BMI270_BMM150() {
    }

    void setContinuousMode();
    void oneShotMode();

    int begin(uint8_t bmi2_addr   = BMI2_I2C_PRIM_ADDR,
              uint8_t bmm150_addr = BMM150_DEFAULT_I2C_ADDRESS);
    void end();

    void debug(Stream&);

    // Accelerometer
    virtual int readAcceleration(
        float& x, float& y, float& z);    // Results are in G (earth gravity).
    virtual int accelerationAvailable();  // Number of samples in the FIFO.
    virtual float accelerationSampleRate();  // Sampling rate of the sensor.

    // Gyroscope
    virtual int readGyroscope(float& x, float& y,
                              float& z);  // Results are in degrees/second.
    virtual int gyroscopeAvailable();     // Number of samples in the FIFO.
    virtual float gyroscopeSampleRate();  // Sampling rate of the sensor.

    // Magnetometer
    virtual int readMagneticField(
        float& x, float& y, float& z);     // Results are in uT (micro Tesla).
    virtual int magneticFieldAvailable();  // Number of samples in the FIFO.
    virtual float magneticFieldSampleRate();  // Sampling rate of the sensor.

   protected:
    // can be modified by subclassing for finer configuration
    virtual int8_t configure_sensor(struct bmm150_dev* dev);
    virtual int8_t configure_sensor(struct bmi2_dev* dev);

   private:
    static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t* reg_data,
                                uint32_t len, void* intf_ptr);
    static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t* reg_data,
                                 uint32_t len, void* intf_ptr);
    static void bmi2_delay_us(uint32_t period, void* intf_ptr);
    void print_rslt(int8_t rslt);

   private:
    TwoWire* _wire;
    Stream* _debug = nullptr;

    bool _initialized = false;
    int _interrupts   = 0;
    struct dev_info accel_gyro_dev_info;
    struct dev_info mag_dev_info;
    struct bmi2_dev bmi2;
    struct bmm150_dev bmm1;
    uint16_t _int_status;

    uint8_t _bmi2_addr;
    uint8_t _bmm150_addr;

   private:
    bool continuousMode;
};

#endif