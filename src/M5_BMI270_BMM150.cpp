#include "Arduino.h"
#include "Wire.h"
#include "M5_BMI270_BMM150.h"

M5_BMI270_BMM150::M5_BMI270_BMM150(TwoWire* wire) {
    _wire = wire;
}

void M5_BMI270_BMM150::debug(Stream& stream) {
    _debug = &stream;
}

int M5_BMI270_BMM150::begin(uint8_t bmi2_addr, uint8_t bmm150_addr) {
    _bmi2_addr   = bmi2_addr;
    _bmm150_addr = bmm150_addr;

    _wire->begin();

    bmi2.chip_id         = _bmi2_addr;
    bmi2.read            = bmi2_i2c_read;
    bmi2.write           = bmi2_i2c_write;
    bmi2.delay_us        = bmi2_delay_us;
    bmi2.intf            = BMI2_I2C_INTF;
    bmi2.intf_ptr        = &accel_gyro_dev_info;
    bmi2.read_write_len  = 30;    // Limitation of the Wire library
    bmi2.config_file_ptr = NULL;  // Use the default BMI270 config file

    accel_gyro_dev_info._wire    = _wire;
    accel_gyro_dev_info.dev_addr = bmi2.chip_id;

    bmm1.chip_id  = _bmm150_addr;
    bmm1.read     = bmi2_i2c_read;
    bmm1.write    = bmi2_i2c_write;
    bmm1.delay_us = bmi2_delay_us;
    bmm1.intf     = BMM150_I2C_INTF;
    bmm1.intf_ptr = &mag_dev_info;

    mag_dev_info._wire    = _wire;
    mag_dev_info.dev_addr = bmm1.chip_id;

    int8_t rslt = bmi270_init(&bmi2);
    print_rslt(rslt);

    rslt = configure_sensor(&bmi2);
    print_rslt(rslt);

    rslt = bmm150_init(&bmm1);
    print_rslt(rslt);

    rslt = configure_sensor(&bmm1);
    print_rslt(rslt);

    _initialized = true;

    return 1;
}

void M5_BMI270_BMM150::setContinuousMode() {
    bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 1, &bmi2);
    continuousMode = true;
}

void M5_BMI270_BMM150::oneShotMode() {
    bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN, 0, &bmi2);
    continuousMode = false;
}

// default range is +-4G, so conversion factor is (((1 << 15)/4.0f))
#define INT16_to_G (8192.0f)

// Accelerometer
int M5_BMI270_BMM150::readAcceleration(float& x, float& y, float& z) {
    struct bmi2_sens_data sensor_data;
    auto ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
#ifdef TARGET_ARDUINO_NANO33BLE
    x = -sensor_data.acc.y / INT16_to_G;
    y = -sensor_data.acc.x / INT16_to_G;
#else
    x = sensor_data.acc.x / INT16_to_G;
    y = sensor_data.acc.y / INT16_to_G;
#endif
    z = sensor_data.acc.z / INT16_to_G;
    return (ret == 0);
}

int M5_BMI270_BMM150::accelerationAvailable() {
    uint16_t status;
    bmi2_get_int_status(&status, &bmi2);
    int ret     = ((status | _int_status) & BMI2_ACC_DRDY_INT_MASK);
    _int_status = status;
    _int_status &= ~BMI2_ACC_DRDY_INT_MASK;
    return ret;
}

float M5_BMI270_BMM150::accelerationSampleRate() {
    struct bmi2_sens_config sens_cfg;
    sens_cfg.type = BMI2_ACCEL;
    bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
    return (1 << sens_cfg.cfg.acc.odr) * 0.39;
}

// default range is +-2000dps, so conversion factor is (((1 << 15)/4.0f))
#define INT16_to_DPS (16.384f)

// Gyroscope
int M5_BMI270_BMM150::readGyroscope(float& x, float& y, float& z) {
    struct bmi2_sens_data sensor_data;
    auto ret = bmi2_get_sensor_data(&sensor_data, &bmi2);
#ifdef TARGET_ARDUINO_NANO33BLE
    x = -sensor_data.gyr.y / INT16_to_DPS;
    y = -sensor_data.gyr.x / INT16_to_DPS;
#else
    x = sensor_data.gyr.x / INT16_to_DPS;
    y = sensor_data.gyr.y / INT16_to_DPS;
#endif
    z = sensor_data.gyr.z / INT16_to_DPS;
    return (ret == 0);
}

int M5_BMI270_BMM150::gyroscopeAvailable() {
    uint16_t status;
    bmi2_get_int_status(&status, &bmi2);
    int ret     = ((status | _int_status) & BMI2_GYR_DRDY_INT_MASK);
    _int_status = status;
    _int_status &= ~BMI2_GYR_DRDY_INT_MASK;
    return ret;
}

float M5_BMI270_BMM150::gyroscopeSampleRate() {
    struct bmi2_sens_config sens_cfg;
    sens_cfg.type = BMI2_GYRO;
    bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
    return (1 << sens_cfg.cfg.gyr.odr) * 0.39;
}

// Magnetometer
int M5_BMI270_BMM150::readMagneticField(float& x, float& y, float& z) {
    struct bmm150_mag_data mag_data;
    int const rc = bmm150_read_mag_data(&mag_data, &bmm1);
    x            = mag_data.x;
    y            = mag_data.y;
    z            = mag_data.z;

    if (rc == BMM150_OK)
        return 1;
    else
        return 0;
}

int M5_BMI270_BMM150::magneticFieldAvailable() {
    bmm150_get_interrupt_status(&bmm1);
    return bmm1.int_status & BMM150_INT_ASSERTED_DRDY;
}

float M5_BMI270_BMM150::magneticFieldSampleRate() {
    struct bmm150_settings settings;
    bmm150_get_sensor_settings(&settings, &bmm1);
    switch (settings.data_rate) {
        case BMM150_DATA_RATE_10HZ:
            return 10;
        case BMM150_DATA_RATE_02HZ:
            return 2;
        case BMM150_DATA_RATE_06HZ:
            return 6;
        case BMM150_DATA_RATE_08HZ:
            return 8;
        case BMM150_DATA_RATE_15HZ:
            return 15;
        case BMM150_DATA_RATE_20HZ:
            return 20;
        case BMM150_DATA_RATE_25HZ:
            return 25;
        case BMM150_DATA_RATE_30HZ:
            return 30;
    }
    return 0;
}

int8_t M5_BMI270_BMM150::configure_sensor(struct bmi2_dev* dev) {
    int8_t rslt;
    uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};

    struct bmi2_int_pin_config int_pin_cfg;
    int_pin_cfg.pin_type             = BMI2_INT1;
    int_pin_cfg.int_latch            = BMI2_INT_NON_LATCH;
    int_pin_cfg.pin_cfg[0].lvl       = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od        = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE;

    struct bmi2_sens_config sens_cfg[2];
    sens_cfg[0].type                = BMI2_ACCEL;
    sens_cfg[0].cfg.acc.bwp         = BMI2_ACC_OSR2_AVG2;
    sens_cfg[0].cfg.acc.odr         = BMI2_ACC_ODR_100HZ;
    sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[0].cfg.acc.range       = BMI2_ACC_RANGE_4G;
    sens_cfg[1].type                = BMI2_GYRO;
    sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[1].cfg.gyr.bwp         = BMI2_GYR_OSR2_MODE;
    sens_cfg[1].cfg.gyr.odr         = BMI2_GYR_ODR_100HZ;
    sens_cfg[1].cfg.gyr.range       = BMI2_GYR_RANGE_2000;
    sens_cfg[1].cfg.gyr.ois_range   = BMI2_GYR_OIS_2000;

    rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
    if (rslt != BMI2_OK) return rslt;

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
    if (rslt != BMI2_OK) return rslt;

    rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
    if (rslt != BMI2_OK) return rslt;

    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    if (rslt != BMI2_OK) return rslt;

    return rslt;
}

int8_t M5_BMI270_BMM150::configure_sensor(struct bmm150_dev* dev) {
    /* Status of api are returned to this variable. */
    int8_t rslt;
    struct bmm150_settings settings;

    /* Set powermode as normal mode */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt              = bmm150_set_op_mode(&settings, dev);

    if (rslt == BMM150_OK) {
        /* Setting the preset mode as Low power mode
         * i.e. data rate = 10Hz, XY-rep = 1, Z-rep = 2
         */
        settings.preset_mode = BMM150_PRESETMODE_REGULAR;
        // rslt = bmm150_set_presetmode(&settings, dev);

        if (rslt == BMM150_OK) {
            /* Map the data interrupt pin */
            settings.int_settings.drdy_pin_en = 0x01;
            // rslt = bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN,
            // &settings, dev);
        }
    }
    return rslt;
}

int8_t M5_BMI270_BMM150::bmi2_i2c_read(uint8_t reg_addr,
                                             uint8_t* reg_data, uint32_t len,
                                             void* intf_ptr) {
    if ((reg_data == NULL) || (len == 0) || (len > 32)) {
        return -1;
    }
    uint8_t bytes_received;

    struct dev_info* dev_info = (struct dev_info*)intf_ptr;
    uint8_t dev_id            = dev_info->dev_addr;

    dev_info->_wire->beginTransmission(dev_id);
    dev_info->_wire->write(reg_addr);
    if (dev_info->_wire->endTransmission() == 0) {
        bytes_received = dev_info->_wire->requestFrom(dev_id, len);
        // Optionally, throw an error if bytes_received != len
        for (uint16_t i = 0; i < bytes_received; i++) {
            reg_data[i] = dev_info->_wire->read();
        }
    } else {
        return -1;
    }

    return 0;
}

int8_t M5_BMI270_BMM150::bmi2_i2c_write(uint8_t reg_addr,
                                              const uint8_t* reg_data,
                                              uint32_t len, void* intf_ptr) {
    if ((reg_data == NULL) || (len == 0) || (len > 32)) {
        return -1;
    }

    struct dev_info* dev_info = (struct dev_info*)intf_ptr;
    uint8_t dev_id            = dev_info->dev_addr;
    dev_info->_wire->beginTransmission(dev_id);
    dev_info->_wire->write(reg_addr);
    for (uint16_t i = 0; i < len; i++) {
        dev_info->_wire->write(reg_data[i]);
    }
    if (dev_info->_wire->endTransmission() != 0) {
        return -1;
    }

    return 0;
}

void M5_BMI270_BMM150::bmi2_delay_us(uint32_t period, void* intf_ptr) {
    delayMicroseconds(period);
}

void M5_BMI270_BMM150::print_rslt(int8_t rslt) {
    if (!_debug) {
        return;
    }
    switch (rslt) {
        case BMI2_OK:
            return; /* Do nothing */
            break;
        case BMI2_E_NULL_PTR:
            _debug->println("Error [" + String(rslt) + "] : Null pointer");

            break;
        case BMI2_E_COM_FAIL:
            _debug->println("Error [" + String(rslt) +
                            "] : Communication failure");

            break;
        case BMI2_E_DEV_NOT_FOUND:
            _debug->println("Error [" + String(rslt) + "] : Device not found");

            break;
        case BMI2_E_OUT_OF_RANGE:
            _debug->println("Error [" + String(rslt) + "] : Out of range");

            break;
        case BMI2_E_ACC_INVALID_CFG:
            _debug->println("Error [" + String(rslt) +
                            "] : Invalid accel configuration");

            break;
        case BMI2_E_GYRO_INVALID_CFG:
            _debug->println("Error [" + String(rslt) +
                            "] : Invalid gyro configuration");

            break;
        case BMI2_E_ACC_GYR_INVALID_CFG:
            _debug->println("Error [" + String(rslt) +
                            "] : Invalid accel/gyro configuration");

            break;
        case BMI2_E_INVALID_SENSOR:
            _debug->println("Error [" + String(rslt) + "] : Invalid sensor");

            break;
        case BMI2_E_CONFIG_LOAD:
            _debug->println("Error [" + String(rslt) +
                            "] : Configuration loading error");

            break;
        case BMI2_E_INVALID_PAGE:
            _debug->println("Error [" + String(rslt) + "] : Invalid page ");

            break;
        case BMI2_E_INVALID_FEAT_BIT:
            _debug->println("Error [" + String(rslt) +
                            "] : Invalid feature bit");

            break;
        case BMI2_E_INVALID_INT_PIN:
            _debug->println("Error [" + String(rslt) +
                            "] : Invalid interrupt pin");

            break;
        case BMI2_E_SET_APS_FAIL:
            _debug->println("Error [" + String(rslt) +
                            "] : Setting advanced power mode failed");

            break;
        case BMI2_E_AUX_INVALID_CFG:
            _debug->println("Error [" + String(rslt) +
                            "] : Invalid auxiliary configuration");

            break;
        case BMI2_E_AUX_BUSY:
            _debug->println("Error [" + String(rslt) + "] : Auxiliary busy");

            break;
        case BMI2_E_SELF_TEST_FAIL:
            _debug->println("Error [" + String(rslt) + "] : Self test failed");

            break;
        case BMI2_E_REMAP_ERROR:
            _debug->println("Error [" + String(rslt) + "] : Remapping error");

            break;
        case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
            _debug->println("Error [" + String(rslt) +
                            "] : Gyro user gain update failed");

            break;
        case BMI2_E_SELF_TEST_NOT_DONE:
            _debug->println("Error [" + String(rslt) +
                            "] : Self test not done");

            break;
        case BMI2_E_INVALID_INPUT:
            _debug->println("Error [" + String(rslt) + "] : Invalid input");

            break;
        case BMI2_E_INVALID_STATUS:
            _debug->println("Error [" + String(rslt) + "] : Invalid status");

            break;
        case BMI2_E_CRT_ERROR:
            _debug->println("Error [" + String(rslt) + "] : CRT error");

            break;
        case BMI2_E_ST_ALREADY_RUNNING:
            _debug->println("Error [" + String(rslt) +
                            "] : Self test already running");

            break;
        case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
            _debug->println("Error [" + String(rslt) +
                            "] : CRT ready for DL fail abort");

            break;
        case BMI2_E_DL_ERROR:
            _debug->println("Error [" + String(rslt) + "] : DL error");

            break;
        case BMI2_E_PRECON_ERROR:
            _debug->println("Error [" + String(rslt) + "] : PRECON error");

            break;
        case BMI2_E_ABORT_ERROR:
            _debug->println("Error [" + String(rslt) + "] : Abort error");

            break;
        case BMI2_E_GYRO_SELF_TEST_ERROR:
            _debug->println("Error [" + String(rslt) +
                            "] : Gyro self test error");

            break;
        case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
            _debug->println("Error [" + String(rslt) +
                            "] : Gyro self test timeout");

            break;
        case BMI2_E_WRITE_CYCLE_ONGOING:
            _debug->println("Error [" + String(rslt) +
                            "] : Write cycle ongoing");

            break;
        case BMI2_E_WRITE_CYCLE_TIMEOUT:
            _debug->println("Error [" + String(rslt) +
                            "] : Write cycle timeout");

            break;
        case BMI2_E_ST_NOT_RUNING:
            _debug->println("Error [" + String(rslt) +
                            "] : Self test not running");

            break;
        case BMI2_E_DATA_RDY_INT_FAILED:
            _debug->println("Error [" + String(rslt) +
                            "] : Data ready interrupt failed");

            break;
        case BMI2_E_INVALID_FOC_POSITION:
            _debug->println("Error [" + String(rslt) +
                            "] : Invalid FOC position");

            break;
        default:
            _debug->println("Error [" + String(rslt) +
                            "] : Unknown error code");

            break;
    }
}
