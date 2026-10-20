# M5Module-GNSS

[![Arduino Lint](https://github.com/m5stack/M5Module-GNSS/actions/workflows/Arduino-Lint-Check.yml/badge.svg)](https://github.com/m5stack/M5Module-GNSS/actions/workflows/Arduino-Lint-Check.yml)
[![Clang Format](https://github.com/m5stack/M5Module-GNSS/actions/workflows/clang-format-check.yml/badge.svg)](https://github.com/m5stack/M5Module-GNSS/actions/workflows/clang-format-check.yml)


## BMI270 I2C access (Arduino core 3.2+ / ESP-IDF 5.4+)

`BMI270` now talks I2C through the Arduino `TwoWire` (`Wire`) API instead of calling the
ESP-IDF legacy `driver/i2c.h` functions directly. Since Arduino core 3.2.0, `Wire` is built on
the new `i2c_master` driver (`driver_ng`), and linking the legacy driver into the same firmware
aborts at startup with:

```
E (1057) i2c: CONFLICT! driver_ng is not allowed to be used with this old driver
abort() was called at PC 0x4204ef83 on core 0
```

Since `BMI270` no longer references the legacy driver, this conflict can no longer occur. Arduino
core 2.x and 3.0/3.1 keep working as before.

`BMI270::init()` now takes a `TwoWire&`:

```cpp
Wire.begin(21, 22, 100000); // the library does not call Wire.begin() itself
bmi270.init(Wire, BIM270_SENSOR_ADDR);
```

The old `init(i2c_port_t, uint8_t)` signature is kept for source compatibility
(`I2C_NUM_0` maps to `Wire`, `I2C_NUM_1` maps to `Wire1` on chips with a second I2C controller
such as ESP32 / ESP32-S3) but is deprecated — prefer the `TwoWire&` overload in new code.


## License

- [M5Module-GNSS - MIT](LICENSE)

