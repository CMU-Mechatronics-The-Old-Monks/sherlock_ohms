#ifndef VL53L1XSENSOR_H
#define VL53L1XSENSOR_H

#include <Wire.h>
#include <VL53L1X.h>

class VL53L1XSensor {
public:
    VL53L1XSensor(uint8_t i2c_addr = 0x29);  // Default I2C address
    void begin();
    void update();
    uint16_t getDistance() const;

private:
    VL53L1X sensor;
    uint8_t address;
    uint16_t distance_mm;
};

#endif
