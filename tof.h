#ifndef tof_H
#define tof_H

#include <Wire.h>
#include <VL53L1X.h>

class tof {
public:
    tof(uint8_t i2c_addr = 0x29);  // Default I2C address
    void begin();
    void update();
    uint16_t getDistance() const;

private:
    VL53L1X sensor;        // The actual sensor object
    uint8_t address;
    uint16_t distance_mm;
};

#endif

