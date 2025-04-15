#include "tof.h"
#include <Arduino.h>

tof::tof(uint8_t i2c_addr) : address(i2c_addr), distance_mm(0) {}

void tof::begin() {
    Wire.begin();
    sensor.setTimeout(500);
    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize VL53L1X sensor!");
        while (1);
    }
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(50);
    Serial.println("VL53L1X sensor initialized.");
}

void tof::update() {
    distance_mm = sensor.read();
}

uint16_t tof::getDistance() const {
    return distance_mm;
}

