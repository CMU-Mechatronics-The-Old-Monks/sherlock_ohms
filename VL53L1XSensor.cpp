#include "VL53L1XSensor.h"
#include <Arduino.h>

VL53L1XSensor::VL53L1XSensor(uint8_t i2c_addr) : address(i2c_addr), distance_mm(0) {}

void VL53L1XSensor::begin() {
    Wire.begin();

    sensor.setTimeout(500);
    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize VL53L1X sensor!");
        while (1);  // Halt
    }

    sensor.setDistanceMode(VL53L1X::Long);  // Good for long-range 400cm max
    sensor.setMeasurementTimingBudget(50000);  // Optional: 50ms timing budget
    sensor.startContinuous(50);  // Continuous reading every 50ms

    Serial.println("VL53L1X sensor initialized.");
}

void VL53L1XSensor::update() {
    distance_mm = sensor.read();
}

uint16_t VL53L1XSensor::getDistance() const {
    return distance_mm;
}
