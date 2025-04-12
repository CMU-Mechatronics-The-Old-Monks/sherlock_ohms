#include "LoadCellSensor.h"
#include <Arduino.h>

#define DOUT 26
#define CLK 27

LoadCellSensor::LoadCellSensor(float calibrationFactor)
    : calibrationFactor(calibrationFactor), weight(0.0f) {}

void LoadCellSensor::begin() {
    scale.begin(DOUT, CLK);
    scale.set_scale(calibrationFactor);
    scale.tare();
}

void LoadCellSensor::update() {
    if (scale.is_ready()) {
        weight = scale.get_units();
    } else {
        Serial.println("HX711 not found.");
    }
}

float LoadCellSensor::getWeight() const {
    return weight;
}
