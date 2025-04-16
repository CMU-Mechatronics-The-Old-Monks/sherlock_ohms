#include "LoadCellSensor.h"
#include <Arduino.h>

LoadCellSensor::LoadCellSensor() : _scale(DOUT_PIN, SCK_PIN), _weight(0.0f) {}

void LoadCellSensor::begin() {
    _scale.begin();
    _scale.start(2000);  // Wait for startup
    _scale.setCalFactor(1000.0);  // Calibration factor – adjust this
    //Serial.println("Load cell initialized (HX711_ADC)");
}

void LoadCellSensor::update() {
    _scale.update();  // Must call repeatedly

    // optionally smooth it out over time:
    _weight = _scale.getData();
}

float LoadCellSensor::getWeight() const {
    return _weight;
}

