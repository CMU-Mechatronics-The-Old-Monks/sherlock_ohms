#include "VoltageSensor.h"
#include <Arduino.h>

VoltageSensor::VoltageSensor(float vRef, float dividerRatio)
    : referenceVoltage(vRef), voltageDividerRatio(dividerRatio) {}

void VoltageSensor::begin() {
    // Optional init
}

float VoltageSensor::readVoltage() {
    int analogValue = analogRead(analogPin);
    float voltage = (analogValue * referenceVoltage / 1023.0) * voltageDividerRatio;
    return voltage;
}
