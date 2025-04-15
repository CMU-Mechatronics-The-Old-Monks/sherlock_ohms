#include "VSensor.h"
#include <Arduino.h>
#include <math.h>

VSensor::VSensor(float vRef, int pin, int samples)
    : referenceVoltage(vRef), analogPin(pin), sampleCount(samples), latestRMSVoltage(0.0) {}

void VSensor::begin() {
    pinMode(analogPin, INPUT);
}

void VSensor::update() {
    float sensorOffset = referenceVoltage / 2.0;
    float sumSquares = 0.0;

    for (int i = 0; i < sampleCount; i++) {
        int raw = analogRead(analogPin);
        float voltage = (raw / 1023.0) * referenceVoltage;
        float centered = voltage - sensorOffset;
        sumSquares += centered * centered;
        delayMicroseconds(200);
    }

    float meanSquares = sumSquares / sampleCount;
    latestRMSVoltage = sqrt(meanSquares);
}

float VSensor::getVoltage() const {
    return latestRMSVoltage;
}

void VSensor::setCalibration(float baseline, float scaleFactor) {
    baselineOffset = baseline;
    calibrationScale = scaleFactor;
}

float VSensor::getTrueVoltage() const {
    return (latestRMSVoltage - baselineOffset) * calibrationScale;
}

float VSensor::getNormalizedVoltage(float expectedMaxRMS, float outputRange) const {
    float delta = fabs(latestRMSVoltage - baselineOffset);  // Absolute difference
    float maxDelta = fabs(expectedMaxRMS - baselineOffset); // Expected max signal swing

    return constrain((delta / maxDelta) * outputRange, 0.0f, outputRange);
}


