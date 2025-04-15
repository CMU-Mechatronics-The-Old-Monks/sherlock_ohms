#ifndef VSENSOR_H
#define VSENSOR_H

#include <Arduino.h>

class VSensor {
public:
    VSensor(float vRef = 5.0, int pin = A0, int samples = 200);
    void begin();
    void update();
    float getVoltage() const;        // Returns scaled RMS
    float getTrueVoltage() const;    // Returns estimated real-world AC voltage
    void setCalibration(float baseline, float scaleFactor); // Set calibration
    float getNormalizedVoltage(float expectedMaxRMS = 2.25, float outputRange = 5.0) const;


private:
    float referenceVoltage;
    int analogPin;
    int sampleCount;
    float latestRMSVoltage;

    float baselineOffset = 0.87;     // Voltage reading when input is 0V
    float calibrationScale = 100.0;  // Scale factor to convert scaled RMS to real RMS
};

#endif
