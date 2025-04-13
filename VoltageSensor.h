#ifndef VOLTAGESENSOR_H
#define VOLTAGESENSOR_H

class VoltageSensor {
public:
    VoltageSensor(float vRef = 5.0, float dividerRatio = 5.0);
    void begin();
    float readVoltage();

private:
    const int analogPin = 22;
    float referenceVoltage;
    float voltageDividerRatio;
};

#endif
