#ifndef LOADCELLSENSOR_H
#define LOADCELLSENSOR_H

#include "HX711.h"

class LoadCellSensor {
public:
    LoadCellSensor(float calibrationFactor = 206700.0);
    void begin();
    void update();
    float getWeight() const;

private:
    HX711 scale;
    float calibrationFactor;
    float weight;
};

#endif

