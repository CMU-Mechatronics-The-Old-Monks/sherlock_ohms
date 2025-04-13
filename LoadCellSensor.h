#ifndef LOADCELLSENSOR_H
#define LOADCELLSENSOR_H

#include <HX711_ADC.h>

class LoadCellSensor {
public:
    LoadCellSensor();  // default pin setup
    void begin();
    void update();
    float getWeight() const;

private:
    static constexpr int DOUT_PIN = 37;
    static constexpr int SCK_PIN = 36;
    float _weight;
    HX711_ADC _scale;
};

#endif

