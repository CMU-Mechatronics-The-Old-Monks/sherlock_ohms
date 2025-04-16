#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU {
public:
    IMU();

    bool begin();               // Initialize the IMU
    void update();              // Must be called regularly (e.g. every 10ms)

    float getVx();              // Integrated velocity X (m/s)
    float getVy();              // Integrated velocity Y (m/s)
    float getYaw();             // Orientation in degrees

private:
    Adafruit_BNO055 bno;
    imu::Vector<3> linearAccel;
    imu::Vector<3> eulerAngles;

    float vx, vy;
    unsigned long lastUpdateTime;
};

#endif
