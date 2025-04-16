#include "IMU.h"

IMU::IMU() : bno(55), vx(0), vy(0), lastUpdateTime(0) {}

bool IMU::begin() {
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055!");
        return false;
    }
    bno.setExtCrystalUse(true);
    lastUpdateTime = millis();
    return true;
}

void IMU::update() {
    unsigned long now = millis();
    float dt = (now - lastUpdateTime) / 1000.0f;  // convert to seconds
    lastUpdateTime = now;

    linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    eulerAngles = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    // Trapezoidal integration (assumes constant accel between updates)
    vx += linearAccel.x() * dt;
    vy += linearAccel.y() * dt;

    // Optional drift handling: stop integration if acceleration is negligible
    if (fabs(linearAccel.x()) < 0.1) vx = 0;
    if (fabs(linearAccel.y()) < 0.1) vy = 0;
}

float IMU::getVx() {
    return vx;
}

float IMU::getVy() {
    return vy;
}

float IMU::getYaw() {
    return eulerAngles.z();
}

