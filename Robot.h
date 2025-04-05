#ifndef ROBOT_H
#define ROBOT_H

#include "Wheel.h"

class Robot {
public:
    Robot(); // Use constructor to initialize pin mappings
    void begin();
    void update(); // Call periodically in loop()

    void setWheelVelocities(float v_fl, float v_fr, float v_rl, float v_rr); // rad/s
    float getWheelAngularVelocity(int index); // 0-3
    float* getWheelAngularVelocities();
    
    void emergencyStopAll();
    void initiateSoftStop(float deceleration = 5.0);
    void updateSoftStop();
    bool isSoftStopping();
    bool isStopped();

private:
    Wheel* _wheels[4]; // 0 = FL, 1 = FR, 2 = RL, 3 = RR

    // Pin decs derived from schematic: https://drive.google.com/drive/u/0/folders/1AQKTDO6EsOuE3jIKUlD8XTELAerbCs_S
    static constexpr uint8_t STEP_EN = 40;
    static constexpr uint8_t Z_STEP = 2;
    static constexpr uint8_t Z_DIR = 35;
    
    static constexpr uint8_t DRIVE_EN = 41;
    static constexpr uint8_t FL_A = 26, FL_B = 27, FL_CW = 5, FL_CCW = 6;
    static constexpr uint8_t FR_A = 30, FR_B = 31, FR_CW = 7, FR_CCW = 8;
    static constexpr uint8_t RL_A = 24, RL_B = 25, RL_CW = 9, RL_CCW = 10;
    static constexpr uint8_t RR_A = 28, RR_B = 29, RR_CW = 11, RR_CCW = 12;

    float _target_wheel_velocities[4];  // latest target velocities
    float _current_wheel_velocities[4]; // latest realized velocities

    bool _soft_stopping;
    float _soft_stop_deceleration;
    unsigned long _last_soft_stop_time;
    bool _is_stopped;
};

#endif
