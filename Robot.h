#ifndef ROBOT_H
#define ROBOT_H

#include "Wheel.h"
#include "LoadCellSensor.h"
#include "VSensor.h"
#include "DataManager.h"
#include "IMU.h"


class Robot {
public:
    Robot(); // Use constructor to initialize pin mappings
    void begin();
    void update(); // Call periodically in loop()
    void enableWheels();
    void disableWheels();

    void setWheelVelocities(float v_fl, float v_fr, float v_rl, float v_rr); // rad/s
    float getWheelAngularVelocity(int index); // 0-3
    float* getWheelAngularVelocities();
    float getWheelAngle(int index);
    float* getWheelAngles();
    float* getWheelPWMValues();
    
    void emergencyStopAll();
    void initiateSoftStop(float deceleration = 5.0);
    void updateSoftStop();
    bool isSoftStopping();
    bool isStopped();
    
    float getBodyVx();   // Get body velocity X
    float getBodyVy();   // Get body velocity Y
    float getYaw();      // Get yaw angle
    void printIMU();  // Optional: rename to printData or printState
    float getWheelAngularVelocity(int wheelIndex);  // 0–3


private:
    Wheel* _wheels[4]; // 0 = FL, 1 = FR, 2 = RL, 3 = RR

    // Pin decs derived from schematic: https://drive.google.com/drive/u/0/folders/1AQKTDO6EsOuE3jIKUlD8XTELAerbCs_S
    static constexpr uint8_t STEP_EN = 40;
    static constexpr uint8_t Z_STEP = 2;
    static constexpr uint8_t Z_DIR = 35;
    
    static constexpr uint8_t DRIVE_EN = 41;
    static constexpr uint8_t FL_A = 26, FL_B = 27, FL_CW = 5, FL_CCW = 6;
    static constexpr uint8_t FR_A = 30, FR_B = 31, FR_CW = 8, FR_CCW = 7;
    static constexpr uint8_t RL_A = 24, RL_B = 25, RL_CW = 9, RL_CCW = 10;
    static constexpr uint8_t RR_A = 28, RR_B = 29, RR_CW = 12, RR_CCW = 11;

    float _target_wheel_velocities[4];  // latest target velocities
    float _current_wheel_velocities[4]; // latest realized velocities
    float _current_pwm_values[4];
    float _current_wheel_angles[4];

    bool _soft_stopping;
    float _soft_stop_deceleration;
    unsigned long _last_soft_stop_time;
    bool _is_stopped;

    LoadCellSensor _loadCell;
    VSensor vsensor;
    IMU imu; 
    
};

#endif

// #ifndef ROBOT_H
// #define ROBOT_H

// #include "Wheel.h"

// class Robot {
// public:
//     Robot();
//     void begin();
//     void update();

//     void enableWheels();
//     void disableWheels();

//     void setWheelVelocity(float velocity);
//     float getWheelAngularVelocity();
//     float getWheelAngle();
//     float getWheelPWMValue();

//     void emergencyStop();
//     void initiateSoftStop(float deceleration = 5.0);
//     void updateSoftStop();
//     bool isSoftStopping();
//     bool isStopped();

// private:
//     Wheel* _wheel;

//     // Pin config (same as FL from 4-wheel config)
//     // static constexpr uint8_t A_PIN   = 26;
//     // static constexpr uint8_t B_PIN   = 27;
//     // static constexpr uint8_t CW_PIN  = 5;
//     // static constexpr uint8_t CCW_PIN = 6;
//     // static constexpr uint8_t EN_PIN  = 41;

//     static constexpr uint8_t A_PIN   = 30;
//     static constexpr uint8_t B_PIN   = 31;
//     static constexpr uint8_t CW_PIN  = 8;
//     static constexpr uint8_t CCW_PIN = 7;
//     static constexpr uint8_t EN_PIN  = 41;

//     float _target_velocity;
//     float _current_velocity;
//     float _current_angle;
//     float _current_pwm;

//     bool _soft_stopping;
//     float _soft_stop_deceleration;
//     unsigned long _last_soft_stop_time;
// };

// #endif
