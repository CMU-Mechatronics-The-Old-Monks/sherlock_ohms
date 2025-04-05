#include "Robot.h"

Robot::Robot() {
    _wheels[0] = new Wheel( // FL
        FL_A,       // Encoder A pin
        FL_B,       // Encoder B pin
        FL_CW,      // Motor CW pin
        FL_CCW,     // Motor CCW pin
        DRIVE_EN,   // DC driver enable pin
        5.0,        // PID kp
        1.0,        // PID ki
        0.1,        // PID kd
        0.01,       // loop_dt
        false       // changes direction for CW / CCW 
    ); 
    _wheels[1] = new Wheel( // FR
        FR_A, 
        FR_B, 
        FR_CW, 
        FR_CCW, 
        DRIVE_EN, 
        5.0, 
        1.0, 
        0.1, 
        0.01, 
        true
    ); 
    _wheels[2] = new Wheel( // BL
        RL_A, 
        RL_B, 
        RL_CW, 
        RL_CCW, 
        DRIVE_EN, 
        5.0, 
        1.0, 
        0.1, 
        0.01, 
        false
    ); 
    _wheels[3] = new Wheel( // BR
        RR_A, 
        RR_B, 
        RR_CW, 
        RR_CCW, 
        DRIVE_EN, 
        5.0, 
        1.0, 
        0.1, 
        0.01, 
        true
    ); 
    for (int i = 0; i < 4; ++i) _target_wheel_velocities[i] = 0.0;
}

void Robot::begin() {
    for (int i = 0; i < 4; ++i) {
        _wheels[i]->begin();
    }
}

void Robot::update() {
    for (int i = 0; i < 4; ++i) {
        _wheels[i]->update(_target_wheel_velocities[i]);
    }
}

void Robot::setWheelVelocities(float v_fl, float v_fr, float v_rl, float v_rr) {
    _target_wheel_velocities[0] = v_fl;
    _target_wheel_velocities[1] = v_fr;
    _target_wheel_velocities[2] = v_rl;
    _target_wheel_velocities[3] = v_rr;
}

float Robot::getWheelAngularVelocity(int index) {
    if (index < 0 || index > 3) return 0.0;
    return _wheels[index]->getAngularVelocity();
}

float* Robot::getWheelAngularVelocities() {
    for (int i = 0; i < 4; ++i) {
        _current_wheel_velocities[i] = getWheelAngularVelocity(i);
    }
    return _current_wheel_velocities;
}

void Robot::emergencyStopAll() {
    for (int i = 0; i < 4; ++i) {
        _wheels[i]->stop();
        _target_wheel_velocities[i] = 0.0;
    }
}

void Robot::initiateSoftStop(float deceleration) {
    _soft_stopping = true;
    _soft_stop_deceleration = deceleration;
    _last_soft_stop_time = millis();
}

void Robot::updateSoftStop() {
    if (!_soft_stopping) return;

    const float dt = 0.01f; // 10ms
    unsigned long now = millis();
    if (now - _last_soft_stop_time < dt * 1000) return;

    _last_soft_stop_time = now;
    bool anyMoving = false;

    for (int i = 0; i < 4; ++i) {
        float v = _target_wheel_velocities[i];
        if (abs(v) > 0.01f) {
            anyMoving = true;

            float delta = _soft_stop_deceleration * dt;
            if (v > 0) {
                _target_wheel_velocities[i] = max(0.0f, v - delta);
            } else {
                _target_wheel_velocities[i] = min(0.0f, v + delta);
            }
        }

        _wheels[i]->update(_target_wheel_velocities[i]);
    }

    if (!anyMoving) {
        _soft_stopping = false;
    }
}

bool Robot::isSoftStopping() {
    return _soft_stopping;
}

bool Robot::isStopped() {
    const float epsilon = 0.02f; // rad/s threshold
    getWheelAngularVelocities();

    for (int i = 0; i < 4; ++i) {
        if (abs(_current_wheel_velocities[i]) > epsilon) {
            return false;
        }
    }
    emergencyStopAll(); // enforce 0.0 rad/s once below safe threshold
    return true;
}