#include "Robot.h"
#include "LoadCellSensor.h"
#include "VSensor.h"
#include "tof.h"
#include "IMU.h"

Robot::Robot() 
    : vsensor(5.0, 38, 200)
  {
    _wheels[0] = new Wheel( // FL
        FL_A,       // Encoder A pin
        FL_B,       // Encoder B pin
        FL_CW,      // Motor CW pin
        FL_CCW,     // Motor CCW pin
        DRIVE_EN,   // DC driver enable pin
        0.1,        // PID kp
        0.0,        // PID ki
        0.001,      // PID kd
        16.0,       // PID ff gain
        0.01,       // loop_dt
        true        // changes direction for CW / CCW 
    ); 
    _wheels[1] = new Wheel( // FR
        FR_A, 
        FR_B, 
        FR_CW, 
        FR_CCW, 
        DRIVE_EN, 
        0.1,        // PID kp
        0.0,        // PID ki
        0.001,      // PID kd
        16.0,       // PID ff gain
        0.01, 
        false
    ); 
    _wheels[2] = new Wheel( // BL
        RL_A, 
        RL_B, 
        RL_CW, 
        RL_CCW, 
        DRIVE_EN, 
        0.1,        // PID kp
        0.0,        // PID ki
        0.001,      // PID kd
        16.0,       // PID ff gain
        0.01, 
        true
    ); 
    _wheels[3] = new Wheel( // BR
        RR_A, 
        RR_B, 
        RR_CW, 
        RR_CCW, 
        DRIVE_EN, 
        0.1,        // PID kp
        0.0,        // PID ki
        0.001,      // PID kd
        16.0,       // PID ff gain
        0.01, 
        false
    ); 
    for (int i = 0; i < 4; ++i) _target_wheel_velocities[i] = 0.0;
  }


void Robot::begin() {
    for (int i = 0; i < 4; ++i) {
        _wheels[i]->begin();
    }
    _loadCell.begin();
    vsensor.begin();
    imu.begin();
}

void Robot::enableWheels() {
    for (int i = 0; i < 4; ++i) {
        _wheels[i]->enable();
    }
}

void Robot::disableWheels() {
    for (int i = 0; i < 4; ++i) {
        _wheels[i]->disable();
    }
}

void Robot::update() {
    for (int i = 0; i < 4; ++i) {
        _wheels[i]->update(_target_wheel_velocities[i]);
    }
    // _loadCell.update();
    // float w = _loadCell.getWeight();

// // Optional print for debug
//     Serial.print("Weight: ");
//     Serial.print(w, 5);
//     Serial.println(" g");

    vsensor.update();
    float trueACVoltage = vsensor.getTrueVoltage();  // Calibrated value

    vsensor.setCalibration(0.87, 100.0);  // Still used for getTrueVoltage()

    // float vNorm = vsensor.getNormalizedVoltage(2.25, 5.0);  // Magnitude only
    // Serial.print("Normalized [0–5]: ");
    // Serial.println(vNorm);
    // Serial.print("  |  True AC Voltage: ");
    // Serial.println(trueACVoltage);

    imu.update();

    // std::vector<float> imu_values = { vx, vy, yaw };
    // imuData.packAndTransmitData(imu_values, Serial);

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

float Robot::getWheelAngle(int index) {
    if (index < 0 || index > 3) return 0.0;
    return _wheels[index]->getAngle();
}

float* Robot::getWheelAngles() {
    for (int i = 0; i < 4; ++i) {
        _current_wheel_angles[i] = getWheelAngle(i);
    }
    return _current_wheel_velocities;
}

float* Robot::getWheelPWMValues() {
    for (int i = 0; i < 4; ++i) {
        _current_pwm_values[i] = _wheels[i]->getPWMOutput();
    }
    return _current_pwm_values;
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

float Robot::getBodyVx() {
    return imu.getVx();
}

float Robot::getBodyVy() {
    return imu.getVy();
}

float Robot::getYaw() {
    return imu.getYaw();
}

void Robot::printIMU() {
    Serial.print("Vx: ");
    Serial.print(getBodyVx(), 3);  // 3 decimal places

    Serial.print("  Vy: ");
    Serial.print(getBodyVy(), 3);

    Serial.print("  Yaw: ");
    Serial.println(getYaw(), 2);
}

// #include "Robot.h"

// Robot::Robot() {
//     // _wheel = new Wheel(
//     //     A_PIN,
//     //     B_PIN,
//     //     CW_PIN,
//     //     CCW_PIN,
//     //     EN_PIN,
//     //     0.1,    // kp
//     //     0.0,    // ki
//     //     0.0001,    // kd
//     //     16.0,       // PID ff gain
//     //     0.01,
//     //     true,
//     //     50.0
//     // );
//     _wheel = new Wheel(
//         A_PIN,
//         B_PIN,
//         CW_PIN,
//         CCW_PIN,
//         EN_PIN,
//         0.1,    // kp
//         0.0,    // ki
//         0.0001,    // kd
//         16.0,       // PID ff gain
//         0.01,   // loop_dt
//         false   // reverse
//     );

//     _target_velocity = 0.0f;
//     _soft_stopping = false;
//     _last_soft_stop_time = 0;
// }

// void Robot::begin() {
//     _wheel->begin();
// }

// void Robot::enableWheels() {
//     _wheel->enable();
// }

// void Robot::disableWheels() {
//     _wheel->disable();
// }

// void Robot::update() {
//     _wheel->update(_target_velocity);
//     _current_velocity = _wheel->getAngularVelocity();
//     _current_angle = _wheel->getAngle();
//     _current_pwm = _wheel->getPWMOutput();
// }

// void Robot::setWheelVelocity(float velocity) {
//     _target_velocity = velocity;
// }

// float Robot::getWheelAngularVelocity() {
//     return _current_velocity;
// }

// float Robot::getWheelAngle() {
//     return _current_angle;
// }

// float Robot::getWheelPWMValue() {
//     return _current_pwm;
// }

// void Robot::emergencyStop() {
//     _wheel->stop();
//     _target_velocity = 0.0f;
// }

// void Robot::initiateSoftStop(float deceleration) {
//     _soft_stopping = true;
//     _soft_stop_deceleration = deceleration;
//     _last_soft_stop_time = millis();
// }

// void Robot::updateSoftStop() {
//     if (!_soft_stopping) return;

//     const float dt = 0.01f;
//     unsigned long now = millis();
//     if (now - _last_soft_stop_time < dt * 1000) return;

//     _last_soft_stop_time = now;

//     if (abs(_target_velocity) > 0.01f) {
//         float delta = _soft_stop_deceleration * dt;
//         if (_target_velocity > 0) {
//             _target_velocity = max(0.0f, _target_velocity - delta);
//         } else {
//             _target_velocity = min(0.0f, _target_velocity + delta);
//         }
//     } else {
//         _soft_stopping = false;
//         _target_velocity = 0.0f;
//     }

//     _wheel->update(_target_velocity);
// }

// bool Robot::isSoftStopping() {
//     return _soft_stopping;
// }

// bool Robot::isStopped() {
//     return abs(_current_velocity) < 0.02f;
// }
