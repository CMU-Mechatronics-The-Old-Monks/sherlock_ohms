#ifndef WHEEL_H
#define WHEEL_H

#include "EncoderWrapper.h"
#include "DC_motor.h"
#include "PID.h"

class Wheel {
public:
    Wheel(
        uint8_t A_pin, 
        uint8_t B_pin,
        uint8_t pwm_cw_pin, 
        uint8_t pwm_ccw_pin, 
        uint8_t enable_pin,
        float kp, 
        float ki, 
        float kd, 
        float kf,
        float loop_dt,
        bool reverse_direction = false,
        float gear_reduction = 50
    );

    void begin();
    void enable();
    void disable();
    void update(float target_velocity_rad_s); // target velocity [rad/s]
    void stop();
    float getAngularVelocity(); // current angular velocity [rad/s]
    float getAngle();
    float getPWMOutput();       // last PWM applied

private:
    EncoderWrapper _encoder;
    DC_motor _motor;
    PID _pid;

    float _last_pwm_output;
    bool _reverse;
    float _gear_reduction;
};

#endif
