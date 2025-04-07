#include "Wheel.h"

Wheel::Wheel(
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
    bool reverse_direction,
    float gear_reduction
    )
    : _encoder(A_pin, B_pin, reverse_direction, 3, 64.0),
      _motor(pwm_cw_pin, pwm_ccw_pin, enable_pin),
      _pid(kp, ki, kd, kf, loop_dt, -150.0, 150.0, 100000.0),
      _last_pwm_output(0),
      _reverse(reverse_direction),
      _gear_reduction(gear_reduction)
{}

void Wheel::begin() {
    _encoder.begin();
    _motor.begin();
}

void Wheel::enable() {
    _motor.enable();
}

void Wheel::disable() {
    _motor.disable();
}


void Wheel::update(float target_velocity_rad_s) {
    _encoder.update();
    float current_velocity = _encoder.getAngularVelocity();

    // float error = target_velocity_rad_s - current_velocity;
    float pwm_output = _pid.update(target_velocity_rad_s, current_velocity);

    // if (_reverse) {
    //   pwm_output = -pwm_output;
    // }
    _last_pwm_output = pwm_output;
    _motor.setPWM((int)pwm_output);
    _motor.update(); // handle timeout logic
}

void Wheel::stop() {
    _motor.setPWM(0);
    _pid.reset();
}

float Wheel::getAngularVelocity() {
    return _encoder.getAngularVelocity() / _gear_reduction;
}

float Wheel::getAngle() {
    return _encoder.getAngularPosition();
}
  
float Wheel::getPWMOutput() {
    return _last_pwm_output;
}
