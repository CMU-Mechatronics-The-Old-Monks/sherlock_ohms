#include "PID.h"

PID::PID(
    float kp, 
    float ki, 
    float kd, 
    float kf,
    float dt,
    float output_min, 
    float output_max,
    float derivative_filter_alpha, 
    float integral_limit
    )
    : _kp(kp), 
      _ki(ki), 
      _kd(kd), 
      _kf(kf),
      _dt(dt),
      _output_min(output_min), 
      _output_max(output_max),
      _integral_limit(integral_limit),
      _filter_alpha(derivative_filter_alpha), 
      _prev_error(0.0), 
      _integral(0.0), 
      _prev_derivative(0.0) {}

float PID::update(float setpoint, float measurement) {

    float error_scaling = 1.0;
    float error = error_scaling*(setpoint - measurement);
   
    float derivative = (error - _prev_error) / _dt;

    float ff = _kf * setpoint;
   

    // Filter derivative
    // if (_kd > 0.0f) {
    //     derivative = _filter_alpha * derivative + (1 - _filter_alpha) * _prev_derivative;
    // } else {
    //     derivative = 0.0f;
    // }



    _integral += error * _dt;

    // Anti-windup
    _integral = clamp(_integral, -_integral_limit, _integral_limit);

    // float output = _kp * error + _ki * _integral + _kd * derivative;
    float output = _kp * error + _kd * derivative + _ki * _integral + ff;
    // Clamp output
    output = clamp(output, _output_min, _output_max);

    _prev_error = error;
    _prev_derivative = derivative;

    return output;
}

void PID::reset() {
    _prev_error = 0.0;
    _integral = 0.0;
    _prev_derivative = 0.0;
}

void PID::setGains(float kp, float ki, float kd,  float kf) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _kf = kf;
}

void PID::setOutputLimits(float min_val, float max_val) {
    _output_min = min_val;
    _output_max = max_val;
}

void PID::setDerivativeFilterAlpha(float alpha) {
    _filter_alpha = alpha;
}

void PID::setIntegralLimit(float limit) {
    _integral_limit = limit;
}

float PID::clamp(float value, float min_val, float max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}



