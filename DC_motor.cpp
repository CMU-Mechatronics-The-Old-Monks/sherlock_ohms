#include "DC_motor.h"

DC_motor::DC_motor(
    uint8_t pwm_cw_pin, 
    uint8_t pwm_ccw_pin, 
    uint8_t enable_pin, 
    int deadband
    )
    : _pwm_cw_pin(pwm_cw_pin), 
      _pwm_ccw_pin(pwm_ccw_pin), 
      _enable_pin(enable_pin),
      _deadband(deadband), 
      _enabled(false), 
      _timeout_ms(10000),
      _timeout_flag(false)
{
    _has_enable = (_enable_pin != 255); // 255 means no enable pin given
}

void DC_motor::begin() {
    pinMode(_pwm_cw_pin, OUTPUT);
    pinMode(_pwm_ccw_pin, OUTPUT);
    if (_has_enable) {
        pinMode(_enable_pin, OUTPUT);
        // digitalWrite(_enable_pin, HIGH);
    }
    _last_command_time = millis();
}

void DC_motor::enable() {
    if (_has_enable) digitalWrite(_enable_pin, HIGH);
    _enabled = true;
}

void DC_motor::disable() {
    applyPWM(0);
    if (_has_enable) digitalWrite(_enable_pin, LOW);
    _enabled = false;
}

void DC_motor::setPWM(int pwm) {
    _last_command_time = millis();

    if (!_enabled) return;

    // Deadband check
    if (abs(pwm) < _deadband) {
        //applyPWM(0);
        pwm = 0;
        //return;
    }

    pwm = constrain(pwm, -255, 255);
    applyPWM(pwm);
}

void DC_motor::applyPWM(int pwm) {
    if (pwm > 0) {
        analogWrite(_pwm_cw_pin, pwm);
        analogWrite(_pwm_ccw_pin, 0);
    } else if (pwm < 0) {
        analogWrite(_pwm_cw_pin, 0);
        analogWrite(_pwm_ccw_pin, -pwm);
    } else {
        analogWrite(_pwm_cw_pin, 0);
        analogWrite(_pwm_ccw_pin, 0);
    }
}

void DC_motor::update() {
    if (_timeout_ms == 0 || !_enabled) return;

    if ((millis() - _last_command_time) > _timeout_ms) {
        disable();
        _timeout_flag = true;
    }
}

void DC_motor::setTimeout(unsigned long timeout_ms) {
    _timeout_ms = timeout_ms;
}

void DC_motor::setDeadband(int db) {
    _deadband = db;
}

bool DC_motor::isEnabled() {
    return _enabled;
}

bool DC_motor::isTimedOut() {
    return _timeout_flag;
}

void DC_motor::clearTimeoutFlag() {
    _timeout_flag = false;
}
