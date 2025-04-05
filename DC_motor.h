#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <Arduino.h>

class DC_motor {
public:
    DC_motor(uint8_t pwm_cw_pin, uint8_t pwm_ccw_pin, uint8_t enable_pin = 255, int deadband = 10);

    void begin();
    void setPWM(int pwm); // -255 to 255, handles direction and deadband
    void disable();
    void enable();
    void update(); // Call periodically to check for timeout

    void setDeadband(int db);
    void setTimeout(unsigned long timeout_ms);
    bool isEnabled();
    bool isTimedOut();
    void clearTimeoutFlag();

private:
    uint8_t _pwm_cw_pin, _pwm_ccw_pin, _enable_pin;
    int _deadband;
    bool _has_enable;
    bool _enabled;
    bool _timeout_flag;

    unsigned long _last_command_time;
    unsigned long _timeout_ms;

    void applyPWM(int pwm);
};

#endif
