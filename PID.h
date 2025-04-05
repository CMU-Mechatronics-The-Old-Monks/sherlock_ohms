#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, 
        float ki, 
        float kd, 
        float dt,
        float output_min = -255.0, float output_max = 255.0,
        float derivative_filter_alpha = 1.0, 
        float integral_limit = 1000.0
    );

    float update(float setpoint, float measurement);
    void reset();

    // Gain setters
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min_val, float max_val);
    void setDerivativeFilterAlpha(float alpha);
    void setIntegralLimit(float limit);

private:
    float _kp, _ki, _kd;
    float _dt;

    float _output_min, _output_max;
    float _integral_limit;
    float _filter_alpha;

    float _prev_error;
    float _integral;
    float _prev_derivative;

    float clamp(float value, float min_val, float max_val);
};

#endif
