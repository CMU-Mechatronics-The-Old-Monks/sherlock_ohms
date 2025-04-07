#ifndef ENCODERWRAPPER_H
#define ENCODERWRAPPER_H

#include <Encoder.h> // PJRC Library

class EncoderWrapper {
public:
    EncoderWrapper(uint8_t A_pin, uint8_t B_pin, bool reverse = false, unsigned int buffer_size = 3, float count_per_revolution = 64.0);
    void begin(); // Optionally store time
    void update(); // Call periodically to compute velocity
    float getVelocity(); // Filtered velocity in counts/sec
    float getAngularVelocity();
    long getCounts();    // Current raw count
    float getAngularPosition();
    void reset();        // Zero the encoder

private:
    uint8_t _A_pin, _B_pin;
    bool _reverse;

    Encoder _encoder; // PJRC Encoder instance

    unsigned long _last_update_time;
    long _last_count;

    float* _velocity_buffer;
    unsigned int _buffer_size;
    unsigned int _buffer_index;
    float _count_per_revolution;

    float computeFilteredVelocity(float new_vel);
};

#endif
