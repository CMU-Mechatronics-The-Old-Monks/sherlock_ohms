#include "EncoderWrapper.h"

EncoderWrapper::EncoderWrapper(
    uint8_t A_pin, 
    uint8_t B_pin, 
    bool reverse,  
    unsigned int buffer_size, 
    float count_per_revolution
    )
    : _A_pin(A_pin), 
      _B_pin(B_pin), 
      _reverse(reverse),
      _encoder(A_pin, B_pin), 
      _last_update_time(0),  
      _last_count(0),
      _buffer_size(buffer_size), 
      _buffer_index(0),
      _count_per_revolution(count_per_revolution)
{
    _velocity_buffer = new float[_buffer_size];
    for (unsigned int i = 0; i < _buffer_size; ++i) {
        _velocity_buffer[i] = 0.0;
    }
}

void EncoderWrapper::begin() {
    _last_update_time = micros();
    _last_count = _encoder.read();
}

void EncoderWrapper::update() {
    unsigned long now = micros();
    long current_count = _encoder.read();

    long delta_counts = current_count - _last_count;
    if (_reverse) delta_counts *= -1;

    float dt = (now - _last_update_time) / 1e6;
    if (dt <= 0) return;

    float vel = delta_counts / dt;
    computeFilteredVelocity(vel);

    _last_update_time = now;
    _last_count = current_count;
}

float EncoderWrapper::computeFilteredVelocity(float new_vel) {
    _velocity_buffer[_buffer_index] = new_vel;
    _buffer_index = (_buffer_index + 1) % _buffer_size;

    float sum = 0.0;
    for (unsigned int i = 0; i < _buffer_size; ++i) {
        sum += _velocity_buffer[i];
    }
    return sum / _buffer_size;
}

float EncoderWrapper::getVelocity() {
    float sum = 0.0;
    for (unsigned int i = 0; i < _buffer_size; ++i) {
        sum += _velocity_buffer[i];
    }
    return sum / _buffer_size;
}

float EncoderWrapper::getAngularVelocity() { // [instantaneous filtered radians / s]
    float counts_per_sec = getVelocity();
    return (counts_per_sec / _count_per_revolution) * 2.0f * PI;
}

long EncoderWrapper::getCounts() {
    long c = _encoder.read();
    return _reverse ? -c : c;
}

float EncoderWrapper::getAngularPosition() { // [cumulative radians]
    return ((float)getCounts() / _count_per_revolution) * 2.0f * PI;
}

void EncoderWrapper::reset() {
    _encoder.write(0);
    _last_count = 0;
    for (unsigned int i = 0; i < _buffer_size; ++i) {
        _velocity_buffer[i] = 0.0;
    }
}
