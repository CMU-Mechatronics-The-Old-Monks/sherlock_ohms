#include "DataPacket.h"
#include <string.h> // for memcpy

DataPacket::DataPacket(uint8_t num_floats)
    : _num_floats(num_floats)
{
    _data = new float[_num_floats];
    _buffer = new uint8_t[byteLength()];

    for (int i = 0; i < _num_floats; ++i) {
        _data[i] = 0.0f;
    }
}

void DataPacket::set(uint8_t index, float value) {
    if (index >= _num_floats) return;
    _data[index] = value;
}

float DataPacket::get(uint8_t index) const {
    if (index >= _num_floats) return 0.0f;
    return _data[index];
}

uint8_t* DataPacket::serialize() {
    _buffer[0] = START_BYTE;

    // Copy each float into the buffer
    for (int i = 0; i < _num_floats; ++i) {
        memcpy(&_buffer[1 + i * sizeof(float)], &_data[i], sizeof(float));
    }

    _buffer[1 + _num_floats * sizeof(float)] = END_BYTE;
    return _buffer;
}

void DataPacket::deserialize(const uint8_t* buffer) {
    // Optionally: check start/end bytes before trusting
    if (buffer[0] != START_BYTE || buffer[byteLength() - 1] != END_BYTE) {
        return; // Invalid packet
    }

    for (int i = 0; i < _num_floats; ++i) {
        memcpy(&_data[i], &buffer[1 + i * sizeof(float)], sizeof(float));
    }
}

uint8_t DataPacket::byteLength() const {
    return 1 + (_num_floats * sizeof(float)) + 1; // start + payload + end
}

uint8_t DataPacket::dataLength() const {
    return _num_floats * sizeof(float);
}
