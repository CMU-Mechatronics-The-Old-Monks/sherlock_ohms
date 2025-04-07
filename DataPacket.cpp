#include "DataPacket.h"
#include <cstring>  // For memcpy


DataPacket::DataPacket(uint8_t num_floats)
    : _size(num_floats), _data(num_floats, 0.0f) {}

void DataPacket::set(uint8_t index, float value) {
    if (index < _size) {
        _data[index] = value;
    }
}

float DataPacket::get(uint8_t index) const {
    if (index < _size) {
        return _data[index];
    }
    return 0.0f;  // Default value in case of out-of-bounds access
}

const uint8_t* DataPacket::serialize() const {
    static uint8_t buffer[256];  // Assuming max packet size
    uint8_t* ptr = buffer;

    // Add start byte
    *ptr++ = START_BYTE;

    // Copy the data into the buffer
    std::memcpy(ptr, _data.data(), _data.size() * sizeof(float));
    ptr += _data.size() * sizeof(float);

    // Add end byte
    *ptr = END_BYTE;

    return buffer;
}


bool DataPacket::deserialize(const uint8_t* data) {
    if (data[0] != START_BYTE || data[byteLength() - 1] != END_BYTE) {
        return false;
    }
    std::memcpy(_data.data(), data + 1, _data.size() * sizeof(float));
    return true;
}


uint8_t DataPacket::byteLength() const {
    // 1 byte for start, 1 byte for end, and size for data
    return static_cast<uint8_t>(_data.size() * sizeof(float) + 2);
}

