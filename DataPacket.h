#ifndef DATA_PACKET_H
#define DATA_PACKET_H

#include <Arduino.h>

class DataPacket {
public:
    static const uint8_t START_BYTE = 0xAA;
    static const uint8_t END_BYTE   = 0x55;

    DataPacket(uint8_t num_floats);

    void set(uint8_t index, float value);
    float get(uint8_t index) const;

    uint8_t* serialize();                    // Call before sending
    void deserialize(const uint8_t* buffer); // Call after receiving

    uint8_t byteLength() const;             // Total bytes (start + floats + end)
    uint8_t dataLength() const;             // Only the float payload in bytes

private:
    uint8_t _num_floats;
    float* _data;

    uint8_t* _buffer; // raw byte buffer for sending/receiving
};

#endif
