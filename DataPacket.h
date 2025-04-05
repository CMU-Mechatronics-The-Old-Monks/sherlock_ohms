#ifndef DATAPACKET_H
#define DATAPACKET_H

#include <vector>
#include <cstdint>

class DataPacket {
public:
    // Constructor with size specification
    DataPacket(uint8_t num_floats);

    // Set the value at a specific index in the packet
    void set(uint8_t index, float value);

    // Get the value at a specific index from the packet
    float get(uint8_t index) const;

    // Serialize the packet data to a byte array with start and end markers
    const uint8_t* serialize() const;

    // Deserialize data from a byte array
    bool deserialize(const uint8_t* data);

    // Return the byte length of the serialized packet
    uint8_t byteLength() const;

private:
    std::vector<float> _data;  // Storing the data in a vector
    uint8_t _size;             // Number of floats expected in the packet
    static constexpr uint8_t START_BYTE = 0xAA;  // Start byte marker
    static constexpr uint8_t END_BYTE = 0xBB;    // End byte marker
};

#endif  // DATAPACKET_H
