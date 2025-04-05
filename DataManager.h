#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include "DataPacket.h"

class DataManager {
public:
    DataManager(uint8_t num_floats); // expected float count

    void packData(const float* values, uint8_t length);
    void parseData(float* output, uint8_t max_length);

    void transmitData(HardwareSerial& serial);
    bool receiveData(HardwareSerial& serial); // returns true if valid packet received

    uint8_t getPacketSize() const;

private:
    DataPacket _packet;
    uint8_t _expected_floats;
};

#endif
