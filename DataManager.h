#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include "DataPacket.h"
#include <HardwareSerial.h>
#include <vector>

class DataManager {
public:
    // Constructor to initialize with number of expected floats
    DataManager(uint8_t num_floats);

    // Pack raw data into the data packet
    void packData(const std::vector<float>& values);

    // Parse data from the packet into a raw array
    void parseData(float* output, uint8_t max_length);

    // Transmit data to serial
    void transmitData(HardwareSerial& serial);

    // Receive data from serial
    bool receiveData(HardwareSerial& serial);

    // Get the byte size of the packet
    uint8_t getPacketSize() const;

    // Helper function to pack and transmit data in one call
    void packAndTransmitData(const std::vector<float>& values, HardwareSerial& serial);

private:
    DataPacket _packet;  // Packet instance to handle data
    uint8_t _expected_floats;  // Expected number of floats
};

#endif  // DATAMANAGER_H
