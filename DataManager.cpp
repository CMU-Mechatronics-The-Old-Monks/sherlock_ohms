#include "DataManager.h"

DataManager::DataManager(uint8_t num_floats)
    : _packet(num_floats), _expected_floats(num_floats) {}

void DataManager::packData(const std::vector<float>& values) {
    uint8_t count = std::min(static_cast<uint8_t>(values.size()), _expected_floats);
    for (int i = 0; i < count; ++i) {
        _packet.set(i, values[i]);
    }
}

void DataManager::parseData(float* output, uint8_t max_length) {
    uint8_t count = std::min(max_length, _expected_floats);
    for (int i = 0; i < count; ++i) {
        output[i] = _packet.get(i);
    }
}

void DataManager::transmitData(HardwareSerial& serial) {
    serial.write(_packet.serialize(), _packet.byteLength());
}

bool DataManager::receiveData(HardwareSerial& serial) {
    if (serial.available() < _packet.byteLength()) return false;

    uint8_t buf[_packet.byteLength()];
    serial.readBytes(buf, _packet.byteLength());

    return _packet.deserialize(buf); // validate result
}

uint8_t DataManager::getPacketSize() const {
    return _packet.byteLength();
}

void DataManager::packAndTransmitData(const std::vector<float>& values, HardwareSerial& serial) {
    // Pack the data into the packet
    packData(values);

    // Transmit the data
    transmitData(serial);
}
