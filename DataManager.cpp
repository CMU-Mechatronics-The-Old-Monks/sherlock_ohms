#include "DataManager.h"

DataManager::DataManager(uint8_t num_floats)
    : _packet(num_floats), _expected_floats(num_floats) {}

void DataManager::packData(const float* values, uint8_t length) {
    uint8_t count = min(length, _expected_floats);
    for (int i = 0; i < count; ++i) {
        _packet.set(i, values[i]);
    }
}

void DataManager::parseData(float* output, uint8_t max_length) {
    uint8_t count = min(max_length, _expected_floats);
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

    // Optional: validate here if you want CRC etc
    _packet.deserialize(buf);
    return true;
}

uint8_t DataManager::getPacketSize() const {
    return _packet.byteLength();
}
