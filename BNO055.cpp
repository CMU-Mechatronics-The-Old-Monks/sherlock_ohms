#include "BNO055.h"

BNO055::BNO055(TwoWire &wirePort, uint8_t addr)
    : _wire(&wirePort), _addr(addr),
      _lastUpdate(0), _accelX(0.0f), _accelY(0.0f),
      _velX(0.0f), _velY(0.0f)
{
}

bool BNO055::begin()
{
    _wire->begin();

    // 1) Check chip ID
    uint8_t id = read8(BNO055_CHIP_ID);
    if (id != 0xA0) {
        Serial.print("BNO055 not detected or wrong chip ID: 0x");
        Serial.println(id, HEX);
        return false;
    }

    // 2) Put into CONFIGMODE before changing operation mode
    setOperationMode(BNO055_OPR_CONFIGMODE);
    delay(20);

    // We'll skip advanced config (units, etc.) for brevity.
    // We'll just set IMU mode to get linear accel w/o gravity on X & Y.

    // 3) Switch to IMU mode
    setOperationMode(BNO055_OPR_IMU);
    delay(20);

    _lastUpdate = millis();
    return true;
}

void BNO055::update()
{
    unsigned long now = millis();
    float dt = (now - _lastUpdate) / 1000.0f; 
    _lastUpdate = now;

    // 1) Read X,Y gravity-free linear acceleration
    readLinearAccelXY();

    // 2) Integrate acceleration -> velocity in X,Y
    _velX += _accelX * dt;
    _velY += _accelY * dt;
}

void BNO055::readLinearAccelXY()
{
    // Each axis is a 16-bit little-endian. 
    // We'll read X from 0x28/0x29, Y from 0x2A/0x2B, ignoring Z (0x2C/0x2D).

    int16_t xRaw = (int16_t)read16(BNO055_LIA_DATA_X_LSB);
    int16_t yRaw = (int16_t)read16(BNO055_LIA_DATA_Y_LSB);

    // In fusion modes, typically 1 LSB = 0.01 m/s^2 (check your unit settings).
    const float scale = 1.0f / 100.0f;

    _accelX = xRaw * scale;
    _accelY = yRaw * scale;
}

// ----------------------- Low-level -----------------------
void BNO055::setOperationMode(uint8_t mode)
{
    write8(BNO055_OPR_MODE, mode);
    delay(20);
}

uint8_t BNO055::read8(uint8_t reg)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);

    _wire->requestFrom((int)_addr, 1);
    return _wire->read();
}

uint16_t BNO055::read16(uint8_t reg)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);

    _wire->requestFrom((int)_addr, 2);
    uint8_t lsb = _wire->read();
    uint8_t msb = _wire->read();
    return (uint16_t)((msb << 8) | lsb);
}

void BNO055::write8(uint8_t reg, uint8_t val)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}
