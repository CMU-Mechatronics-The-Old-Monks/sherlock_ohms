#ifndef BNO055_H
#define BNO055_H

#include <Arduino.h>
#include <Wire.h>

// Default BNO055 address (if ADR pin is low). If high, use 0x29 instead.
#define BNO055_ADDRESS 0x28

// Key registers for checking chip, setting mode, etc.
#define BNO055_CHIP_ID   0x00
#define BNO055_OPR_MODE  0x3D
// We won't worry about Page ID, since we're only doing basic config.
 
// Linear Acceleration registers (gravity-free) in fusion mode
// We'll read only X and Y. We'll ignore Z entirely.
#define BNO055_LIA_DATA_X_LSB 0x28
#define BNO055_LIA_DATA_X_MSB 0x29
#define BNO055_LIA_DATA_Y_LSB 0x2A
#define BNO055_LIA_DATA_Y_MSB 0x2B
// If you wanted Z, you'd also use 0x2C, 0x2D.

// BNO055 operation modes
#define BNO055_OPR_CONFIGMODE 0x00  // must enter config to change certain settings
#define BNO055_OPR_IMU        0x08  // IMU = Gyro + Accel, fusion output for linear accel

class BNO055 {
public:
    BNO055(TwoWire &wirePort = Wire, uint8_t addr = BNO055_ADDRESS);

    bool begin();    // Initializes the BNO055 in IMU mode so we can read linear accel
    void update();   // Reads X,Y linear acceleration, integrates into velocity

    // Gravity-free linear acceleration (m/s^2), ignoring Z
    float getAccX() const { return _accelX; }
    float getAccY() const { return _accelY; }

    // Integrated velocity in 2D (m/s)
    float getVelX() const { return _velX; }
    float getVelY() const { return _velY; }

private:
    // Reads X,Y from the linear acceleration registers (LIA). Ignores Z entirely.
    void readLinearAccelXY();

    // Helper to set operation mode
    void setOperationMode(uint8_t mode);

    // Low-level read/write
    uint8_t read8(uint8_t reg);
    uint16_t read16(uint8_t reg);
    void write8(uint8_t reg, uint8_t val);

    TwoWire *_wire;
    uint8_t _addr;

    unsigned long _lastUpdate;  // track elapsed time between updates

    float _accelX;  // linear accel X (m/s^2), gravity removed
    float _accelY;  // linear accel Y (m/s^2), gravity removed
    float _velX;    // integrated velocity X (m/s)
    float _velY;    // integrated velocity Y (m/s)
};

#endif




