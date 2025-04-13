#pragma once
#include <WString.h>
#include <Wire.h>
#include <cstdint>

struct HeadingSensor
{
    HeadingSensor() {}

    virtual int64_t lastRead() = 0;
    virtual int16_t heading() = 0;
};

enum CompassMode
{
    COMPASS_IDLE,
    CONTINUOUS_PERF_HIGH_GAIN, // high-performance, high gain
    CONTINUOUS_EFF_HIGH_GAIN,  // energy-saving, high gain
    CONTINUOUS_EFF_LOW_GAIN    // energy-saving, low gain
};

enum CompassStatus
{
    COMPASS_OK = 0,
    COMPASS_DATA_TOO_LONG = 1,
    COMPASS_NACK_ADDR = 2,
    COMPASS_NACK_DATA = 3,
    COMPASS_OTHER_ERR = 4,
    COMPASS_TIMEOUT = 5,
    COMPASS_UNINITIALIZED = 6
};

struct CompassXYZ
{
    uint8_t status;
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Compass : HeadingSensor
{
    int8_t _lastErr; // if less than 0, it's a read error; otherwise it's CompassStatus
    int64_t _lastRead;
    CompassXYZ xyz;

    Compass() : HeadingSensor(), _lastErr(CompassStatus::COMPASS_UNINITIALIZED) {}

    virtual bool begin() = 0;
    virtual String selfTest() = 0;

    virtual uint8_t setMode(CompassMode m) = 0;
    virtual int8_t readXYZ() = 0;

    virtual int64_t lastRead() override;
    virtual int16_t heading() override;
};

struct QMC5883LCompass : Compass
{
    TwoWire &wire;
    QMC5883LCompass(TwoWire &wire) : Compass(), wire(wire) {}

    bool begin() override;
    String selfTest() override;

    uint8_t setMode(CompassMode m) override;
    int8_t readXYZ() override;
};

struct UninitializedCompass : Compass
{
    UninitializedCompass() : Compass() {}
    bool begin() override;
    String selfTest() override;

    uint8_t setMode(CompassMode m) override;
    int8_t readXYZ() override;
};

#ifdef COMPASS_ENABLED
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#define HMC5883Type Adafruit_HMC5883_Unified
#else
#define HMC5883Type UninitializedCompass
#endif

extern HMC5883Type _mag;

struct HMC5883LCompass : Compass
{
    HMC5883Type &mag;
    long calStart = 0;
    // Variables for dynamic calibration
    float x_min = 1000, x_max = -1000;
    float y_min = 1000, y_max = -1000;
    float z_min = 1000, z_max = -1000;

    HMC5883LCompass() : Compass(), mag(_mag) {}

    bool begin() override;
    String selfTest() override;

    uint8_t setMode(CompassMode m) override;
    int8_t readXYZ() override;
};

struct DroneHeading : HeadingSensor
{
    int64_t _lastRead;
    int16_t _heading;

    DroneHeading() : HeadingSensor(), _lastRead(-1), _heading(-999) {}

    void setHeading(int64_t now, int16_t heading);

    int64_t lastRead() override;
    int16_t heading() override;
};

#define QMC5883_ADDR 0xD
#define QMC5883_X_LSB_REG 0
#define QMC5883_X_MSB_REG 1
#define QMC5883_Y_LSB_REG 2
#define QMC5883_Y_MSB_REG 3
#define QMC5883_Z_LSB_REG 4
#define QMC5883_Z_MSB_REG 5
#define QMC5883_STATUS_REG 6
#define QMC5883_T_LSB_REG 7
#define QMC5883_T_MSB_REG 8
#define QMC5883_CTR_REG 9
#define QMC5883_CTR2_REG 0xA
#define QMC5883_FBR_REG 0xB

#define QMC5883_STATUS_DRDY 1
