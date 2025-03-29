#include "heading.h"

/*
 * QMC5883L Registers:
 *
 * 00...05 Data Output registers
 * 00: X[7:0] (X LSB)
 * 01: X[15:8] (X MSB)
 * 02: Y[7:0] (Y LSB)
 * 03: Y[15:8] (Y MSB)
 * 04: Z[7:0] (Z LSB)
 * 05: Z[15:8] (Z MSB)
 *
 * 06 Status:
 *      |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0   |
 *      |      Reserved               | DOR | OVL | DRDY |
 *
 * DRDY - Data Ready, set to 1 when all three axis data is ready and loaded in
 * the continuous measurement mode, and it is reset to 0 upon reading any of the
 * registers 00...05
 *
 * OVL - Overflow, set to 1 when any data for any axis of the magnetic sensor is
 * out of range. The output data saturates at -32768 and 32767. If any of the
 * axis exceeds the range, OVL is set to 1.
 *
 * DOR - Data Skip is set to 1, if all the channels of output data registers are
 * skipped in the continuous-measurement mode. It is set back to 0 by reading
 * data from registers 00...05
 *
 * 07, 08 Temperature Data
 * 07: TOUT[7:0]
 * 08: TOUT[15:8]
 *
 * 09 Control:
 *      |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 *      |  OSR[1:0] | RNG[1:0]  | ODR[1:0]  | MODE[1:0] |
 *
 *                     | 00      | 01         |   10  |   11  |
 * Mode | Mode Control | Standby | Continuous |   Reserved    |
 *      |              |         |            |               |
 * ODR  | Output Data  | 10Hz    | 50Hz       | 100Hz | 200Hz |
 *      | Rate         |         |            |       |       |
 * RNG  | Full Scale   | +/-2G   | +/-8G      |   Reserved    |
 *      |              |         |            |               |
 * OSR  | Over Sample  |   512   |    256     |  128  |  64   |
 *      | Ratio        |         |            |       |       |
 *
 * - Recommended RNG=+/-2G for open field, for better energy saving
 * - Recommended ODR=10Hz for plain compass
 *
 *
 * 0A Control2:
 *      |     7     |     6     |  5  |  4  |  3  |  2  |  1  |    0    |
 *      |  SOFT_RST | ROLL_PNT  |          Reserved           | INT_ENB |
 *
 * 0B Set/Reset period:
 * FBR[7:0]. Recommended to set this to 1
 */

bool QMC5883LCompass::begin()
{
    _lastErr = CompassStatus::COMPASS_UNINITIALIZED;

    String err = selfTest();
    if (!err.startsWith("OK\n"))
    {
        return false;
    }

    _lastErr = CompassStatus::COMPASS_OK;
    return true;
}

uint8_t _write_register(TwoWire &wire, uint8_t addr, uint8_t reg, uint8_t value,
                        bool skipValue = false)
{
    wire.beginTransmission(addr);
    size_t s = wire.write(reg);
    if (s == 1 && !skipValue)
    {
        s = wire.write(value);
    }

    size_t s1 = wire.endTransmission();
    if (s != 1 && s1 == 0)
    {
        return 1; // "data too long to fit in transmit buffer"
    }

    return s1;
}

int8_t _read_registers(TwoWire &wire, uint8_t addr, uint8_t reg, uint8_t *v, size_t sz,
                       bool skipRegister = false)
{
    if (!skipRegister)
    {
        uint8_t s = _write_register(wire, addr, reg, 0, true);
        if (s != 0)
        {
            return s;
        }
    }

    uint8_t r = wire.requestFrom(addr, sz);
    for (int i = 0; i < r; i++, v++)
    {
        *v = wire.read();
    }

    return r - sz;
}

uint8_t _read_register(TwoWire &wire, uint8_t addr, uint8_t reg, uint8_t &v,
                       bool skipRegister = false)
{
    uint8_t r = _read_registers(wire, addr, reg, &v, 1, skipRegister);
    if (r != 0)
    {
        return 1;
    }

    return 0;
}

int8_t _read_xyz(TwoWire &wire, CompassXYZ &xyz)
{
    xyz.status = 0;
    size_t s = _read_register(wire, QMC5883_ADDR, QMC5883_STATUS_REG, xyz.status);
    if (s != 0)
    {
        return s;
    }

    if ((xyz.status & QMC5883_STATUS_DRDY) == 0)
    {
        delay(10);
        s = _read_register(wire, QMC5883_ADDR, QMC5883_STATUS_REG, xyz.status);
        if (s != 0)
        {
            return s;
        }
    }

    int16_t mags[3];

    int8_t r = _read_registers(wire, QMC5883_ADDR, 0, (uint8_t *)&mags, 6);
    xyz.x = mags[0];
    xyz.y = mags[1];
    xyz.z = mags[2];

    return r;
}

uint8_t QMC5883LCompass::setMode(CompassMode m)
{
    if (m == CompassMode::COMPASS_IDLE)
    {
        uint8_t s = _write_register(wire, QMC5883_ADDR, QMC5883_FBR_REG, 0);
        s |= _write_register(wire, QMC5883_ADDR, QMC5883_CTR_REG, 0);
        return s;
    }

    uint8_t osr = 0b00;  // OSR=512
    uint8_t rng = 0b01;  // RNG = +/-8 Gauss
    uint8_t odr = 0b11;  // ODR = 200HZ
    uint8_t mode = 0b01; // MODE = Continuous

    switch (m)
    {
    case CompassMode::CONTINUOUS_PERF_HIGH_GAIN:
        break;
    case CompassMode::CONTINUOUS_EFF_HIGH_GAIN:
        odr = 0;
        break;
    case CompassMode::CONTINUOUS_EFF_LOW_GAIN:
        odr = 0;
        rng = 0;
        break;
    default:
        return 1;
    }

    uint8_t s =
        _write_register(wire, QMC5883_ADDR, QMC5883_FBR_REG, 1); // set/reset period
    if (s != 0)
    {
        return s;
    }

    return _write_register(wire, QMC5883_ADDR, QMC5883_CTR_REG,
                           (osr << 6) | (rng << 4) | (odr << 2) | mode);
}

String QMC5883LCompass::selfTest()
{
    uint8_t s = setMode(CompassMode::CONTINUOUS_PERF_HIGH_GAIN);
    if (s != 0)
    {
        return String("Could not set Compass");
    }

    delay(100);

    bool errors = false;

    String res;
    for (int i = 0; i < 100; i++)
    {
        CompassXYZ xyz;
        int8_t r = _read_xyz(wire, xyz);
        if (r < 0)
        {
            errors = true;
            res += " oops, requested 6, got back only " + String(-r);
            continue;
        }

        if (r != 0)
        {
            errors = true;
            res += "\nCould not get XYZ: err:" + String(r);
            continue;
        }

        res += "\n status: " + String(xyz.status, 2) + " X: " + String(xyz.x) +
               " Y: " + String(xyz.y) + " Z: " + String(xyz.z);
    }

    if (!errors)
    {
        res = "OK\n" + res;
    }

    return res;
}

int8_t QMC5883LCompass::readXYZ() { return _read_xyz(wire, xyz); }

int64_t Compass::lastRead()
{
    if (_lastErr == CompassStatus::COMPASS_UNINITIALIZED)
    {
        return -1;
    }
    return _lastRead;
}

int16_t Compass::heading()
{
    if (_lastErr == CompassStatus::COMPASS_UNINITIALIZED)
    {
        return -999;
    }

    _lastRead = millis();

    int8_t r = readXYZ();

    if (r != 0)
    {
        return -999;
    }

    float heading = atan2(xyz.y, xyz.x);
    float declinationAngle = 0.22;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if (heading < 0)
        heading += 2 * M_PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * M_PI)
        heading -= 2 * M_PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180 / M_PI;

    return headingDegrees;
}
