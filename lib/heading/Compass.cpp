#include "heading.h"
#include <bus.h>

enum MountingOrientation
{
    XY,  // X forward, Y right, Z down
    X_Y, // X forward, Y left, Z up
    XZ,  // X forward, Z right, Y up
    X_Z, // X forward, Z left, Y down
    YX,  // Y forward, X right, Z up
    Y_X, // Y forward, X left, Z down
    YZ,  // Y forward, Z right, X down
    Y_Z, // Y forward, Z left, X up
    ZX,  // Z forward, X right, Y down
    Z_X, // Z forward, X left, Y up
    ZY,  // Z forward, Y right, X down
    Z_Y  // Z forward, Y left, X up
};

// Produces CompassXYZ in a canonical mounting orientation: X forward, Y left, Z up
CompassXYZ _orientation(MountingOrientation o, int16_t x, int16_t y, int16_t z)
{
    CompassXYZ res;
    res.status = 0;

    switch (o)
    {
    case XY:
    case X_Y:
    case XZ:
    case X_Z:
        res.x = x;
        break;
    case YX:
    case Y_X:
    case YZ:
    case Y_Z:
        res.x = y;
        break;
    case ZY:
    case Z_Y:
    case ZX:
    case Z_X:
        res.x = z;
        break;
    }

    switch (o)
    {
    case X_Y:
    case Z_Y:
        res.y = y;
        break;
    case XY:
    case ZY:
        res.y = -y;
        break;
    case Y_X:
    case Z_X:
        res.y = x;
        break;
    case YX:
    case ZX:
        res.y = -x;
        break;
    case X_Z:
    case Y_Z:
        res.y = z;
        break;
    case XZ:
    case YZ:
        res.y = -z;
        break;
    }

    switch (o)
    {
    case X_Y:
    case YX:
        res.z = z;
        break;
    case XY:
    case Y_X:
        res.z = -z;
        break;
    case XZ:
    case Z_X:
        res.z = y;
        break;
    case X_Z:
    case ZX:
        res.z = -y;
        break;
    case Y_Z:
    case Z_Y:
        res.z = x;
        break;
    case YZ:
    case ZY:
        res.z = -x;
        break;
    }

    return res;
}

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

    // heading for canonical mounting orientation: X forward, Y left, Z up
    float heading = atan2(xyz.y, xyz.x);

    // Once you have your heading, you must then add your 'Declination Angle', which
    // is the 'Error' of the magnetic field in your location. Find yours here:
    // http://www.magnetic-declination.com/ Mine is: -13* 2' W, which is ~13 Degrees,
    // or (which we need) 0.22 radians If you cannot find your Declination, comment
    // out these two lines, your compass will be slightly off.

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

bool UninitializedCompass::begin()
{
    _lastErr = CompassStatus::COMPASS_UNINITIALIZED;
    return false;
}

String UninitializedCompass::selfTest() { return "No compass is attached"; }

uint8_t UninitializedCompass::setMode(CompassMode m) { return 4; }

int8_t UninitializedCompass::readXYZ() { return 4; }

#ifdef COMPASS_ENABLED
Adafruit_HMC5883_Unified _mag = Adafruit_HMC5883_Unified(12345);
#else
UninitializedCompass _mag;
#endif

bool HMC5883LCompass::begin()
{
    _lastErr = CompassStatus::COMPASS_UNINITIALIZED;

#ifdef COMPASS_ENABLED

    if (!mag.begin())
    {
        return false;
    }

    String err = selfTest();
    if (!err.startsWith("OK\n"))
    {
        return false;
    }

    _lastErr = CompassStatus::COMPASS_OK;
    return true;
#else
    return mag.begin();
#endif
}

String HMC5883LCompass::selfTest()
{
#ifdef COMPASS_ENABLED
    sensor_t sensor;
    mag.getSensor(&sensor);
    return "OK\nSensor:       " + String(sensor.name) +
           "\nDriver Ver:   " + String(sensor.version) +
           "\nUnique ID:    " + String(sensor.sensor_id) +
           "\nMax Value:    " + String(sensor.max_value) + " uT" +
           "\nMin Value:    " + String(sensor.min_value) + " uT" +
           "\nResolution:   " + String(sensor.resolution) + " uT";
#else
    return mag.selfTest();
#endif
}

uint8_t HMC5883LCompass::setMode(CompassMode m)
{
#ifdef COMPASS_ENABLED
    _lastErr = m;
    return 0;
#else
    return 1;
#endif
}

int8_t HMC5883LCompass::readXYZ()
{
#ifdef COMPASS_ENABLED
    if (calStart == 0)
    {
        calStart = millis();
    }

    /* Get a new sensor event */
    sensors_event_t event2;
    mag.getEvent(&event2);
    sensors_event_t event3;
    mag.getEvent(&event3);

#ifdef COMPASS_DEBUG
    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    Serial.print("X: ");
    Serial.print(event2.magnetic.x);
    Serial.print("  ");
    Serial.print("Y: ");
    Serial.print(event2.magnetic.y);
    Serial.print("  ");
    Serial.print("Z: ");
    Serial.print(event2.magnetic.z);
    Serial.print("  ");
    Serial.println("uT");
#endif

    // Hold the module so that Z is pointing 'up' and you can measure the heading with
    // x&y Calculate heading when the magnetometer is level, then correct for signs of
    // axis. float heading = atan2(event.magnetic.y, event.magnetic.x); Use Y as the
    // forward axis float heading = atan2(event.magnetic.x, event.magnetic.y);
    /// If Z-axis is forward and Y-axis points upward:
    // float heading = atan2(event.magnetic.x, event.magnetic.y);
    //  If Z-axis is forward and X-axis points upward:
    //  float heading = atan2(event.magnetic.y, -event.magnetic.x);

    // heading based on the magnetic readings from the Z-axis (forward) and the X-axis
    // (perpendicular to Z, horizontal).
    // float heading = atan2(event.magnetic.z, event.magnetic.x);

    // Dynamicly Calibrated out

    // Read raw magnetometer data
    float x = (event2.magnetic.x + event3.magnetic.x) / 2;
    float y = (event2.magnetic.y + event3.magnetic.y) / 2;
    float z = (event2.magnetic.z + event3.magnetic.z) / 2;

    // Doing calibration first 1 minute
    if (millis() - calStart < 60000)
    {
        // Update min/max values dynamically
        x_min = min(x_min, x);
        x_max = max(x_max, x);
        y_min = min(y_min, y);
        y_max = max(y_max, y);
        z_min = min(z_min, z);
        z_max = max(z_max, z);
    }

#ifdef COMPASS_DEBUG
    Serial.println("x_min:" + String(x_min) + " x_max: " + String(x_max) +
                   " y_min: " + String(y_min));
#endif

    // Calculate offsets and scales in real-time
    float x_offset = (x_max + x_min) / 2;
    float y_offset = (y_max + y_min) / 2;
    float z_offset = (z_max + z_min) / 2;

    float x_scale = (x_max - x_min) / 2;
    float y_scale = (y_max - y_min) / 2;
    float z_scale = (z_max - z_min) / 2;

    // Apply calibration to raw data
    float calibrated_x = (x - x_offset) / x_scale;
    float calibrated_y = (y - y_offset) / y_scale;
    float calibrated_z = (z - z_offset) / z_scale;

    xyz = _orientation(ZX, calibrated_x, calibrated_y, calibrated_z);
    return 0;
#else
    return mag.readXYZ();
#endif
}
