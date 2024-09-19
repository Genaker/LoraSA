#include <cstdint>
#include <stdlib.h>

#ifndef LORASA_CORE_H

#define LORASA_CORE_H

#ifdef PRINT_DEBUG
#define LOG(args...) Serial.printf(args...)
#define LOG_IF(cond, args...)                                                            \
    if (cond)                                                                            \
    LOG(args...)
#elif !defined(LOG)
#define LOG(args...)
#define LOG_IF(cond, args...)
#endif

// Output Pixel Formula
// 1 = rssi / 4, 2 = (rssi / 2) - 22 or 20
constexpr int RSSI_OUTPUT_FORMULA = 2;

// based on the formula for RSSI_OUTPUT_FORMULA == 2
// -2 * (22 + RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE) < rssi =< -44
// practice may require a better pair of thresholds
constexpr float HI_RSSI_THRESHOLD = -44.0;
constexpr float LO_RSSI_THRESHOLD = HI_RSSI_THRESHOLD - 66;

// number of samples for RSSI method
#define SAMPLES_RSSI 12 // 21 //

struct Scan
{
    Scan(int sz)
        : res_size(sz), scale((float)sz / (HI_RSSI_THRESHOLD - LO_RSSI_THRESHOLD + 0.1))
    {
    }

    virtual float getRSSI();

    uint16_t rssiMethod(uint16_t *result);

    int res_size;
    float scale;
};

#endif
