#include <cstdint>
#include <events.h>
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
#ifdef USING_SX1280PA
#define SAMPLES_RSSI 20
#endif

struct Scan
{
    uint64_t epoch;
    float current_frequency;
    uint64_t fr_begin;
    uint64_t fr_end;
    uint64_t drone_detection_level;
    bool sound_on;
    bool led_flag;
    uint64_t detection_count;
    bool animated;
    float trigger_level;

    Listener **eventListeners[(size_t)EventType::_MAX_EVENT_TYPE];
    size_t listener_count[(size_t)EventType::_MAX_EVENT_TYPE];

    Scan()
        : epoch(0), current_frequency(0), fr_begin(0), fr_end(0),
          drone_detection_level(0), sound_on(false), led_flag(false), detection_count(0),
          animated(false), trigger_level(0), listener_count{
                                                 0,
                                             } {};

    virtual float getRSSI() = 0;

    // rssiMethod gets the data similar to the scan method,
    // but uses getRSSI directly.
    uint16_t rssiMethod(size_t samples, uint16_t *result, size_t res_size);

    // detect method analyses result, and produces filtered_result, marking
    // those values that represent a detection event.
    // It returns index that represents strongest signal at which a detection event
    // occurred.
    Event detect(uint16_t *result, bool *filtered_result, size_t result_size,
                 int samples);

    size_t addEventListener(EventType t, Listener &l);
    void fireEvent(Event &e);
};

// Remove reading without neighbors
#define FILTER_SPECTRUM_RESULTS true

#endif
