#ifndef LORASA_CORE_CPP
#define LORASA_CORE_CPP

#include "scan.h"
#include <cstdint>
#include <cstring>
#include <events.h>
#include <stdlib.h>

uint16_t Scan::rssiMethod(float (*getRSSI)(void *), void *param, size_t samples,
                          uint16_t *result, size_t res_size)
{
    float scale((float)res_size / (HI_RSSI_THRESHOLD - LO_RSSI_THRESHOLD + 0.1));

    memset(result, 0, res_size * sizeof(uint16_t));
    int result_index = 0;

    //
    uint16_t max_signal = 65535;
    // N of samples
    for (int r = 0; r < samples; r++)
    {
        float rssi = getRSSI(param);
        if (rssi < -65535)
            rssi = -65535;

        uint16_t abs_rssi = abs(rssi);
        if (abs_rssi < max_signal)
        {
            max_signal = abs_rssi;
        }
        // ToDO: check if 4 is correct value for 33 power bins
        // Now we have more space because we are ignoring low dB values
        // we can  / 3 default 4
        if (RSSI_OUTPUT_FORMULA == 1)
        {
            result_index =
                /// still not clear formula but it works
                uint8_t(abs(rssi) / 4);
        }
        else if (RSSI_OUTPUT_FORMULA == 2)
        {
            if (rssi > HI_RSSI_THRESHOLD)
            {
                rssi = HI_RSSI_THRESHOLD;
            }
            else if (rssi < LO_RSSI_THRESHOLD)
            {
                rssi = LO_RSSI_THRESHOLD;
            }

            result_index = uint8_t((HI_RSSI_THRESHOLD - rssi) * scale);
        }

        if (result_index >= res_size)
        {
            // Maximum index possible
            result_index = res_size - 1;
        }

        LOG("RSSI: %f IDX: %d\n", rssi, result_index);
        if (result[result_index] == 0 || result[result_index] > abs_rssi)
        {
            result[result_index] = abs_rssi;
        }
    }

    return max_signal;
}

Event Scan::detect(uint16_t *result, bool *filtered_result, size_t result_size,
                   int samples)
{
    size_t max_rssi_x = result_size;

    for (int y = 0; y < result_size; y++)
    {

        LOG("%i:%i,", y, result[y]);
#if !defined(FILTER_SPECTRUM_RESULTS) || FILTER_SPECTRUM_RESULTS == false
        if (result[y] && result[y] != 0)
        {
            filtered_result[y] = 1;
        }
        else
        {
            filtered_result[y] = 0;
        }
#endif

// if samples low ~1 filter removes all values
#if FILTER_SPECTRUM_RESULTS

        filtered_result[y] = 0;
        // Filter Elements without neighbors
        // if RSSI method actual value is -xxx dB
        if (result[y] > 0 && samples > 1)
        {
            // do not process 'first' and 'last' row to avoid out of index
            // access.
            if ((y > 0) && (y < (result_size - 2)))
            {
                if (((result[y + 1] != 0) && (result[y + 2] != 0)) ||
                    (result[y - 1] != 0))
                {
                    filtered_result[y] = 1;
                    // Fill empty pixel
                    result[y + 1] = 1;
                }
                else
                {
                    LOG("Filtered::%i,", y);
                }
            }
        } // not filtering if samples == 1 because it will be filtered
        else if (result[y] > 0 && samples == 1)
        {
            filtered_result[y] = 1;
        }
#endif
        if (filtered_result[y] && max_rssi_x > y)
        {
            max_rssi_x = y;
        }
    }

    Event event(*this, EventType::DETECTED, 0);
    event.epoch = epoch;
    event.detected.detected = max_rssi_x < result_size;
    event.detected.freq = current_frequency;
    event.detected.rssi =
        event.detected.detected ? -(float)result[max_rssi_x] : LO_RSSI_THRESHOLD;
    event.detected.detected_at = max_rssi_x;
    event.detected.trigger =
        event.detected.detected && event.detected.rssi >= trigger_level;
    detection_count++;

    return event;
}

size_t Scan::addEventListener(EventType t, Listener &l)
{
    size_t c = listener_count[(size_t)t];
    Listener **new_list = new Listener *[c + 1];
    new_list[c] = &l;
    listener_count[(size_t)t] = c + 1;

    if (c > 0)
    {
        Listener **old_list = eventListeners[(size_t)t];
        memcpy(new_list, old_list, c * sizeof(Listener *));
        delete[] old_list;
    }

    eventListeners[(size_t)t] = new_list;
    return c;
}

struct CallbackFunction : Listener
{
    void (*cb)(void *arg, Event &e);
    void *arg;

    CallbackFunction(void cb(void *arg, Event &e), void *arg) : cb(cb), arg(arg) {}

    void onEvent(Event &e) { cb(arg, e); }
};

size_t Scan::addEventListener(EventType t, void cb(void *arg, Event &e), void *arg)
{
    return addEventListener(t, *(new CallbackFunction(cb, arg)));
}

void Scan::fireEvent(Event &event)
{
    Listener **list = eventListeners[(size_t)event.type];
    size_t c = listener_count[(size_t)event.type];
    for (int i = 0; i < c; i++)
    {
        list[i]->onEvent(event);
    }

    list = eventListeners[(size_t)EventType::ALL_EVENTS];
    c = listener_count[(size_t)EventType::ALL_EVENTS];
    for (int i = 0; i < c; i++)
    {
        list[i]->onEvent(event);
    }
}

#endif
