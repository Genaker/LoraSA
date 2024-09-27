#ifndef LORASA_CORE_CPP
#define LORASA_CORE_CPP

#include "scan.h"
#include <cstdint>
#include <cstring>
#include <stdlib.h>

uint16_t Scan::rssiMethod(size_t samples, uint16_t *result, size_t res_size)
{
    float scale((float)res_size / (HI_RSSI_THRESHOLD - LO_RSSI_THRESHOLD + 0.1));

    memset(result, 0, res_size * sizeof(uint16_t));
    int result_index = 0;

    //
    uint16_t max_signal = 65535;
    // N of samples
    for (int r = 0; r < samples; r++)
    {
        float rssi = getRSSI();
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

size_t Scan::detect(uint16_t *result, bool *filtered_result, size_t result_size,
                    int samples)
{
    size_t max_rssi_x = 999;

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

    return max_rssi_x;
}

#endif
