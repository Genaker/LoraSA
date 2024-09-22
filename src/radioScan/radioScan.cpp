#ifndef LORASA_CORE_CPP
#define LORASA_CORE_CPP

#include "radioScan.h"
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

#endif
