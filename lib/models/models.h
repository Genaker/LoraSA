#ifndef CHARTS_MODELS_H
#define CHARTS_MODELS_H

#include <cstdint>
#include <cstring>
#include <stdlib.h>

struct WaterfallModel
{
    uint32_t **events;
    uint32_t **counts;
    uint64_t *times;
    uint64_t *dt;

    size_t buckets;
    size_t width;

    WaterfallModel(size_t w, uint64_t base_dt, size_t m_sz, const size_t *multiples);

    void reset(uint64_t t0, size_t width);
    size_t updateModel(uint64_t t, size_t x, uint16_t y);
    size_t push();

    char *toString();
};

#endif
