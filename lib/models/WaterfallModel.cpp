#include "models.h"
#include <algorithm>
using namespace std;

#include <cstring>

WaterfallModel::WaterfallModel(size_t w, uint64_t base_dt, size_t m_sz,
                               const size_t *multiples)
{
    width = w;

    size_t dt_sz = 0;
    for (int i = 0; i < m_sz; i++)
        dt_sz += multiples[i];

    buckets = dt_sz;
    dt = new uint64_t[buckets];
    events = new uint32_t *[buckets];
    counts = new uint32_t *[buckets];
    times = new uint64_t[buckets];

    uint64_t m = base_dt;
    for (int i = 0, j = 0; i < m_sz; i++)
    {
        for (int k = 0; k < multiples[i]; k++, j++)
        {
            dt[j] = m;

            events[j] = new uint32_t[width];
            counts[j] = new uint32_t[width];
        }

        m *= multiples[i];
    }
}

void WaterfallModel::reset(uint64_t t0, size_t w)
{
    if (w != width)
    {
        width = w;
        for (int i = 0; i < buckets; i++)
        {
            delete[] counts[i];
            delete[] events[i];

            counts[i] = new uint32_t[w];
            events[i] = new uint32_t[w];
        }
    }

    for (int i = 0; i < buckets; i++)
    {
        memset(counts[i], 0, width * sizeof(uint32_t));
        memset(events[i], 0, width * sizeof(uint32_t));
        times[i] = t0 + dt[i];
    }
}

/*
 * The model is literally a stack of counters:
 * - incomplete second
 * - n complete seconds
 * - incomplete minute
 * - n complete minutes
 * - ...
 *
 * updateModel updates incomplete second. When the second becomes complete, it
 * gets pushed to complete seconds, and the last complete second is rotated out
 * and it gets added to incomplete minute. This gets repeated for incomplete
 * minutes, etc.
 */
size_t WaterfallModel::updateModel(uint64_t t, size_t x, uint16_t y)
{
    size_t changed = 1;

    while (t > times[0])
    {
        changed = max(changed, push());
    }

    counts[0][x]++;
    events[0][x] += y;
    return changed;
}

size_t WaterfallModel::push()
{
    size_t i = 1;
    for (; i < buckets; i++)
    {
        if (dt[i - 1] == dt[i])
            continue;
        if (times[i - 1] <= times[i])
            break;
        times[i - 1] = times[i] + dt[i];
    }

    uint64_t t0 = times[0];

    uint32_t *cc = counts[i - 1];
    uint32_t *ee = events[i - 1];
    memmove(times + 1, times, (i - 1) * sizeof(uint64_t));
    memmove(counts + 1, counts, (i - 1) * sizeof(uint32_t *));
    memmove(events + 1, events, (i - 1) * sizeof(uint32_t *));

    if (i < buckets)
    {
        for (int j = 0; j < width; j++)
        {
            counts[i][j] += cc[j];
            events[i][j] += ee[j];
        }

        i++;
    }

    memset(cc, 0, width * sizeof(uint32_t));
    memset(ee, 0, width * sizeof(uint32_t));
    counts[0] = cc;
    events[0] = ee;
    times[0] = t0 + dt[0];

    return i;
}

#ifdef TO_STRING

#include <sstream>
#include <string>

#endif

char *WaterfallModel::toString()
{
#ifdef TO_STRING
    std::stringstream r;
    r << "w:" << width << " b:" << buckets << " [";

    for (int i = 0; i < buckets; i++)
    {
        r << "dt:" << dt[i] << " t:" << times[i] << " [";
        for (int j = 0; j < width; j++)
            r << " c:" << counts[i][j] << " e:" << events[i][j];
        r << " ]";
    }

    r << " ]";

    char *ret = new char[r.str().length() + 1];
    strncpy(ret, r.str().c_str(), r.str().length());
#else
    char *ret = NULL;
#endif
    return ret;
}
