#include "heading.h"

void DroneHeading::setHeading(int64_t now, int16_t h)
{
    _heading = h;
    _lastRead = now;
}

int64_t DroneHeading::lastRead() { return _lastRead; }
int16_t DroneHeading::heading() { return _heading; }

int16_t meanHeading(int16_t h1, int16_t h2)
{
    while (h1 < 0)
    {
        h1 += 360;
    }

    while (h2 < 0)
    {
        h2 += 360;
    }

    h1 = h1 % 360;
    h2 = h2 % 360;

    if (h1 > h2)
    {
        h1 += h2;
        h2 = h1 - h2;
        h1 -= h2;
    }
    // h1 is min, h2 is max

    if (h2 - h1 > 180)
    {
        h1 += 360;
    }

    return (h1 + h2) / 2;
}
