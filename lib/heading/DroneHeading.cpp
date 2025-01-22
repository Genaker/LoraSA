#include "heading.h"

void DroneHeading::setHeading(int64_t now, int16_t h)
{
    _heading = h;
    _lastRead = now;
}

int64_t DroneHeading::lastRead() { return _lastRead; }
int16_t DroneHeading::heading() { return _heading; }
