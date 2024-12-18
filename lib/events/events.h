#ifndef LORASA_EVENTS_H
#define LORASA_EVENTS_H

#include <cstdint>
#include <event_types.h>
#include <scan.h>

struct Event
{
    EventType type;
    uint64_t epoch;
    uint64_t time_ms;

    Scan &emitter;

    union
    {
        struct
        {
            float rssi;
            float freq;
            bool trigger;
            bool detected;
            size_t detected_at;
        } detected;
    };

    Event(Scan &emitter, EventType type, uint64_t time_ms)
        : emitter(emitter), type(type), epoch(emitter.epoch), time_ms(time_ms) {};
};

struct Listener
{
    virtual void onEvent(Event &event) = 0;
};

#endif
