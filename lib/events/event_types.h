#ifndef LORASA_EVENT_TYPES_H
#define LORASA_EVENT_TYPES_H

struct Event;
enum EventType
{
    ALL_EVENTS = 0, // used only at registration time
    DETECTED,
    SCAN_TASK_COMPLETE,
    _MAX_EVENT_TYPE = SCAN_TASK_COMPLETE // unused as event type
};
struct Listener;
#endif
