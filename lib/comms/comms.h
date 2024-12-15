#ifndef __COMMS_H
#define __COMMS_H

#ifdef SERIAL_OUT
#include <HardwareSerial.h>
#include <config.h>

enum MessageType
{
    WRAP = 0,
    SCAN,
    SCAN_RESULT,
    _MAX_MESSAGE_TYPE = SCAN_RESULT
};

struct Wrapper
{
    int32_t length;
    uint16_t crc;
};

struct ScanTask
{
    int64_t count;
    int64_t delay;
};

struct ScanTaskResult
{
    size_t sz;
    uint32_t *freqs_khz;
    int16_t *rssis;
};

struct Message
{
    MessageType type;
    union
    {
        Wrapper wrap;
        ScanTask scan;
        ScanTaskResult dump;
    } payload;
};

struct Comms
{
    Stream &serial;
    Message **received;
    size_t received_sz;
    size_t received_pos;

    Message *wrap;

    Comms(Stream &serial)
        : serial(serial), received(NULL), received_sz(0), received_pos(0), wrap(NULL) {};

    virtual size_t available();
    virtual bool send(Message &) = 0;
    virtual Message *receive();

    virtual void _onReceive() = 0;
    virtual bool _messageArrived(Message &);

    static bool initComms(Config &);
};

struct NoopComms : Comms
{
    NoopComms() : Comms(Serial0) {};

    virtual bool send(Message &) { return true; };
    virtual void _onReceive() {};
};

struct ReadlineComms : Comms
{
    String partialPacket;

    ReadlineComms(Stream &serial) : Comms(serial), partialPacket("") {};

    virtual bool send(Message &) override;

    virtual void _onReceive() override;
};

extern Comms *Comms0;

#endif
#endif
