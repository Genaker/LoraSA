#ifndef __COMMS_H
#define __COMMS_H

#include <HardwareSerial.h>
#include <LoRaBoards.h>

#include <LiLyGo.h>
#include <config.h>

#ifndef ARDUINO_USB_CDC_ON_BOOT
#define SERIAL0 Serial
#else
#define SERIAL0 Serial0
#endif

#ifndef SCAN_MAX_RESULT_KHZ_SCALE
// kHz scale: round frequency, so it fits into 2 bytes
// 2500000 / 40 = 62500, scale 40 fits 2.5GHz into two bytes
#define SCAN_MAX_RESULT_KHZ_SCALE 40
#endif

enum MessageType
{
    WRAP = 0,
    SCAN,
    SCAN_RESULT,
    SCAN_MAX_RESULT,
    CONFIG_TASK,
    _MAX_MESSAGE_TYPE = CONFIG_TASK
};

enum ConfigTaskType
{
    GET = 0,
    SET,
    GETSET_SUCCESS,
    SET_FAIL,
    _MAX_CONFIG_TASK_TYPE = SET_FAIL
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
    int16_t prssi;
};

struct ConfigTask
{
    String *key;
    String *value;
    ConfigTaskType task_type;
};

struct Message
{
    MessageType type;
    union
    {
        Wrapper wrap;
        ConfigTask config;
        ScanTask scan;
        ScanTaskResult dump;
    } payload;

    ~Message();
};

struct Endpoint
{
    union
    {

        struct
        {
            uint8_t loop : 1, // self
                uart0 : 1, uart1 : 1,
                lora : 1, // rx or tx_lora, depending on is_host
                host : 1; // USB
        };
        uint8_t addr;
    };
};

struct RoutedMessage
{
    Endpoint from;
    Endpoint to;
    Message *message;
};

struct Comms
{
    String name;
    Stream &serial;
    Message **received;
    size_t received_sz;
    size_t received_pos;

    Message *wrap;

    Comms(String name, Stream &serial)
        : name(name), serial(serial), received(NULL), received_sz(0), received_pos(0),
          wrap(NULL) {};

    virtual size_t available();
    virtual bool send(Message &) = 0;
    virtual Message *receive();

    virtual void _onReceive() = 0;
    virtual bool _messageArrived(Message &);

    static bool initComms(Config &);
};

struct NoopComms : Comms
{
    NoopComms() : Comms("no-op", SERIAL0) {};

    virtual bool send(Message &) { return true; };
    virtual void _onReceive() {};
};

struct ReadlineComms : Comms
{
    String partialPacket;

    ReadlineComms(String name, Stream &serial)
        : Comms(name, serial), partialPacket("") {};

    virtual bool send(Message &) override;

    virtual void _onReceive() override;
};

extern Comms *HostComms;

extern Comms *Comms0;

extern Comms *Comms1;

struct RadioComms
{
    String name;
    RADIO_TYPE &radio;
    LoRaConfig &loraCfg;

    RadioComms(String name, RADIO_TYPE &radio, LoRaConfig &cfg)
        : name(name), radio(radio), loraCfg(cfg)
    {
    }

    Message **received;

    int16_t configureRadio();
    int16_t send(Message &);
    Message *receive(uint16_t timeout_ms);
};

extern RadioComms *RxComms;
extern RadioComms *TxComms;

#endif
