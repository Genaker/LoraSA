#include "comms.h"
#include <config.h>

#include <HardwareSerial.h>
#include <USB.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

Comms *HostComms;
Comms *Comms0 = NULL;
Comms *Comms1 = NULL;

RadioComms *RxComms = NULL;
RadioComms *TxComms = NULL;

void _onReceiveUsb(size_t len)
{
    if (HostComms == NULL)
    {
        return;
    }

    HostComms->_onReceive();
}

void _onReceive0()
{
    if (Comms0 == NULL)
    {
        return;
    }

    Comms0->_onReceive();
}

void _onReceive1()
{
    if (Comms1 == NULL)
    {
        return;
    }

    Comms1->_onReceive();
}

#if ARDUINO_USB_MODE
#define IF_CDC_EVENT(e, data)                                                            \
    arduino_hw_cdc_event_data_t *data = (arduino_hw_cdc_event_data_t *)event_data;       \
    if (event_base == ARDUINO_HW_CDC_EVENTS && event_id == ARDUINO_HW_CDC_##e)
#else
#define IF_CDC_EVENT(e, data)                                                            \
    arduino_usb_cdc_event_data_t *data = (arduino_usb_cdc_event_data_t *)event_data;     \
    if (event_base == ARDUINO_USB_CDC_EVENTS && event_id == ARDUINO_USB_CDC_##e)
#endif

void _onUsbEvent0(void *arg, esp_event_base_t event_base, int32_t event_id,
                  void *event_data)
{
    IF_CDC_EVENT(RX_EVENT, data) { _onReceiveUsb(data->rx.len); }
}

bool Comms::initComms(Config &c)
{
    bool fine = false;

#ifdef ARDUINO_USB_CDC_ON_BOOT
    if (c.listen_on_usb.equalsIgnoreCase("readline"))
    {
        // comms using readline plaintext protocol
        HostComms = new ReadlineComms("Host", Serial);
#if ARDUINO_USB_MODE
        // if Serial is HWCDC...
        Serial.onEvent(ARDUINO_HW_CDC_RX_EVENT, _onUsbEvent0);
#else
        // if Serial is USBCDC...
        Serial.onEvent(ARDUINO_USB_CDC_RX_EVENT, _onUsbEvent0);
#endif
        Serial.begin();

        Serial.println("Initialized communications on Serial using readline protocol");

        fine = true;
    }
#endif

    if (c.listen_on_serial0.equalsIgnoreCase("readline"))
    {
        // comms using readline plaintext protocol
        Comms0 = new ReadlineComms("UART0", Uart0);
        Uart0.onReceive(_onReceive0, false);

        Serial.println("Initialized communications on Serial0 using readline protocol");
    }
    else
    {
        Comms0 = new NoopComms();

        Serial.println("Configured none - Initialized no communications on Serial0");
    }

    if (c.listen_on_serial1.equalsIgnoreCase("readline"))
    {
        // comms using readline plaintext protocol
        Comms1 = new ReadlineComms("UART1", Uart1);
        Uart1.onReceive(_onReceive1, false);

        Serial.println("Initialized communications on Serial1 using readline protocol");
    }
    else
    {
        Comms1 = new NoopComms();

        Serial.println("Configured none - Initialized no communications on Serial1");
    }

    if (c.rx_lora != NULL)
    {
        RxComms = new RadioComms("RxComms", radio, *c.rx_lora);
    }

    if (c.tx_lora != NULL)
    {
        TxComms = new RadioComms("TxComms", radio, *c.tx_lora);
    }

    if (!fine)
    {
        HostComms = new NoopComms();
        Serial.println("Nothing is configured - initialized no communications");
    }
    return fine;
}

size_t Comms::available() { return received_pos; }

#define _RECEIVED_BUF_INCREMENT 10
#define _MAX_RECEIVED_SZ 100
bool Comms::_messageArrived(Message &m)
{
    if (received_pos == received_sz)
    {
        // TODO: if received_sz exceeds a configurable bound, complain and drop the
        // message on the floor
        if (received_sz >= _MAX_RECEIVED_SZ)
        {
            Serial.println("Receive buffer backlog too large; dropping the message");
            return false;
        }
        Message **m = new Message *[received_sz + _RECEIVED_BUF_INCREMENT];
        if (received_sz > 0)
        {
            memcpy(m, received, received_sz * sizeof(Message *));
            delete[] received;
        }
        received = m;
        received_sz += _RECEIVED_BUF_INCREMENT;
    }

    received[received_pos] = &m;
    received_pos++;

    return true;
}

Message *Comms::receive()
{
    if (received_pos == 0)
    {
        return NULL;
    }

    Message *m = received[0];
    received_pos--;

    memmove(received, received + 1, received_pos * sizeof(Message *));

    return m;
}

Message *_parsePacket(String);
String _scan_str(ScanTask &);
String _scan_result_str(ScanTaskResult &);
String _wrap_str(String);

uint16_t crc16(uint16_t poly, uint16_t c, size_t sz, uint8_t *v)
{
    c ^= 0xffff;
    for (int i = 0; i < sz; i++)
    {
        uint16_t ch = v[i];
        c = c ^ (ch << 8);
        for (int j = 0; j < 8; j++)
        {
            if (c & 0x8000)
            {
                c = (c << 1) ^ poly;
            }
            else
            {
                c <<= 1;
            }
        }
    }

    return c ^ 0xffff;
}

#define POLY 0x1021
uint16_t crc16(String v, uint16_t c)
{
    return crc16(POLY, c, v.length(), (uint8_t *)v.c_str());
}

void ReadlineComms::_onReceive()
{
    while (serial.available() > 0)
    {
        partialPacket = partialPacket + serial.readString();
        int i = partialPacket.indexOf('\n');
        while (i >= 0)
        {
            String pack = partialPacket.substring(0, i);
            bool messageOk = true;

            if (wrap != NULL)
            {
                messageOk = pack.length() == wrap->payload.wrap.length;
                if (messageOk)
                {
                    messageOk = crc16(pack, 0) == wrap->payload.wrap.crc;
                }
                delete wrap;
                wrap = NULL;
            }
            Message *m = messageOk ? _parsePacket(pack) : NULL;
            if (m != NULL)
            {
                if (m->type == WRAP)
                {
                    wrap = m;
                }
                else if (!_messageArrived(*m))
                {
                    delete m;
                }
            }
            else
            {
                Serial.println(name + ": discarding > " + pack);
            }
            partialPacket = partialPacket.substring(i + 1);
            i = partialPacket.indexOf('\n');
        }
    }
}

bool ReadlineComms::send(Message &m)
{
    String p;

    switch (m.type)
    {
    case MessageType::SCAN:
        p = _scan_str(m.payload.scan);
        Serial.println(name + ": the message is: " + p);
        break;
    case MessageType::SCAN_RESULT:
    case MessageType::SCAN_MAX_RESULT:
        p = _scan_result_str(m.payload.dump);
        break;
    case MessageType::CONFIG_TASK:
        ConfigTaskType ctt = m.payload.config.task_type;
        p = ctt == GET              ? "GET "
            : ctt == SET            ? "SET "
            : ctt == GETSET_SUCCESS ? "Success: "
                                    : "Failed to set: ";
        p += *m.payload.config.key;
        if (ctt == SET || ctt == GETSET_SUCCESS)
        {
            p += " " + *m.payload.config.value;
        }

        p += "\n";
        break;
    }

    serial.print(_wrap_str(p));
    return true;
}

String _stringParam(String &p, String default_v)
{
    p.trim();
    int i = p.indexOf(' ');
    if (i < 0)
    {
        i = p.length();
    }

    String v = p.substring(0, i);
    p = p.substring(i + 1);

    if (i == 0)
    {
        v = default_v;
    }
    return v;
}

int64_t _intParam(String &p, int64_t default_v)
{
    p.trim();
    int i = p.indexOf(' ');
    if (i < 0)
    {
        i = p.length();
    }

    int64_t v = p.substring(0, i).toInt();
    p = p.substring(i + 1);
    return v;
}

int64_t _hexParam(String &p, int64_t default_v)
{
    p.trim();
    int i = p.indexOf(' ');
    if (i < 0)
    {
        i = p.length();
    }

    int64_t v = strtol(p.substring(0, i).c_str(), 0, 16);
    p = p.substring(i + 1);
    return v;
}

Message *_parsePacket(String p)
{
    p.trim();
    if (p.length() == 0)
    {
        return NULL;
    }

    String cmd = p;
    int i = p.indexOf(' ');
    if (i < 0)
    {
        p = "";
    }
    else
    {
        cmd = p.substring(0, i);
        p = p.substring(i + 1);
        p.trim();
    }

    if (cmd.equalsIgnoreCase("wrap"))
    {
        Message *m = new Message();
        m->type = MessageType::WRAP;
        m->payload.wrap.crc = _hexParam(p, -1);
        m->payload.wrap.length = _intParam(p, -1);
        return m;
    }

    if (cmd.equalsIgnoreCase("scan"))
    {
        Message *m = new Message();
        m->type = MessageType::SCAN;
        m->payload.scan.count = _intParam(p, 1);
        m->payload.scan.delay = _intParam(p, -1);
        return m;
    }

    if (cmd.equalsIgnoreCase("get"))
    {
        Message *m = new Message();
        m->type = MessageType::CONFIG_TASK;
        m->payload.config.task_type = GET;
        m->payload.config.key = new String(_stringParam(p, ""));
        m->payload.config.value = NULL;
        return m;
    }

    if (cmd.equalsIgnoreCase("set"))
    {
        Message *m = new Message();
        m->type = MessageType::CONFIG_TASK;
        m->payload.config.task_type = SET;
        m->payload.config.key = new String(_stringParam(p, ""));
        m->payload.config.value = new String(_stringParam(p, ""));

        return m;
    }

    if (cmd.equalsIgnoreCase("heading"))
    {
        Message *m = new Message();
        m->type = MessageType::HEADING;
        m->payload.heading.heading = _intParam(p, 0);
        return m;
    }

    return NULL;
}

String _scan_str(ScanTask &t)
{
    return "SCAN " + String(t.count) + " " + String(t.delay) + "\n";
}

String _scan_result_str(ScanTaskResult &r)
{
    String p = "SCAN_RESULT " + String(r.sz) + " [ ";

    for (int i = 0; i < r.sz; i++)
    {
        p += (i == 0 ? "(" : ", (") + String(r.freqs_khz[i]) + ", " + String(r.rssis[i]) +
             (r.rssis2 ? ", " + String(r.rssis2[i]) : "") + ")";
    }

    return p + " ]\n";
}

String _wrap_str(String v)
{
    String r = String(v.length()) + "\n" + v;
    return "WRAP " + String(crc16(r, 0), 16) + " " + r;
}

Message::~Message()
{
    if (type == SCAN_RESULT || type == SCAN_MAX_RESULT)
    {
        if (payload.dump.sz > 0)
        {
            delete[] payload.dump.freqs_khz;
            delete[] payload.dump.rssis;
            payload.dump.sz = 0;
        }

        return;
    }

    if (type == CONFIG_TASK)
    {
        delete payload.config.key;

        ConfigTaskType ctt = payload.config.task_type;
        if (ctt == GETSET_SUCCESS || ctt == SET)
        {
            delete payload.config.value;
        }

        return;
    }
}
