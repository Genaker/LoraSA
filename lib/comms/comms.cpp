#ifdef SERIAL_OUT
#include "comms.h"
#include <config.h>

#include <HardwareSerial.h>
#include <USB.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

Comms *Comms0;

void _onReceive0()
{
    if (Comms0 == NULL)
    {
        return;
    }

    Comms0->_onReceive();
}

void _onUsbEvent0(void *arg, esp_event_base_t event_base, int32_t event_id,
                  void *event_data)
{
    if (event_base == ARDUINO_HW_CDC_EVENTS)
    {
        // arduino_hw_cdc_event_data_t *data = (arduino_hw_cdc_event_data_t *)event_data;
        if (event_id == ARDUINO_HW_CDC_RX_EVENT)
        {
            _onReceive0(/*data->rx.len*/);
        }
    }
}

bool Comms::initComms(Config &c)
{
    if (c.listen_on_usb.equalsIgnoreCase("readline"))
    {
        // comms using readline plaintext protocol
        Comms0 = new ReadlineComms(Serial);
        Serial.onEvent(ARDUINO_HW_CDC_RX_EVENT, _onUsbEvent0);
        Serial.begin();

        Serial.println("Initialized communications on Serial using readline protocol");

        return true;
    }
    else if (c.listen_on_serial0.equalsIgnoreCase("readline"))
    {
        // comms using readline plaintext protocol
        Comms0 = new ReadlineComms(Serial0);
        Serial0.onReceive(_onReceive0, false);

        Serial.println("Initialized communications on Serial0 using readline protocol");

        return true;
    }

    if (c.listen_on_serial0.equalsIgnoreCase("none"))
    {
        Comms0 = new NoopComms();

        Serial.println("Configured none - Initialized no communications");
        return false;
    }

    Comms0 = new NoopComms();
    Serial.println("Nothing is configured - initialized no communications");
    return false;
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

#define POLY 0x1021
uint16_t crc16(String v, uint16_t c)
{
    for (int i = 0; i < v.length(); i++)
    {
        uint16_t ch = v.charAt(i);
        c = c ^ (ch << 8);
        for (int j = 0; j < 8; j++)
        {
            if (c & 0x8000)
            {
                c = (c << 1) ^ POLY;
            }
            else
            {
                c <<= 1;
            }
        }
    }

    return c;
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
        break;
    case MessageType::SCAN_RESULT:
        p = _scan_result_str(m.payload.dump);
        break;
    }

    serial.print(_wrap_str(p));
    return true;
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

    Serial.println("ignoring unknown message " + p);
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
             ")";
    }

    return p + " ]\n";
}

String _wrap_str(String v)
{
    String r = String(v.length()) + "\n" + v;
    return "WRAP " + String(crc16(r, 0), 16) + " " + r;
}
#endif
