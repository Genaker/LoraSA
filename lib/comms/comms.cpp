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

void ReadlineComms::_onReceive()
{
    while (serial.available() > 0)
    {
        partialPacket = partialPacket + serial.readString();
        int i = partialPacket.indexOf('\n');
        while (i >= 0)
        {
            Message *m = _parsePacket(partialPacket.substring(0, i));
            if (m != NULL)
            {
                if (!_messageArrived(*m))
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

    const char *cstr = p.c_str();
    size_t cstr_len = strlen(cstr);

    int loops = 0;
    uint64_t t0 = millis();
    uint64_t idle_started = 0;

    for (size_t a = serial.availableForWrite(); a < cstr_len;
         a = serial.availableForWrite(), loops++)
    {
        uint64_t now = millis();
        if (now - t0 > 1000)
        {
            Serial.printf("Unable to make progress after %d loops; %d bytes available "
                          "for write, %d chars still to write\n",
                          loops, a, cstr_len);
            break;
        }

        if (a == 0)
        {
            if (idle_started == 0)
            {
                idle_started = now;
            }

            if (now - idle_started > 2)
            {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            else
            {
                yield();
            }
            continue;
        }

        idle_started = 0;

        serial.write(cstr, a);
        cstr += a;
        cstr_len -= a;
    }
    serial.println(cstr);

    uint64_t dt = millis() - t0;
    Serial.printf("Wrote stuff in %d iterations and %" PRIu64 " ms.\n", loops, dt);
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
    return "SCAN " + String(t.count) + " " + String(t.delay);
}

String _scan_result_str(ScanTaskResult &r)
{
    String p = "SCAN_RESULT " + String(r.sz) + " [";

    for (int i = 0; i < r.sz; i++)
    {
        p += (i == 0 ? "(" : ", (") + String(r.freqs_khz[i]) + ", " + String(r.rssis[i]) +
             ")";
    }

    return p + " ]";
}
