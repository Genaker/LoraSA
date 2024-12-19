#include "comms.h"

int16_t RadioComms::configureRadio()
{
    int16_t status =
        radio.begin(loraCfg.freq, loraCfg.bw, loraCfg.sf, loraCfg.cr, loraCfg.sync_word,
                    loraCfg.tx_power, loraCfg.preamble_len);
    if (status != RADIOLIB_ERR_NONE)
    {
        Serial.printf("could not configure Lora: %d\n", status);
        return status;
    }

    if (!loraCfg.crc)
    {
        status = radio.setCRC(0);
        if (status != RADIOLIB_ERR_NONE)
        {
            Serial.printf("could not configure CRC for Lora: %d\n", status);
            return status;
        }
    }

    return status;
}

size_t _write(uint8_t *m, size_t sz, size_t p, uint8_t v)
{
    if (p >= sz)
        return sz;
    m[p] = v;
    return p + 1;
}

size_t _write(uint8_t *m, size_t sz, size_t p, uint8_t *v, size_t v_sz)
{
    while (v_sz > 0)
    {
        p = _write(m, sz, p, *v);
        v++;
        v_sz--;
    }

    return p;
}

#define MAX_MSG 128
#define RSSI_HI -80
#define DETAIL_RSSIS 8
#define RSSI_LO (RSSI_HI - 1 - DETAIL_RSSIS)
int16_t RadioComms::send(Message &m)
{
    if (m.type != SCAN_RESULT)
    {
        return RADIOLIB_ERR_INVALID_FUNCTION;
    }

    uint8_t msg[MAX_MSG];
    size_t dump_sz = m.payload.dump.sz;
    size_t p = _write(msg, MAX_MSG, 0, (uint8_t)m.type);

    // first cut: dump the RSSI as-is
    // optimize the message size later
    p = _write(msg, MAX_MSG, p, (uint8_t *)&m.payload.dump.freqs_khz[0], 4);
    p = _write(msg, MAX_MSG, p, (uint8_t *)&m.payload.dump.freqs_khz[dump_sz - 1], 4);
    p = _write(msg, MAX_MSG, p, (uint8_t *)&dump_sz, 2);

    size_t rem = MAX_MSG - p;
    if (rem > dump_sz)
        rem = dump_sz;

    uint8_t bits = 0;
    size_t pp = p;
    for (int i = 0; i < dump_sz; i++)
    {
        if (i * rem / dump_sz > p - pp)
        {
            p = _write(msg, MAX_MSG, p, bits);
            bits = 0;
        }
        int16_t v = m.payload.dump.rssis[i];
        if (v >= 0)
        {
            v = 255;
        }
        else
        {
            v += 255;
            if (v < 0)
            {
                v = 0;
            }
        }

        bits = max(bits, (uint8_t)v);
    }

    if (dump_sz > 0)
    {
        p = _write(msg, MAX_MSG, p, bits);
    }

    return radio.transmit(msg, p);
}

size_t _read(uint8_t *buf, size_t sz, size_t p, uint8_t *v, size_t len)
{
    for (; p < sz && len > 0; v++, p++, len--)
    {
        *v = buf[p];
    }

    return p;
}

size_t _read(uint8_t *buf, size_t sz, size_t p, uint8_t *v)
{
    return _read(buf, sz, p, v, 1);
}

volatile bool _received = false;
void _rcv() { _received = true; }

Message *RadioComms::receive(uint16_t timeout_ms)
{
    uint8_t msg[MAX_MSG];

    // because of this, receive is single-threaded, single-device
    _received = false;
    radio.setDio1Action(_rcv);
    uint32_t timeout_ticks = (uint32_t)timeout_ms * (1000000 / 15625);

    int16_t status = radio.startReceive(timeout_ticks);
    if (status != RADIOLIB_ERR_NONE)
    {
        radio.clearDio1Action();
        Serial.printf("Failed to start receive: %d\n", status);
        return NULL;
    }

    // wait on a semaphore
    while (!_received)
    {
        yield();
    }
    radio.clearDio1Action();

    size_t len = radio.getPacketLength(true);
    uint8_t *packet = msg;

    if (len > MAX_MSG)
    {
        packet = new uint8_t[len];
    }
    status = radio.readData(packet, len);

    if (status == RADIOLIB_ERR_RX_TIMEOUT)
    {
        return NULL;
    }

    if (status != RADIOLIB_ERR_NONE || len == 0)
    {
        if (packet != msg)
            delete[] packet;

        Serial.printf("Failed to read data: E(%d), len == %d\n", status, len);
        return NULL;
    }

    if (len > MAX_MSG)
    {
        Serial.printf("Packet is longer than expected: %d\n", len);
    }

    size_t p;
    uint8_t b;
    p = _read(packet, len, 0, &b);

    Message *message = NULL;
    if (b == SCAN_RESULT)
    {
        message = new Message();
        message->type = SCAN_RESULT;

        uint32_t s, e;
        size_t dump_sz = 0;
        p = _read(packet, len, p, (uint8_t *)&s, 4);
        p = _read(packet, len, p, (uint8_t *)&e, 4);
        p = _read(packet, len, p, (uint8_t *)&dump_sz, 2);
        size_t rem = len - p;

        message->payload.dump.sz = dump_sz;
        if (dump_sz > 0)
        {
            message->payload.dump.rssis = new int16_t[dump_sz];
            message->payload.dump.freqs_khz = new uint32_t[dump_sz];
            message->payload.dump.freqs_khz[0] = s;
            message->payload.dump.freqs_khz[dump_sz - 1] = e;

            for (int i = 1; i < dump_sz - 1; i++)
            {
                uint32_t incr = (e - s) / (dump_sz - i);
                s += incr;
                message->payload.dump.freqs_khz[i] = s;
            }

            for (int i = 0, k = 0; i < rem; i++)
            {
                int j = (i + 1) * dump_sz / rem;
                int16_t rssi = 0;
                p = _read(packet, len, p, (uint8_t *)&rssi);
                rssi -= 255;
                for (; k < j; k++)
                {
                    message->payload.dump.rssis[k] = rssi;
                }
            }
        }
    }
    else
    {
        Serial.printf("Received message type %" PRIu8 ", length %d, but expecting %" PRIu8
                      " - ignoring\n",
                      b, len, (uint8_t)MessageType::SCAN_RESULT);
    }

    if (packet != msg)
    {
        delete[] packet;
    }

    return message;
}
