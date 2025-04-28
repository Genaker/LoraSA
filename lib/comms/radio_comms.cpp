#include "comms.h"

int packetRssi = 0;

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

uint8_t *_serialize_scan_result(Message &m, size_t &p, uint8_t *msg)
{
    if (m.type != SCAN_RESULT)
    {
        return NULL;
    }

    size_t dump_sz = m.payload.dump.sz;
    size_t max_msg = p;
    p = _write(msg, max_msg, 0, (uint8_t)m.type);

    // first cut: dump the RSSI as-is
    // optimize the message size later
    p = _write(msg, max_msg, p, (uint8_t *)&dump_sz, 2);
    p = _write(msg, max_msg, p, (uint8_t *)&m.payload.dump.freqs_khz[0], 4);
    p = _write(msg, max_msg, p, (uint8_t *)&m.payload.dump.freqs_khz[dump_sz - 1], 4);

    size_t rem = max_msg - p;
    if (rem > dump_sz)
        rem = dump_sz;

    uint8_t bits = 0;
    size_t pp = p;
    for (int i = 0; i < dump_sz; i++)
    {
        if (i * rem / dump_sz > p - pp)
        {
            p = _write(msg, max_msg, p, bits);
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
        p = _write(msg, max_msg, p, bits);
    }

    return msg;
}

uint8_t *_serialize_scan_max_result(Message &m, size_t &p, uint8_t *msg)
{
    if (m.type != SCAN_MAX_RESULT)
    {
        return NULL;
    }

    size_t dump_sz = m.payload.dump.sz;
    size_t max_msg = p;
    p = _write(msg, max_msg, 0, (uint8_t)m.type);
    p = _write(msg, max_msg, p, (uint8_t *)&dump_sz, 2);

    int16_t b = SCAN_MAX_RESULT_KHZ_SCALE; // scale to fit khz into 2 bytes
    p = _write(msg, max_msg, p, (uint8_t)b);

    for (int i = 0; i < dump_sz; i++)
    {
        b = m.payload.dump.freqs_khz[i] / SCAN_MAX_RESULT_KHZ_SCALE;
        p = _write(msg, max_msg, p, (uint8_t *)&b, 2);
        b = m.payload.dump.rssis[i];
        if (b >= 0)
        {
            b = 255;
        }
        else
        {
            b += 255;
            if (b < 0)
            {
                b = 0;
            }
        }
        p = _write(msg, max_msg, p, (uint8_t)b);
    }

    return msg;
}

uint8_t *_serialize_scan_heading_max(Message &m, size_t &p, uint8_t *msg)
{
    if (m.type != SCAN_HEADING_MAX)
    {
        return NULL;
    }

    size_t written = 0;
    written = _write(msg, p, 0, (uint8_t *)&m.payload.dump.heading_min, 2);
    written = _write(msg, p, written, (uint8_t *)&m.payload.dump.heading_max, 2);

    uint8_t *sub_msg = msg + written;
    p -= written;
    m.type = SCAN_MAX_RESULT;
    sub_msg = _serialize_scan_max_result(m, p, sub_msg);
    p += written;
    m.type = SCAN_HEADING_MAX;

    if (sub_msg == NULL)
    {
        msg = NULL;
    }

    return msg;
}

uint8_t *_serialize_config_task(Message &m, size_t &p, uint8_t *msg)
{
    if (m.type != CONFIG_TASK)
    {
        return NULL;
    }
    size_t max_msg = p;
    ConfigTaskType ctt = m.payload.config.task_type;
    p = _write(msg, max_msg, 0, (uint8_t)m.type);
    p = _write(msg, max_msg, p, (uint8_t)ctt);

    int key_len = m.payload.config.key->length();
    if (max_msg - p < key_len + 1)
    {
        return NULL;
    }

    p = _write(msg, max_msg, p, (uint8_t)key_len);
    p = _write(msg, max_msg, p, (uint8_t *)m.payload.config.key->c_str(), key_len);

    if (ctt == GET || ctt == SET_FAIL)
    {
        return msg;
    }

    int v_len = m.payload.config.value->length();

    if (max_msg - p < v_len + 1)
    {
        return NULL;
    }

    p = _write(msg, max_msg, p, (uint8_t)v_len);
    p = _write(msg, max_msg, p, (uint8_t *)m.payload.config.value->c_str(), v_len);

    return msg;
}

int16_t RadioComms::send(Message &m)
{
    uint8_t msg_buf[MAX_MSG];
    size_t p = MAX_MSG;
    uint8_t *msg = NULL;

    if (loraCfg.crc)
    {
        p -= 2;
    }

    if (m.type == SCAN_RESULT)
    {
        msg = _serialize_scan_result(m, p, msg_buf);
    }
    else if (m.type == MessageType::SCAN_MAX_RESULT)
    {
        msg = _serialize_scan_max_result(m, p, msg_buf);
    }
    else if (m.type == MessageType::SCAN_HEADING_MAX)
    {
        msg = _serialize_scan_heading_max(m, p, msg_buf);
    }
    else if (m.type == MessageType::CONFIG_TASK)
    {
        msg = _serialize_config_task(m, p, msg_buf);
    }

    if (msg == NULL)
    {
        Serial.printf("Failed to encode message\n");
        return RADIOLIB_ERR_INVALID_FUNCTION;
    }

    if (loraCfg.crc)
    {
        uint16_t c = loraCfg.crc_seed ^ 0xffff;
        c = crc16(loraCfg.crc_poly, c, p, msg);
        _write(msg, MAX_MSG, p, (uint8_t *)&c, 2);
    }

    int16_t status = radio.transmit(msg, p);

    if (msg != msg_buf)
    {
        delete[] msg;
    }

    return status;
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

Message *_deserialize_scan_result(size_t len, size_t &p, uint8_t *packet)
{
    Message *message = new Message();
    message->type = SCAN_RESULT;

    uint32_t s, e;
    size_t dump_sz = 0;
    p = _read(packet, len, p, (uint8_t *)&dump_sz, 2);
    p = _read(packet, len, p, (uint8_t *)&s, 4);
    p = _read(packet, len, p, (uint8_t *)&e, 4);
    size_t rem = len - p;

    message->payload.dump.sz = dump_sz;
    message->payload.dump.prssi = packetRssi;
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

    return message;
}

Message *_deserialize_scan_max_result(size_t len, size_t &p, uint8_t *packet)
{
    Message *message = new Message();
    message->type = SCAN_MAX_RESULT;

    uint32_t b = 0;
    size_t dump_sz = 0;
    p = _read(packet, len, p, (uint8_t *)&dump_sz, 2);
    uint32_t *freqs = new uint32_t[dump_sz];
    int16_t *rssis = new int16_t[dump_sz];
    message->payload.dump.sz = dump_sz;
    message->payload.dump.freqs_khz = freqs;
    message->payload.dump.rssis = rssis;
    message->payload.dump.rssis2 = NULL;

    uint32_t scale = 0;
    p = _read(packet, len, p, (uint8_t *)&scale);

    for (int i = 0; i < dump_sz; i++)
    {
        p = _read(packet, len, p, (uint8_t *)&b, 2);
        freqs[i] = scale * b;

        b = 0;
        p = _read(packet, len, p, (uint8_t *)&b);
        rssis[i] = ((int16_t)b) - 255;
    }

    return message;
}

Message *_deserialize_scan_heading_max(size_t len, size_t &p, uint8_t *packet)
{
    int16_t heading_min = -999;
    int16_t heading_max = 0;
    p = _read(packet, len, p, (uint8_t *)&heading_min, 2);
    p = _read(packet, len, p, (uint8_t *)&heading_max, 2);

    Message *message = _deserialize_scan_max_result(len, p, packet);

    if (message != NULL)
    {
        message->type = SCAN_HEADING_MAX;
        message->payload.dump.heading_min = heading_min;
        message->payload.dump.heading_max = heading_max;
    }
    return message;
}

Message *_deserialize_config_task(size_t len, size_t &p, uint8_t *packet)
{
    Message *message = new Message();
    message->type = CONFIG_TASK;

    ConfigTaskType ctt = GET;
    p = _read(packet, len, p, (uint8_t *)&ctt);
    message->payload.config.task_type = ctt;

    size_t key_len = 0;
    size_t p1 = _read(packet, len, p, (uint8_t *)&key_len);
    memmove(packet + p, packet + p + 1, key_len);
    packet[p + key_len] = 0;
    String *key = new String((char *)packet + p);
    message->payload.config.key = key;

    p = p1 + key_len;

    if (key->length() != key_len)
    {
        delete message;
        return NULL;
    }

    if (ctt == GET || ctt == SET_FAIL)
    {
        return message;
    }

    size_t v_len = 0;
    p1 = _read(packet, len, p, (uint8_t *)&v_len);
    memmove(packet + p, packet + p + 1, v_len);
    packet[p + v_len] = 0;
    String *value = new String((char *)packet + p);
    message->payload.config.value = value;

    p = p1 + v_len;

    if (value->length() != v_len)
    {
        delete message;
        return NULL;
    }

    return message;
}

volatile bool _received = false;
void _rcv() { _received = true; }

Message *RadioComms::receive(uint16_t timeout_ms)
{
    uint8_t msg[MAX_MSG];

#if defined(USING_LR1121) || defined(USING_SX1276)
    Message *message = NULL;
#warning Radio Comms not fully supported for LR1121 or SX1276
#else
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

    packetRssi = radio.getRSSI(true);
    // Serial.println("LORA_RSSI:" + String(packetRssi));
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
        message = _deserialize_scan_result(len, p, packet);
    }
    else if (b == SCAN_MAX_RESULT)
    {
        message = _deserialize_scan_max_result(len, p, packet);
    }
    else if (b == SCAN_HEADING_MAX)
    {
        message = _deserialize_scan_heading_max(len, p, packet);
    }
    else if (b == CONFIG_TASK)
    {
        message = _deserialize_config_task(len, p, packet);
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
#endif
    return message;
}
