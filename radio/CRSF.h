#pragma once

#include <Arduino.h>

//
// Standard Crossfire definitions:
//
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAME_TYPE_CHANNELS 0x16
#define CRSF_MAX_CHANNELS 16
#define CRSF_BAUDRATE 420000 // standard CRSF baud rate

// Betaflight CRSF docs:
// https://github.com/betaflight/betaflight/blob/4.5-maintenance/src/main/rx/crsf_protocol.h
// Crosfire Protocol specs:
// https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#broadcast-frame

// The CRSF "RC Channels" frame for 16 channels is always 24 bytes total:
//   Byte[0]   = 0xC8 (address)
//   Byte[1]   = 22   (length from byte[2] to the last CRC byte)
//   Byte[2]   = 0x16 (frame type for channels)
//   Byte[3..22] = 20 bytes of channel data (10 bits × 16 = 160 bits)
//   Byte[23]  = CRC
//
// If type=0x16 and length=22, we interpret it as channel data (16ch, 10 bits each).

class CRSF2
{
  public:
    // Pass in the HardwareSerial, plus the pins to use on ESP32
    // (rxPin and txPin). If you only do TX, you can set rxPin=-1 or vice versa.
    CRSF2(HardwareSerial &serialPort, int rxPin, int txPin, long baudRate = CRSF_BAUDRATE)
        : serialPort(serialPort), rxPin(rxPin), txPin(txPin), baudRate(baudRate)
    {
    }

    void begin()
    {
        // Initialize the UART. If you want two-way, specify both rxPin, txPin.
        // Example: 8N1, no inversion, 20s timeout
        serialPort.begin(baudRate, SERIAL_8N1, rxPin, txPin, false, 20000UL);
    }

    // -------------------------------
    // Send a 16-channel RC frame (10 bits per channel).
    // channelData[i] should be in the range [0..1023].
    // For typical servo-like values (1000..2000), you must scale them down if needed.
    // -------------------------------
    void sendRCFrame(const uint16_t *channelData)
    {
        constexpr size_t CHANNEL_BITS = 10;
        constexpr size_t CHANNEL_COUNT = CRSF_MAX_CHANNELS;
        constexpr size_t CHANNEL_BYTES =
            ((CHANNEL_COUNT * CHANNEL_BITS) + 7) / 8; // => 20
        constexpr size_t FRAME_TYPE_SIZE = 1;         // 1 byte for frame type (0x16)
        constexpr size_t CRC_SIZE = 1;                // 1 byte for CRC
        constexpr size_t CRSF_PAYLOAD_LEN =
            FRAME_TYPE_SIZE + CHANNEL_BYTES + CRC_SIZE;      // => 1 + 20 + 1 = 22
        constexpr size_t PACKET_SIZE = 2 + CRSF_PAYLOAD_LEN; // => 24 total

        uint8_t packet[PACKET_SIZE];

        // Address (sync) byte
        packet[0] = CRSF_SYNC_BYTE;
        // Length (22)
        packet[1] = CRSF_PAYLOAD_LEN;
        // Frame Type
        packet[2] = CRSF_FRAME_TYPE_CHANNELS;

        // Pack channel data (16 × 10 bits = 160 bits => 20 bytes)
        uint32_t bitBuffer = 0;
        uint8_t bitCount = 0;
        size_t byteIndex = 3; // start filling after [0..2]

        for (size_t i = 0; i < CHANNEL_COUNT; i++)
        {
            uint16_t val = channelData[i] & 0x03FF; // 10-bit mask
            bitBuffer |= (static_cast<uint32_t>(val) << bitCount);
            bitCount += CHANNEL_BITS; // +10

            while (bitCount >= 8)
            {
                packet[byteIndex++] = static_cast<uint8_t>(bitBuffer & 0xFF);
                bitBuffer >>= 8;
                bitCount -= 8;
            }
        }

        // If leftover bits remain, put them into the last byte
        if (bitCount > 0)
        {
            packet[byteIndex++] = static_cast<uint8_t>(bitBuffer & 0xFF);
        }

        // Compute CRC over [Type + channel bytes] => 21 bytes
        const size_t CRC_LENGTH = CRSF_PAYLOAD_LEN - 1; // (22 - 1) = 21
        uint8_t crc = calculateCRC(&packet[2], CRC_LENGTH);
        // Place CRC at last byte (index 23)
        packet[PACKET_SIZE - 1] = crc;

        // Send out the 24-byte CRSF RC frame
        serialPort.write(packet, PACKET_SIZE);
    }

    // -------------------------------
    // Poll the serial port for incoming CRSF frames.
    // If we receive a valid 16-ch "Channel Data" frame (type=0x16),
    // decode the channels into the provided channelData[] array.
    // Returns 'true' if a new channel frame was successfully parsed.
    //
    // Call this in your loop() or in a periodic task.
    // -------------------------------
    bool receiveRCFrame(uint16_t *channelData)
    {
        // Run our state machine as long as there's data available
        while (serialPort.available() > 0)
        {
            uint8_t c = static_cast<uint8_t>(serialPort.read());

            switch (rxState)
            {
            case WAIT_SYNC:
                if (c == CRSF_SYNC_BYTE)
                {
                    rxPacket[0] = c;
                    rxIndex = 1;
                    rxState = WAIT_LEN;
                }
                break;

            case WAIT_LEN:
                // The next byte after sync is "length"
                rxPacket[rxIndex++] = c;
                rxLength = c; // length should be the payload from [type..CRC]
                // Basic sanity check: max 64 or so. CRSF frames can vary, but let's be
                // safe.
                if (rxLength < 2 || rxLength > 64)
                {
                    // Invalid length, reset parser
                    rxState = WAIT_SYNC;
                    rxIndex = 0;
                }
                else
                {
                    // Next step: read length more bytes
                    rxState = WAIT_PAYLOAD;
                }
                break;

            case WAIT_PAYLOAD:
                // Store the byte
                rxPacket[rxIndex++] = c;

                // If we've reached the entire frame, parse
                if (rxIndex >= (2 + rxLength))
                {
                    // We have [0]=0xC8, [1]=length, plus 'length' bytes
                    // Check CRC, parse if valid
                    bool ok = parseFrame(channelData);
                    // Reset to look for the next frame
                    rxState = WAIT_SYNC;
                    rxIndex = 0;

                    if (ok)
                    {
                        // We got a valid channel frame
                        return true;
                    }
                }
                break;
            }
        }

        return false; // no new channel frame parsed
    }

  private:
    HardwareSerial &serialPort;
    int rxPin;
    int txPin;
    long baudRate;

    // ---- Receive State Machine Fields ----
    enum RxState
    {
        WAIT_SYNC,
        WAIT_LEN,
        WAIT_PAYLOAD
    } rxState = WAIT_SYNC;

    uint8_t rxPacket[64]; // buffer for incoming frame
    size_t rxIndex = 0;   // how many bytes we've stored
    uint8_t rxLength = 0; // length byte from the packet

    // 8-bit CRC with polynomial 0xD5
    uint8_t calculateCRC(const uint8_t *data, size_t length)
    {
        uint8_t crc = 0;
        for (size_t i = 0; i < length; i++)
        {
            crc ^= data[i];
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                if (crc & 0x80)
                {
                    crc <<= 1;
                    crc ^= 0xD5;
                }
                else
                {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }

    // Attempt to parse the frame in rxPacket[].
    // If it's a valid 16-ch RC frame (type=0x16) and CRC checks out,
    // decode into channelData[].
    // Returns true on success, false otherwise.
    bool parseFrame(uint16_t *channelData)
    {
        // Basic layout:
        // rxPacket[0] = 0xC8
        // rxPacket[1] = length (n)
        // Then we have n bytes: [2..(1+n)]
        // The last of those n bytes is CRC.

        uint8_t lengthField = rxPacket[1];
        // The "payload" = lengthField bytes from rxPacket[2]..rxPacket[1 + lengthField]
        // So the CRC is at rxPacket[1 + lengthField].
        uint8_t crc = rxPacket[1 + lengthField];

        // Check CRC over the [Frame Type..(payload)] excluding the CRC byte
        // i.e. from rxPacket[2]..rxPacket[1 + lengthField - 1]
        size_t dataLen = lengthField - 1; // e.g. if length=22, dataLen=21
        uint8_t calcCRC = calculateCRC(&rxPacket[2], dataLen);

        if (calcCRC != crc)
        {
            // CRC mismatch
            return false;
        }

        // Check frame type
        uint8_t frameType = rxPacket[2];
        if (frameType != CRSF_FRAME_TYPE_CHANNELS)
        {
            // Not a channel frame
            return false;
        }

        // For a 16-ch frame, lengthField should be 22
        // but let's check if we want to be sure
        if (lengthField != 22)
        {
            // not a 16-ch RC frame
            return false;
        }

        // If we reach here, we have a valid 16-ch frame
        // Decode the 20 bytes of channel data
        // They live at rxPacket[3..22], which is 20 bytes
        decodeChannels(&rxPacket[3], channelData);
        return true;
    }

    // Decode 16 channels of 10 bits from the 20-byte payload
    // into channelData[16].
    void decodeChannels(const uint8_t *payload, uint16_t *channelData)
    {
        constexpr size_t CHANNEL_COUNT = CRSF_MAX_CHANNELS; // 16
        constexpr size_t CHANNEL_BITS = 10;

        uint32_t bitBuffer = 0;
        uint8_t bitCount = 0;
        size_t bytePos = 0;

        for (size_t i = 0; i < CHANNEL_COUNT; i++)
        {
            // accumulate bits until we have at least 10
            while (bitCount < CHANNEL_BITS)
            {
                bitBuffer |= (static_cast<uint32_t>(payload[bytePos++]) << bitCount);
                bitCount += 8;
            }
            // extract 10 bits
            uint16_t val = static_cast<uint16_t>(bitBuffer & 0x3FF); // 10-bit mask
            bitBuffer >>= CHANNEL_BITS;
            bitCount -= CHANNEL_BITS;

            // store
            channelData[i] = val;
        }
    }
};
