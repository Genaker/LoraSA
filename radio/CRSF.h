#include <Arduino.h>

class CRSF
{
  public:
    CRSF(HardwareSerial &serialPort, int txPin, int rxPin, long baudRate = 420000)
        : serialPort(serialPort), txPin(txPin), rxPin(rxPin), baudRate(baudRate)
    {
    }

    void begin() { serialPort.begin(baudRate, SERIAL_8N1, rxPin, txPin); }

    void setChannels(const uint16_t *channels, size_t numChannels)
    {
        const uint8_t DEVICE_ADDRESS = 0xC8;    // Transmitter address
        const uint8_t TYPE_CHANNEL_DATA = 0x16; // Channel data type
        const size_t PAYLOAD_SIZE =
            (numChannels * 11 + 7) / 8; // Calculate payload size for 11 bits per channel
        const size_t PACKET_SIZE =
            4 + PAYLOAD_SIZE; // Address, type, length, payload, CRC

        uint8_t packet[PACKET_SIZE];
        packet[0] = DEVICE_ADDRESS;
        packet[1] = PAYLOAD_SIZE + 2; // Length includes type and CRC
        packet[2] = TYPE_CHANNEL_DATA;

        // Pack the channel data into the payload
        uint8_t bitsMerged = 0;
        uint32_t readValue = 0;
        unsigned writeIndex = 3;
        for (size_t i = 0; i < numChannels; ++i)
        {
            readValue |= ((uint32_t)channels[i] & 0x7FF) << bitsMerged;
            bitsMerged += 11;
            while (bitsMerged >= 8)
            {
                packet[writeIndex++] = readValue & 0xFF;
                readValue >>= 8;
                bitsMerged -= 8;
            }
        }
        if (bitsMerged > 0)
        {
            packet[writeIndex++] = readValue & 0xFF;
        }

        // Calculate CRC
        uint8_t crc = calculateCRC(packet, PACKET_SIZE - 1);
        packet[PACKET_SIZE - 1] = crc;

        // Send the packet
        serialPort.write(packet, PACKET_SIZE);
    }

  private:
    HardwareSerial &serialPort;
    int txPin;
    int rxPin;
    long baudRate;

    uint8_t calculateCRC(const uint8_t *data, size_t length)
    {
        uint8_t crc = 0;
        for (size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
        }
        return crc;
    }
};
