#include "config.h"
#include <Arduino.h>

#define SBUS_CHANNELS 16
#define SBUS_PACKET_SIZE 25

class SBUS2
{
  public:
    SBUS2(HardwareSerial &serialPort) : sbusSerial(serialPort)
    {
        sbusSerial.begin(100000, SERIAL_8E2, -1, TXD1, true); // Inverted SBUS signal
    }

    void send(uint16_t channels[SBUS_CHANNELS])
    {
        uint8_t sbusPacket[SBUS_PACKET_SIZE] = {0};

        sbusPacket[0] = 0x0F; // SBUS Start Byte

        // Correctly pack 16 channels (11-bit each) into 22 bytes
        uint16_t packedData[SBUS_CHANNELS] = {0};
        for (int i = 0; i < SBUS_CHANNELS; i++)
        {
            packedData[i] = channels[i] & 0x07FF; // Ensure 11-bit limit
        }

        sbusPacket[1] = (packedData[0] & 0xFF);
        sbusPacket[2] = ((packedData[0] >> 8) | (packedData[1] << 3)) & 0xFF;
        sbusPacket[3] = ((packedData[1] >> 5) | (packedData[2] << 6)) & 0xFF;
        sbusPacket[4] = (packedData[2] >> 2) & 0xFF;
        sbusPacket[5] = ((packedData[2] >> 10) | (packedData[3] << 1)) & 0xFF;
        sbusPacket[6] = ((packedData[3] >> 7) | (packedData[4] << 4)) & 0xFF;
        sbusPacket[7] = ((packedData[4] >> 4) | (packedData[5] << 7)) & 0xFF;
        sbusPacket[8] = (packedData[5] >> 1) & 0xFF;
        sbusPacket[9] = ((packedData[5] >> 9) | (packedData[6] << 2)) & 0xFF;
        sbusPacket[10] = ((packedData[6] >> 6) | (packedData[7] << 5)) & 0xFF;
        sbusPacket[11] = (packedData[7] >> 3) & 0xFF;
        sbusPacket[12] = (packedData[8] & 0xFF);
        sbusPacket[13] = ((packedData[8] >> 8) | (packedData[9] << 3)) & 0xFF;
        sbusPacket[14] = ((packedData[9] >> 5) | (packedData[10] << 6)) & 0xFF;
        sbusPacket[15] = (packedData[10] >> 2) & 0xFF;
        sbusPacket[16] = ((packedData[10] >> 10) | (packedData[11] << 1)) & 0xFF;
        sbusPacket[17] = ((packedData[11] >> 7) | (packedData[12] << 4)) & 0xFF;
        sbusPacket[18] = ((packedData[12] >> 4) | (packedData[13] << 7)) & 0xFF;
        sbusPacket[19] = (packedData[13] >> 1) & 0xFF;
        sbusPacket[20] = ((packedData[13] >> 9) | (packedData[14] << 2)) & 0xFF;
        sbusPacket[21] = ((packedData[14] >> 6) | (packedData[15] << 5)) & 0xFF;
        sbusPacket[22] = (packedData[15] >> 3) & 0xFF;

        // Flags: Failsafe & Frame Lost
        sbusPacket[23] = 0x00; // Set failsafe/lost frame bits here if needed

        // End byte
        sbusPacket[24] = 0x00; // Must be 0x00 or 0x04 if failsafe active

        // Send SBUS packet
        sbusSerial.write(sbusPacket, SBUS_PACKET_SIZE);
    }

  private:
    HardwareSerial &sbusSerial;
};
