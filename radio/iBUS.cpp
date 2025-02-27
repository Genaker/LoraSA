// Ibus.cpp
#include "Ibus.h"
#include <Arduino.h>

void Ibus2::begin(HardwareSerial &serial, int txPin)
{
    this->serial = &serial;
    this->serial->begin(IBUS_BAUD_RATE, SERIAL_8N1, -1, txPin); // Set custom TX pin
}

void Ibus2::createPacket(uint16_t commands[IBUS_CHANNELS_COUNT])
{
    packet[0] = 0x20; // Start byte
    packet[1] = 0x40; // Length byte
    uint_fast16_t checksum = 0xFFFF - 0x20 - 0x40;
    for (int i = 0; i < IBUS_CHANNELS_COUNT; i++)
    {
        packet[2 + i * 2] = commands[i] & 0xFF;        // Low byte
        packet[3 + i * 2] = (commands[i] >> 8) & 0xFF; // High byte
        checksum -= packet[2 + i * 2];
        checksum -= packet[3 + i * 2];
    }
    packet[IBUS_PACKET_BYTES_COUNT - 2] = lowByte(checksum);
    packet[IBUS_PACKET_BYTES_COUNT - 1] = highByte(checksum);
}

void Ibus2::sendCommands(uint16_t commands[IBUS_CHANNELS_COUNT])
{
    createPacket(commands);
    for (int i = 0; i < IBUS_PACKET_BYTES_COUNT; i++)
    {
        this->serial->write(packet[i]);
    }
}
