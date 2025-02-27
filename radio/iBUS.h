// Ibus.h
#ifndef IBUS_H
#define IBUS_H

#include <HardwareSerial.h>

#define IBUS_BAUD_RATE 115200
#define IBUS_CHANNELS_COUNT 14
#define IBUS_PACKET_BYTES_COUNT ((IBUS_CHANNELS_COUNT * 2) + 4)

class Ibus2
{
  public:
    void begin(HardwareSerial &serial, int txPin);
    void sendCommands(uint16_t commands[IBUS_CHANNELS_COUNT]);

  private:
    HardwareSerial *serial;
    uint8_t packet[IBUS_PACKET_BYTES_COUNT];
    void createPacket(uint16_t commands[IBUS_CHANNELS_COUNT]);
};

#endif // IBUS_H
