#include <Arduino.h>
#include <HardwareSerial.h>
// Define constants for iBUS
#define IBUS_BAUD_RATE 115200
#define IBUS_SEND_INTERVAL_MS 20
#define IBUS_CHANNELS_COUNT 14
#define IBUS_PACKET_BYTES_COUNT ((IBUS_CHANNELS_COUNT * 2) + 4)

// Define the custom TX pin
#define CUSTOM_TX_PIN 17 // Change this to your desired TX pin

// Define the Ibus class
class Ibus
{
  public:
    void begin(HardwareSerial &serial, int txPin);
    void loop();
    void enable();
    void disable();
    void readLoop();
    void sendPacket();
    bool unpackIbusData(uint8_t *packet);
    void setControlValue(uint8_t channel, uint8_t value);
    void setControlValuesList(uint8_t list[IBUS_CHANNELS_COUNT * 2]);

  private:
    HardwareSerial *serial;
    uint8_t controlValuesList[IBUS_CHANNELS_COUNT * 2] = {0};
    bool isEnabled = false;
    unsigned long previousMillis = 0;
    unsigned long currentMillis = 0;

    uint8_t *createPacket();
};

// Implement the Ibus methods
void Ibus::begin(HardwareSerial &serial, int txPin)
{
    this->serial = &serial;
    this->serial->begin(IBUS_BAUD_RATE, SERIAL_8N1, -1, txPin); // Set custom TX pin
}

void Ibus::loop()
{
    if (this->isEnabled)
    {
        this->sendPacket();
        // this->readLoop();
    }
}

uint8_t *Ibus::createPacket()
{
    static uint8_t packetBytesList[IBUS_PACKET_BYTES_COUNT];
    packetBytesList[0] = 0x20;
    packetBytesList[1] = 0x40;
    uint_fast16_t checksum = 0xFFFF - 0x20 - 0x40;
    for (size_t i = 2; i < (IBUS_CHANNELS_COUNT * 2) + 2; i++)
    {
        packetBytesList[i] = this->controlValuesList[i - 2];
        checksum -= packetBytesList[i];
    }
    packetBytesList[IBUS_PACKET_BYTES_COUNT - 2] = lowByte(checksum);
    packetBytesList[IBUS_PACKET_BYTES_COUNT - 1] = highByte(checksum);
    return packetBytesList;
}

void Ibus::sendPacket()
{
    if (this->isEnabled)
    {
        uint8_t *packetBytesList = this->createPacket();
        for (size_t i = 0; i < IBUS_PACKET_BYTES_COUNT; i++)
        {
            this->serial->write(packetBytesList[i]);
        }
    }
}

void Ibus::enable() { this->isEnabled = true; }

void Ibus::disable() { this->isEnabled = false; }

void Ibus::setControlValuesList(uint8_t list[IBUS_CHANNELS_COUNT * 2])
{
    for (size_t i = 0; i < (IBUS_CHANNELS_COUNT * 2); i++)
    {
        this->controlValuesList[i] = list[i];
    }
}

void Ibus::setControlValue(uint8_t channel, uint8_t value)
{
    this->controlValuesList[channel] = value;
}

bool Ibus::unpackIbusData(uint8_t *packet)
{
    // Verify start and length bytes
    if (packet[0] != 0x20 || packet[1] != 0x40)
    {
        return false; // Invalid packet
    }

    // Calculate checksum
    uint_fast16_t checksum = 0xFFFF;
    for (int i = 0; i < IBUS_PACKET_BYTES_COUNT - 2; i++)
    {
        checksum -= packet[i];
    }

    // Verify checksum
    uint_fast16_t receivedChecksum =
        packet[IBUS_PACKET_BYTES_COUNT - 2] | (packet[IBUS_PACKET_BYTES_COUNT - 1] << 8);
    if (checksum != receivedChecksum)
    {
        return false; // Checksum mismatch
    }

    // Extract channel values
    for (int i = 0; i < IBUS_CHANNELS_COUNT; i++)
    {
        this->controlValuesList[i] = packet[2 + i * 2] | (packet[3 + i * 2] << 8);
    }

    return true; // Successfully unpacked
}

void Ibus::readLoop()
{
    uint8_t ibusPacket[IBUS_PACKET_BYTES_COUNT];
    int packetIndex = 0;
    while (this->serial->available())
    {
        uint8_t byte = this->serial->read();

        // Store byte in packet buffer
        ibusPacket[packetIndex++] = byte;

        // Check if we have a full packet
        if (packetIndex == IBUS_PACKET_BYTES_COUNT)
        {
            if (unpackIbusData(ibusPacket))
            {
                Serial.println("iBUS Data Unpacked:");
                for (int i = 0; i < IBUS_CHANNELS_COUNT; i++)
                {
                    Serial.print("Channel ");
                    Serial.print(i + 1);
                    Serial.print(": ");
                    Serial.println(this->controlValuesList[i]);
                }
            }
            else
            {
                Serial.println("Failed to unpack iBUS data.");
            }
            packetIndex = 0; // Reset for next packet
        }
    }
}
