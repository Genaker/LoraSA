#include "SSD1306Wire.h"
#include <LiLyGo.h>
#include <LoRaBoards.h>
#include <RadioLib.h>

void onReceive();
void onReceiveFlag(void);
void forceRestartLoRa();

bool radioIsRX = false;
unsigned long lastPacketTime = 0;
long int packetN = 0;
int packetSave = 0;
bool packetReceived = false;
// Function to send 2-byte LoRa packet OR 4-byte LoRa packet
void sendLoRaRCPacket(uint8_t cmd1, uint8_t val1, uint8_t cmd2, uint8_t val2,
                      uint8_t cmd3 = 0, uint8_t val3 = 0, uint8_t cmd4 = 0,
                      uint8_t val4 = 0)
{
    cmd1 &= 0x0F;
    val1 &= 0x0F;
    cmd2 &= 0x0F;
    val2 &= 0x0F;
    int packetSize = LORA_DATA_BYTE;
    if (cmd3 != 0 && val3 != 0)
    {
        cmd3 &= 0x0F;
        val3 &= 0x0F;
        cmd4 &= 0x0F;
        val4 &= 0x0F;
        packetSize = 4;
    }

    Serial.printf("Sending LoRa Packet: CMD1=%d, VAL1=%d, CMD2=%d, VAL2=%d, ", cmd1, val1,
                  cmd2, val2);
    if (packetSize == 4)
    {
        Serial.printf("CMD3=%d, VAL3=%d, CMD4=%d, VAL4=%d, ", cmd3, val3, cmd4, val4);
    }
    Serial.println();

    uint8_t paddedData[packetSize] = {0}; // Initialize with zeros
    // P:2:5:4:10
    // 0010:0101:0100:1010
    // RX 11001110
    // 9546
    // 0010:0101:0100:1010
    uint16_t packet = (cmd1 << 12) | (val1 << 8) | (cmd2 << 4) | val2;
    uint16_t packet2 = 0;

    if (packetSize == 4)
    {
        packet2 = (cmd3 << 12) | (val3 << 8) | (cmd4 << 4) | val4;
    }

    packetSave = packet;
    if (packet2 != 0)
    {
        packetSave = (static_cast<uint32_t>(packet) << 16) | packet2;
    }

    uint8_t data[packetSize] = {0};
    data[0] = (packet >> 8) & 0xFF; // High byte
    data[1] = packet & 0xFF;        // Low byte

    if (packetSize == 4)
    {
        data[2] = (packet2 >> 8) & 0xFF; // High byte
        data[3] = packet2 & 0xFF;        // Low byte
    }

    int len = sizeof(data);

    Serial.println("Packet  length: " + String(len));
    Serial.print("Sending Packet: ");
    Serial.print(data[0], BIN);
    Serial.print(" ");
    Serial.print(data[1], BIN);
    if (packetSize == 4)
    {
        Serial.print(" ");
        Serial.print(data[2], BIN);
        Serial.print(" ");
        Serial.print(data[3], BIN);
    }
    Serial.println();

    // size_t dataSize = sizeof(data) / sizeof(data[0]);
    // memcpy(paddedData, data, min(dataSize, (size_t)LORA_DATA_BYTE));

    int status = radio.transmit(data, sizeof(data));
    if (status == RADIOLIB_ERR_NONE)
    {
        Serial.println("LoRa Packet Sent!");
    }
    else
    {
        Serial.println("LoRa Transmission Failed.");
    }
}

void sendLoRaPacket(uint8_t *packetData, int length)
{
    // Size of pointer here not an array count.
    // int length = sizeof(packetData);
    String packetStr = "";

    Serial.println("Packet  length: " + String(length));

    Serial.print("Sending Packet: ");
    for (size_t i = 0; i < length; i++)
    {
        Serial.print(packetData[i], BIN);
        Serial.print(" ");
        packetStr += String(packetData[i]);
    }
    Serial.println();
    display.println("P[" + String(length) + "]:" + packetStr);
    Serial.println("P[" + String(length) + "]:" + packetStr);

    int status = radio.transmit(packetData, length);
    if (status == RADIOLIB_ERR_NONE)
    {
        Serial.println("LoRa Packet Sent!");
    }
    else
    {
        Serial.println("LoRa Transmission Failed.");
    }
}

void onReceiveFlag(void) { packetReceived = true; }

void onReceive(void)
{
#if LORA_RX
    receivedPacketCounter++;
    size_t len = radio.getPacketLength(true);
    uint8_t data[len] = {0};
    Serial.println("[LoRa] onReceive(" + String(len) + ")");

#if DEBUG_RX
    Serial.println("[LoRa] onReceive");
    if (len > LORA_DATA_BYTE)
    {
        Serial.println("WARNING: Packet size is too large:" + String(len));
    }

    Serial.println("[LoRa] Length: " + String(len));
#endif
    int state = radio.readData(data, len);

    if (state == RADIOLIB_ERR_NONE)
    {
#if DEBUG_RX
        if (sizeof(data) != LORA_DATA_BYTE)
        {
            Serial.println("[LoRa] ERROR: Packet Length Mismatch!");
        }
#endif
        // Packet size 4 processing
        if (len == 8)
        {
            uint64_t seqNum = (static_cast<uint64_t>(data[0]) << 56) |
                              (static_cast<uint64_t>(data[1]) << 48) |
                              (static_cast<uint64_t>(data[2]) << 40) |
                              (static_cast<uint64_t>(data[3]) << 32) |
                              (static_cast<uint64_t>(data[4]) << 24) |
                              (static_cast<uint64_t>(data[5]) << 16) |
                              (static_cast<uint64_t>(data[6]) << 8) |
                              static_cast<uint64_t>(data[7]);
#if DEBUG

            Serial.println("Lost:" + String(seqNum) + "/" +
                           String(receivedPacketCounter) + ":" +
                           String(seqNum - receivedPacketCounter));
#endif
            int lost = seqNum - receivedPacketCounter;
            display.println("Lost:" + String(seqNum) + "/" +
                            String(receivedPacketCounter) + ":" + String(lost));
        }

        // Check if data is actually zero
        else if (data[0] == 0 && data[1] == 0)
        {
            int length = len;
#if DEBUG
            Serial.println("[LoRa] WARNING: Received 0 : 0. I don't know why");
            Serial.print("[LoRa Receiver] Packet length: ");

            Serial.println(String(length));

            // return; // Ignore this packet
            Serial.print("[LoRa Receiver] RSSI: ");

            Serial.print(radio.getRSSI());
            // display.println("0-0");

            Serial.print("[LoRa Receiver] SNR: ");
            Serial.print(radio.getSNR());
            Serial.println(" dB");
#endif
        }
        else if (len == 2 || len == 4) //** ToDo: process length 4 */)
        {
            int length = len;
#if DEBUG
            Serial.println("[LoRa Receiver] Packet Received!");
            Serial.print("[LoRa Receiver] Packet length: ");
            Serial.println(String(length));
            Serial.println("[LoRa Receiver] DATA: " + String(data[0]) + ":" +
                           String(data[1]));
#endif
            display.println("DATA[" + String(packetN) + "]: " + String(data[0]) + ":" +
                            String(data[1]));
            if (len == 4)
            {
                display.println("DATA-4[" + String(packetN) + "]: " + String(data[2]) +
                                ":" + String(data[3]));
            }
            packetN++;
            // Decode received data
            uint16_t receivedPacket = (data[0] << 8) | data[1];
            uint8_t cmd1 = (receivedPacket >> 12) & 0x0F;
            uint8_t val1 = (receivedPacket >> 8) & 0x0F;
            uint8_t cmd2 = (receivedPacket >> 4) & 0x0F;
            uint8_t val2 = receivedPacket & 0x0F;

            display.println("Data:" + String(cmd1) + ":" + String(val1) + ":" +
                            String(cmd2) + ":" + (val2));

            // If size is 4, decode another 16 bits
            uint8_t cmd3 = 0, val3 = 0, cmd4 = 0, val4 = 0;

            if (len == 4)
            {
                uint16_t receivedPacket2 = (data[2] << 8) | data[3];
                uint8_t cmd3 = (receivedPacket2 >> 12) & 0x0F;
                uint8_t val3 = (receivedPacket2 >> 8) & 0x0F;
                uint8_t cmd4 = (receivedPacket2 >> 4) & 0x0F;
                uint8_t val4 = receivedPacket2 & 0x0F;

                display.println("Data:" + String(cmd3) + ":" + String(val3) + ":" +
                                String(cmd4) + ":" + (val4));
            }

#if DEBUG
            // Print received data
            Serial.print("[LoRa Receiver] Data: ");
            Serial.print("CMD1=");
            Serial.print(cmd1);
            Serial.print(", VAL1=");
            Serial.print(val1);
            Serial.print(", CMD2=");
            Serial.print(cmd2);
            Serial.print(", VAL2=");
            Serial.println(val2);
            Serial.print("[LoRa Receiver] Data: ");

            // Print RSSI (Signal Strength)
            Serial.print("[LoRa Receiver] RSSI: ");
            Serial.print(radio.getRSSI());
            Serial.println(" dBm");

            // Print SNR (Signal-to-Noise Ratio)
            Serial.print("[LoRa Receiver] SNR: ");
            Serial.print(radio.getSNR());
            Serial.println(" dB");
#endif
            display.print("RSSI: " + String(radio.getRSSI()));
            display.println(" SNR: " + String(radio.getSNR()));
        }
    }
    else if (state == RADIOLIB_ERR_RX_TIMEOUT)
    {
        //  No packet received
        Serial.println("[LoRa Receiver] No packet received.");
    }
    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        // Packet received but corrupted
        Serial.println("[LoRa Receiver] Packet received but CRC mismatch!");
    }
    else
    {
        // Other error
        Serial.print("[LoRa Receiver] Receive failed, error code: ");
        Serial.println(state);
    }
    // Restart LoRa receiver
    // radio.implicitHeader(LORA_DATA_BYTE);
    radio.startReceive();
#endif
}

void forceRestartLoRa()
{
    Serial.println("[LoRa] Forcing Restart...");
    radio.standby();
    radio.reset();
    delay(100);
    radio.begin();
    radio.startReceive();
    lastPacketTime = millis(); // Reset timeout
}

void initRadioSwitchTable()
{
#ifdef USING_LR1121 // USING_SX1262
    // LR1121
    // set RF switch configuration for Wio WM1110
    // Wio WM1110 uses DIO5 and DIO6 for RF switching
    static const uint32_t rfswitch_dio_pins[] = {RADIOLIB_LR11X0_DIO5,
                                                 RADIOLIB_LR11X0_DIO6, RADIOLIB_NC,
                                                 RADIOLIB_NC, RADIOLIB_NC};

    static const Module::RfSwitchMode_t rfswitch_table[] = {
        // mode                  DIO5  DIO6
        {LR11x0::MODE_STBY, {LOW, LOW}},  {LR11x0::MODE_RX, {HIGH, LOW}},
        {LR11x0::MODE_TX, {LOW, HIGH}},   {LR11x0::MODE_TX_HP, {LOW, HIGH}},
        {LR11x0::MODE_TX_HF, {LOW, LOW}}, {LR11x0::MODE_GNSS, {LOW, LOW}},
        {LR11x0::MODE_WIFI, {LOW, LOW}},  END_OF_MODE_TABLE,
    };
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);

    // LR1121 TCXO Voltage 2.85~3.15V
    radio.setTCXO(3.0);
    heltec_delay(500);
#endif
}

void setupRadio()
{
    radio.setFrequency(LORA_BASE_FREQ);
    radio.setBandwidth(LORA_BW);
    radio.setSpreadingFactor(LORA_SF);
#if LORA_HEADER
    // Some issue it receives 0 0
    radio.implicitHeader(LORA_DATA_BYTE);
#else
    radio.explicitHeader();
#endif
    radio.setCodingRate(LORA_CR);
    radio.setPreambleLength(LORA_PREAMBLE);
    radio.forceLDRO(true);
    radio.setCRC(2);
#if LORA_TX
    radio.setOutputPower(22);
    display.println("Turn ON RX to pair you have 30 seconds");
    for (int t = 0; t < 30; t++)
    {
        display.print(".");
        delay(50);
    }
    display.println();
#endif

    initRadioSwitchTable();

#if LORA_RX
    radio.setPacketReceivedAction(onReceiveFlag);
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE)
    {
        radioIsRX = true;
        Serial.println("Listening for LoRa Packets...");
    }
    else
    {
        Serial.print("Receiver failed to start, code: ");
        Serial.println(state);
        while (true)
        {
            delay(5);
        }
    }
#endif
}
