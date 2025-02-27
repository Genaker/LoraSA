#include <Arduino.h>
#include <FreeRTOS.h>
#include <cmath>
#include <esp_system.h>
#include <map>
#include <sbus.h>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#define RUN_TESTS 0

#ifndef LORA_SF
// Sets LoRa spreading factor. Allowed values range from 5 to 12.
#define LORA_SF 8
#endif

#ifndef LORA_CR
#define LORA_CR 8
#endif

#ifndef DEBUG_RX
#define DEBUG_RX 0
#endif

#ifndef LORA_HEADER
#define LORA_HEADER 0
#endif

#ifndef LORA_FHSS
#define LORA_FHSS 0
#endif

#ifndef LORA_RX
#define LORA_RX 0
#endif

#ifndef LORA_TX
#define LORA_TX 1
#endif

#ifndef LORA_BASE_FREQ
// Sets LoRa coding rate denominator. Allowed values range from 5 to 8.
#define LORA_BASE_FREQ 915
#endif

#ifndef LORA_BW
// Sets LoRa bandwidth. Allowed values are 62.5, 125.0, 250.0 and 500.0 kHz. (default,
// high = false)
#define LORA_BW 62.5 // 31.25 // 125.0 // 62.5 // 31.25
#endif

#ifndef LORA_DATA_BYTE
#define LORA_DATA_BYTE 2
#endif

#define SBUS 1 // Doesn't work
#define IBUS 2
#define CROS 3

#ifndef PROTOCOL
#define PROTOCOL CROS // IBUS // SBUS
#endif

#ifndef LORA_PREAMBLE
// 8 is default
#if LORA_SF == 6 || LORA_SF == 5
#define LORA_PREAMBLE 8
#elif LORA_SF == 7
#define LORA_PREAMBLE 8
#else
#define LORA_PREAMBLE 10
#endif
#endif

#if defined(LILYGO)
// LiLyGO device does not support the auto download mode, you need to get into the
// download mode manually. To do so, press and hold the BOOT button and then press the
// RESET button once. After that release the BOOT button. Or OFF->ON together with BOOT

// Default LilyGO code
#include <LoRaBoards.h>

// #include "utilities.h"
//  Our Code
#include <LiLyGo.h>
#endif // end LILYGO

#define RADIOLIB_GODMODE (1)
#define RADIOLIB_CHECK_PARAMS (0)

#include <RadioLib.h>

// Define the UART ports and pins
#define TXD1 39 // Transmit pin for Serial1
#define RXD2 40 // Receive pin for Serial2

String readSerialInput();
std::map<int, int> processSerialCommand(const String &input);

long int lastWriteTime = 0;

#if PROTOCOL == IBUS
#include "i-bus.h"
// Create an instance of the Ibus class
Ibus ibus;
// Test data list with all control values set to 1700
uint8_t testControlValues[IBUS_CHANNELS_COUNT * 2];
#endif

#if PROTOCOL == SBUS
#include "s-bus.h"
#endif

#include "FHSS.h"

#if PROTOCOL == CROS
#include "CRSF.h"
CRSF crsf(Serial1, TXD1, -1, 420000); // Use Serial1, TX_PIN, RX_PIN, BAUD_RATE
#endif
// Example usage

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

void onReceive();
void onReceiveFlag(void);
#if RUN_TESTS
void testMap11BitTo4Bit();
void testMap4BitTo11Bit();
#endif

bool radioIsRX = false;
long int startTime = 0;
void setup()
{

    Serial.begin(115200);

    // Initialize Serial1 for iBUS communication with a custom TX pin
#if PROTOCOL == IBUS
    ibus.begin(Serial1, TXD1);
    ibus.enable();
#endif

#if PROTOCOL == SBUS
    clearSbusData();
#if LORA_RX
    sbusWrite.Begin();
#endif
#endif

#if PROTOCOL == CROS
    crsf.begin();
#endif // end CRSF

#if RUN_TESTS
    testMap11BitTo4Bit();
    testMap4BitTo11Bit();
#endif
    // testMap11BitTo4Bit();
    //  Initialize SBUS communication

#if LORA_TX
    sbusRead.Begin();
#endif
    Serial.println("SBUS write and read are ready");

    heltec_setup();
    startTime = millis();

    uint32_t syncWord = 98754386857476;             // Example sync word
    float maxWidthMHz = 5.0;                        // Max hopping width of 20 MHz
    float startFreq = LORA_BASE_FREQ - maxWidthMHz; // Start at 900 MHz
    float stepKHz = 10.0;                           // 10 kHz step size

    numChannels = generateFrequencies(syncWord, startFreq, stepKHz, maxWidthMHz);

    delay(100);
    Serial.println("------");
    Serial.println("Generated Frequency Hopping Table [" + String(numChannels) + "]:");
    for (int i = 0; i < numChannels; i++)
    {
        // Serial.println(String(i) + ": " + hopTable[i] + " MHz\n");
    }
    delay(1000);

    printHopTable(numChannels); // Print the generated table

#ifdef LILYGO
    setupBoards(); // true for disable U8g2 display library
    delay(200);
    Serial.println("Setup LiLyGO board is done");
    display.println("Setup LiLyGO board is done");
#endif
    /// beginGFSK
    if (radio.begin() == RADIOLIB_ERR_NONE)
    {
        Serial.println("LoRa Initialized");
    }
    else
    {
        Serial.println("LoRa Initialization Failed!");
        while (true)
            ;
    }

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

void forceRestartLoRa();
String toBinary(int num, int bitSize = 4);

unsigned long lastPacketTime = 0;
long int packetN = 0;
void loop()
{
    String input = readSerialInput();
    std::map<int, int> result = {};
    if (!input.isEmpty())
    {
        result = processSerialCommand(input);
    }

    if (result.empty())
    {
        Serial.println("The map is empty.");
    }
    else
    {
        Serial.println("The map contains data.");
    }
#if PROTOCOL == IBUS
    uint32_t seed = esp_random() ^ millis();
    randomSeed(seed);
    String str = "";
    // Set all control values to 1700
    for (int i = 0; i < IBUS_CHANNELS_COUNT; i++)
    {
        uint16_t randomValue =
            random(1200, 1900); // Generate random values between 1200 and 1900
        str += String(randomValue) + ",";
        testControlValues[i * 2] = randomValue & 0xFF;            // Low byte
        testControlValues[i * 2 + 1] = (randomValue >> 8) & 0xFF; // High byte
    }
    Serial.println("I-BUS:" + str);

    ibus.setControlValuesList(testControlValues);
    ibus.sendPacket();
#endif // end IBUS

#if PROTOCOL == SBUS
    uint32_t seed = esp_random() ^ millis();

    randomSeed(seed);
    //  Set all control values to 1700

    uint16_t sbusSend[16] = INIT_SBUS_ARRAY;

    for (int i = 0; i < 12; i++)
    {
        uint16_t randomValue = random(1200, 1900);
        sbusSend[i] = randomValue; // map4BitTo11Bit(randomValue);
    }
    writeSbusData(sbusSend);
    // delay(500);
    //   Read data for test purpose
    //  readSbusData();
#endif // end SBUS

#if PROTOCOL == CROS
    // Example: Set channel values
    uint16_t channels[] = {1700, 1800, 1600, 1200,
                           1580, 1600, 1300, 1900}; // Example channel values
    crsf.setChannels(channels, sizeof(channels) / sizeof(channels[0]));
#endif // end CRSF

    uint8_t cmd1 = 0;  // Example command 1
    uint8_t val1 = 5;  // Example value 1
    uint8_t cmd2 = 1;  // Example command 2
    uint8_t val2 = 10; // Example value 2
    uint8_t val3 = 0, val4 = 0;
    uint8_t cmd3 = 2, cmd4 = 3;
    long int start = millis();
#if LORA_FHSS
    updateFrequency();
#endif
#if LORA_TX
    // listen to the Sbus commands form the Ground station or RC
    readSbusData();

    val1 = convertTo4Bit(sbusDataRead.ch[0]);
    val2 = convertTo4Bit(sbusDataRead.ch[1]);
    // 4 byte packet
    val3 = convertTo4Bit(sbusDataRead.ch[2]);
    val4 = convertTo4Bit(sbusDataRead.ch[3]);
    if (val3 != 8 && val4 != 8)
    {
        sendLoRaRCPacket(cmd1, val1, cmd2, val2, cmd3, val3, cmd4, val4);
    }
    else
    {
        sendLoRaRCPacket(cmd1, val1, cmd2, val2);
    }

    packetNumber++;

    if (packetNumber % 10 == 0)
    {
        // 8 B
        uint8_t data8[8];
        data8[0] = (packetNumber >> 56) & 0xFF;
        data8[1] = (packetNumber >> 48) & 0xFF;
        data8[2] = (packetNumber >> 40) & 0xFF;
        data8[3] = (packetNumber >> 32) & 0xFF;
        data8[4] = (packetNumber >> 24) & 0xFF;
        data8[5] = (packetNumber >> 16) & 0xFF;
        data8[6] = (packetNumber >> 8) & 0xFF;
        data8[7] = packetNumber & 0xFF;
        packetNumber++;

        Serial.println("Sending 8 byte packet Number(" + String(sizeof(data8)) + ")");
        sendLoRaPacket(data8, 8);
        heltec_delay(50);
    }

#endif
#if LORA_RX
    if (packetReceived)
    {
        packetReceived = false;
        onReceive();
    }
    /*if (millis() - lastPacketTime > 15000)
    { // No packet for 15s? Reset.
        Serial.println("[LoRa] No packets received for a while, restarting...");
        forceRestartLoRa();
    }*/
    // Serial.print("[LoRa] Current Status: ");
#endif

    long int end = millis();

#if LORA_TX && LORA_FHSS
    long int currentTime = millis();
    if (currentTime - startTime < PACKET_SEND_DURATION)
    {                                    // Check if within first 5 minutes
        char packetData[LORA_DATA_BYTE]; // 2-byte array

        // Store packet number into 2 bytes (big-endian format)
        packetData[0] = (packetNumber >> 8) & 0xFF; // High byte
        packetData[1] = packetNumber & 0xFF;        // Low byte
        radio.setSpreadingFactor(5);
        radio.setBandwidth(125.0);

        radio.transmit((uint8_t *)packetData, 2); // Send exactly 2 bytes
        Serial.printf("Sent: %s on %.3f MHz\n", packetData, hopTable[hopIndex]);
    }
    radio.setSpreadingFactor(LORA_SF);
    radio.setBandwidth(LORA_BW);
#endif

#if LORA_TX

#if LORA_FHSS
    Serial.printf("Hopping [%s] to: %.3f MHz\n", String(packetNumber), currentFreq);
    display.printf("FHSS: %.3fMHz\n", currentFreq);
#endif
    Serial.println("Packet: " + String(packetSave));

    display.println("Time in the Air: " + String((end - start)));
    Serial.println("Time in the Air: " + String((end - start)));

    display.println("P:" + String(cmd1) + ":" + String(val1) + ":" + String(cmd2) + ":" +
                    String(val2));
    display.println("BP:" + toBinary(cmd1) + ":" + toBinary(val1) + ":" + toBinary(cmd2) +
                    ":" + toBinary(val2));

    Serial.println("P:" + String(cmd1) + ":" + String(val1) + ":" + String(cmd2) + ":" +
                   String(val2));
    Serial.println("BP:" + toBinary(cmd1) + ":" + toBinary(val1) + ":" + toBinary(cmd2) +
                   ":" + toBinary(val2));

    if (val3 != 8 && val4 != 8)
    {
        Serial.println("P2:" + String(cmd3) + ":" + String(val3) + ":" + String(cmd4) +
                       ":" + String(val4));
        Serial.println("BP2:" + toBinary(cmd3) + ":" + toBinary(val3) + ":" +
                       toBinary(cmd4) + ":" + toBinary(val4));
    }
    // display.println("Packet Sent");
    // delay(1000);
#endif
}

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif

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

String toBinary(int num, int bitSize)
{
    if (num == 0)
        return "0";

    String binary = "";
    while (num > 0)
    {
        binary = String(num % 2) + binary;
        num /= 2;
    }

    // Pad with leading zeros to match `bitSize`
    while (binary.length() < bitSize)
    {
        binary = "0" + binary;
    }

    return binary;
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

String readSerialInput()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil(
            '\n');    // Read the incoming data until a newline character
        input.trim(); // Remove any leading or trailing whitespace
        return input;
    }
    return ""; // Return an empty string if no data is available
}

std::map<int, int> processSerialCommand(const String &input)
{
    std::map<int, int> keyValuePairs;
    if (input.startsWith("RC "))
    {
        int key1, value1, key2, value2;
        char command[3];
        int parsed = sscanf(input.c_str(), "%s %d %d %d %d", command, &key1, &value1,
                            &key2, &value2);

        if (parsed == 5 && key1 >= 1 && key1 <= 16 && value1 >= 0 && value1 < 2048 &&
            key2 >= 1 && key2 <= 16 && value2 >= 0 && value2 < 2048)
        {
            // Process the command
            Serial.print("Received command: ");
            Serial.print(command);
            Serial.print(", Key1: ");
            Serial.print(key1);
            Serial.print(", Value1: ");
            Serial.print(value1);
            Serial.print(", Key2: ");
            Serial.print(key2);
            Serial.print(", Value2: ");
            Serial.println(value2);

            keyValuePairs[key1] = value1;
            keyValuePairs[key2] = value2;
        }
        else
        {
            Serial.println("Invalid command format or out of range values.");
        }
    }
    else
    {
        Serial.println("Invalid command prefix.");
    }
    return keyValuePairs;
}
