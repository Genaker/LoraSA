#include <Arduino.h>
#include <FreeRTOS.h>
#include <cmath>
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

// SBUS packet structure
#define SBUS_PACKET_SIZE 25

uint16_t channels[16];
bool failSafe;
bool lostFrame;

void readSbusData();
void writeSbusData(uint16_t channels[]);

String readSerialInput();
std::map<int, int> processSerialCommand(const String &input);

#include <pt.h>

// Define the protothread control structure
static struct pt ptWriteSbusData;

/*
commands      sending message              comments
-----------------------------------------------------
roll	      rc 1 <value>	        // 	move left or right
pitch         rc 2 <value>          // move forward or backwards
yaw	          rc 4 <value>	        // turn left or right
throttle      rc 3 <value>  	    // move up or down
*/
enum Command
{
    HEART_BEAT = 0, // Corresponds to rc 0
    ROLL = 1,       // Corresponds to rc 1
    PITCH = 2,      // Corresponds to rc 2
    THROTTLE = 3,   // Corresponds to rc 3
    YAW = 4,        // Corresponds to rc 4
    ////  ----- Not Assigned Yet -----
    AUX1 = 5, // Corresponds to rc 5
    AUX2 = 6, // Corresponds to rc 6
    AUX3 = 7, // Corresponds to rc 7
    AUX4 = 8, // Corresponds to rc 8
    AUX5 = 9, // Corresponds to rc 9
    AUX6 = 10 // Corresponds to rc 10
};

// Create a map from Command to string
std::unordered_map<Command, String> commandToStringMap = {{HEART_BEAT, "HEART_BEAT"},
                                                          {ROLL, "ROLL"},
                                                          {PITCH, "PITCH"},
                                                          {YAW, "YAW"},
                                                          {THROTTLE, "THROTTLE"}};

// Define the mapping table
std::vector<std::pair<uint8_t, uint16_t>> channelValueMappingTable = {
    {0, 1300},  {1, 1325},  {2, 1350},  {3, 1375}, {4, 1400},  {5, 1425},
    {6, 1450},  {7, 1475},  {8, 1500},  {9, 1525}, {10, 1550}, {11, 1575},
    {12, 1600}, {13, 1625}, {14, 1650}, {15, 1675}};
// 0 - 1300 0
// 1300 - 1325 1
// 1325 - 1350 2
// 1350 - 1375 3
// 1375 - 1400 4
// 1400 - 1425 5
// 1425 - 1450 6
// 1450 - 1475 7
// 1475 - 1500 8
// 1500 - 1525 9
// 1525 - 1550 10
// 1550 - 1575 11
// 1575 - 1600 12
// 1600 - 1625 13
// 1625 - 1650 14
// 1650 - 1675 15

#define INIT_SBUS_ARRAY                                                                  \
    {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,                                     \
     1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}

// Create SBUS objects for reading and writing
// Create SBUS objects for reading and writing
bfs::SbusTx sbusWrite(&Serial1, -1, TXD1, true); // Use Serial1 for SBUS transmission
bfs::SbusRx sbusRead(&Serial2, RXD2, -1, true);  // Use Serial2 for SBUS reception

// Data structure to hold channel data
bfs::SbusData sbusDataRead;
bfs::SbusData sbusDataWrite;
// print command details
uint16_t getCommandValue(Command cmd)
{
    String commandName =
        commandToStringMap.count(cmd) ? commandToStringMap[cmd] : "UNKNOWN";
    Serial.println("RC " + commandName + " : " + String(sbusDataRead.ch[cmd]));
    return sbusDataRead.ch[cmd];
}

uint16_t testChannels[16] = {1500, 2000, 1350, 1400, 1505, 1506, 1507, 1508,
                             1509, 1510, 1511, 1512, 1513, 1514, 1515, 1516};

long int lastWriteTime = 0;
// Protothread function to write SBUS data
int writeSbusDataThread(struct pt *pt)
{
    PT_BEGIN(pt);

    while (1)
    {
        writeSbusData(testChannels); // Call your SBUS data writing function
        PT_WAIT_UNTIL(pt, millis() - lastWriteTime >= 100);
        lastWriteTime = millis();
    }

    PT_END(pt);
}

// P:2:15:4:2
// BP:0010:1111:0100:0010
uint8_t convertTo4Bit(uint16_t value11Bit);
int16_t map4BitTo11Bit(uint8_t value4Bit);

void clearSbusData();

#define SYNC_FREQUENCY 915.000

#define MAX_HOP_CHANNELS 5000              // 20 MHz range with 10 kHz step
#define PACKET_SEND_DURATION 1 * 60 * 1000 // 1 minutes in milliseconds

float hopTable[MAX_HOP_CHANNELS];
uint64_t packetNumber = 0;
long int receivedPacketCounter = 0;

uint32_t syncWord = 0x1A2B3C4D; // Example sync word (can be any 32-bit value)
int numChannels = 0;
// Function to generate a frequency hopping table, adapting if channels are fewer
int generateFrequencies(uint32_t syncWord, float startFreq, float stepKHz,
                        float maxWidthMHz)
{
    float stepMHz = stepKHz / 1000.0; // Convert kHz to MHz
    numChannels = int((maxWidthMHz * 1e3) /
                      stepKHz); // Calculate number of channels within max width

    // If fewer channels are available, adjust dynamically
    if (numChannels < 10)
    { // Less than 10 channels is not good for FHSS
        Serial.println("Warning: Too few channels! FHSS may not work well.");
        numChannels = 10; // Ensure a minimum of 10 channels
    }

    if (numChannels > MAX_HOP_CHANNELS)
    {
        Serial.println("Warning: Reducing channels to MAX_HOP_CHANNELS.");
        numChannels = MAX_HOP_CHANNELS; // Prevent overflow
    }

    // Generate sequential frequencies within max width
    for (int i = 0; i < numChannels; i++)
    {
        hopTable[i] = startFreq + (i * stepMHz);
    }

    // Shuffle using sync word (randomize the order)
    for (int i = 0; i < numChannels; i++)
    {
        syncWord = (syncWord * 1103515245 + 12345) & 0x7FFFFFFF;
        int swapIndex = syncWord % numChannels;

        // Swap values
        float temp = hopTable[i];
        hopTable[i] = hopTable[swapIndex];
        hopTable[swapIndex] = temp;
    }

    return numChannels; // Return actual number of channels generated
}

// Function to print the generated table (for debugging)
void printHopTable(int numChannels)
{
    delay(100);
    Serial.println("------");
    Serial.println("Generated Frequency Hopping Table [" + String(numChannels) + "]:");
    /*for (int i = 0; i < numChannels; i++)
    {
        Serial.println(String(i) + ": " + hopTable[i] + " MHz\n");
    }*/
    delay(1000);
}

// Get the next frequency from the hopping table
int hopIndex = 0;
unsigned long lastHopTime = 0;
unsigned long dwellTime = 500; // 500ms dwell time
float currentFreq = 999;
void updateFrequency()
{
    unsigned long currentTime = millis();

    if (currentTime - lastHopTime >= dwellTime)
    {
        if (hopIndex == numChannels)
        {
            hopIndex = 0;
        }
        hopIndex = hopIndex + 1;
        // packetNumber = hopIndex;
        currentFreq = hopTable[hopIndex];
        radio.setFrequency(hopTable[hopIndex]);

        lastHopTime = currentTime;
    }
}

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
    clearSbusData();
    Serial.begin(115200);
#if RUN_TESTS
    testMap11BitTo4Bit();
    testMap4BitTo11Bit();
#endif
    // testMap11BitTo4Bit();
    //  Initialize SBUS communication
#if LORA_RX
    sbusWrite.Begin();
#endif
#if LORA_TX
    sbusRead.Begin();
#endif
    Serial.println("SBUS write and read are ready");

    // Initialize the protothread
    PT_INIT(&ptWriteSbusData);

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
    // Run the protothread
    // writeSbusDataThread(&ptWriteSbusData);
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

            uint16_t sbusSend[16] = INIT_SBUS_ARRAY;
            sbusSend[cmd1] = map4BitTo11Bit(val1);
            sbusSend[cmd2] = map4BitTo11Bit(val2);
            if (len == 4 && cmd3 != 0 && cmd4 != 0)
            {
                sbusSend[cmd3] = map4BitTo11Bit(val3);
                sbusSend[cmd4] = map4BitTo11Bit(val4);
            }
            writeSbusData(sbusSend);
            //  Read data for test purpose
            // readSbusData();
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

void readSbusData()
{
    // Read SBUS data from Serial2
    if (false && sbusRead.Read())
    {
        sbusDataRead = sbusRead.data();

        Serial.println("Received SBUS data:");
        for (int i = 0; i < bfs::SbusData::NUM_CH; i++)
        {
            Serial.print("Channel ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(sbusDataRead.ch[i]);
        }
        Serial.print("FailSafe: ");
        Serial.println(sbusDataRead.failsafe);
        Serial.print("Lost Frame: ");
        Serial.println(sbusDataRead.lost_frame);
    }
    if (bool test = true)
    {
        for (int i = 0; i < 16; i++)
        {
            sbusDataRead.ch[i] = testChannels[i];
            Serial.print("Channel ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(sbusDataRead.ch[i]);
        }
    }
}

void writeSbusData(uint16_t channels[])
{
    // Example: Send SBUS data over Serial1
    for (int i = 0; i < bfs::SbusData::NUM_CH; i++)
    {
        sbusDataWrite.ch[i] = channels[i]; // Example data
    }
    sbusWrite.data(sbusDataWrite);
    sbusWrite.Write();
}

void clearSbusData()
{
    // Assuming bfs::SbusData has a member array `ch` and boolean members `failsafe` and
    // `lost_frame`
    for (int i = 0; i < bfs::SbusData::NUM_CH; i++)
    {
        sbusDataRead.ch[i] = 1500;
        sbusDataWrite.ch[i] = 1500;
    }
    sbusDataRead.failsafe = false;
    sbusDataRead.lost_frame = false;
    sbusDataWrite.failsafe = false;
    sbusDataWrite.lost_frame = false;
}

uint8_t map11BitTo4Bit(uint16_t value11Bit)
{
    // Initialize variables to track the closest match
    uint8_t closest4BitValue = 0;
    uint16_t smallestDifference = UINT16_MAX;

    const auto &lastEntry = channelValueMappingTable.back();
    // Find the closest match in the table
    for (const auto &entry : channelValueMappingTable)
    {

        uint8_t key = entry.first;     // Access the key
        uint16_t value = entry.second; // Access the value
        if (value11Bit >= lastEntry.second)
        {
            closest4BitValue = lastEntry.first;
            break;
        }
        else if (value11Bit <= value)
        {
            closest4BitValue = key;
            break;
        }
    }

    return closest4BitValue;
}

int16_t map4BitTo11Bit(uint8_t value4Bit)
{
    // Iterate through the mapping table to find the corresponding 11-bit value
    for (const auto &entry : channelValueMappingTable)
    {
        uint8_t key = entry.first;     // Access the 4-bit key
        uint16_t value = entry.second; // Access the 11-bit value

        if (key == value4Bit)
        {
            return value; // Return the 11-bit value if the key matches
        }
    }

    // If no match is found, return a default value or handle the error
    // For example, return 0 - 1500 or throw an exception
    return 1500; // Or handle the error as needed
}

uint8_t convertTo4Bit(uint16_t value11Bit) { return map11BitTo4Bit(value11Bit); }

// Test function
void testMap11BitTo4Bit()
{
    Serial.println("Test Mapping 11-bit to 4-bit");

    // Test cases: {input, expected_output}
    std::vector<std::pair<uint16_t, uint8_t>> testCases = {
        {1290, 0}, {1300, 0},  {1310, 1},  {1325, 1},  {1340, 2},
        {1500, 8}, {1600, 12}, {1700, 15}, {1675, 15}, {1800, 15}};
    // 0 - 1300 0
    // 1300 - 1325 1
    // 1325 - 1350 2
    // 1350 - 1375 3
    // 1375 - 1400 4
    // 1400 - 1425 5
    // 1425 - 1450 6
    // 1450 - 1475 7
    // 1475 - 1500 8
    // 1500 - 1525 9
    // 1525 - 1550 10
    // 1550 - 1575 11
    // 1575 - 1600 12
    // 1600 - 1625 13
    // 1625 - 1650 14
    // 1650 - 1675 15
    bool failed = false;

    for (const auto &testCase : testCases)
    {
        uint16_t input = testCase.first;
        uint8_t expectedOutput = testCase.second;
        uint8_t actualOutput = map11BitTo4Bit(input);
        bool assert = actualOutput == expectedOutput;
        if (!assert)
        {
            Serial.print("Test Failed->");
            failed = true;
        }
        Serial.println("Test input " + String(input) + ": expected " +
                       String(expectedOutput) + ", got " + String(actualOutput));
        delay(100);
    }
    if (failed)
    {
        Serial.println("Test Failed");
        delay(500);
    }
}

// Test function
void testMap4BitTo11Bit()
{
    Serial.println("Testing map4BitTo11Bit");
    // Test cases: {input, expected_output}
    std::vector<std::pair<uint8_t, uint16_t>> testCases = {
        {0, 1300}, {0, 1300},  {1, 1325},  {1, 1325},  {2, 1350},
        {8, 1500}, {12, 1600}, {15, 1675}, {14, 1650}, {18, 1500}};

    // 1300 0
    // 1325 1
    // 1350 2
    // 1375 3
    // 1400 4
    // 1425 5
    // 1450 6
    // 1475 7
    // 1500 8
    // 1525 9
    // 1550 10
    // 1575 11
    // 1600 12
    // 1625 13
    // 1650 14
    // 1675 15
    bool failed = false;

    for (const auto &testCase : testCases)
    {
        uint16_t input = testCase.first;
        uint16_t expectedOutput = testCase.second;
        uint16_t actualOutput = map4BitTo11Bit(input);
        bool assert = actualOutput == expectedOutput;
        if (!assert)
        {
            Serial.print("Test Failed->");
            failed = true;
        }
        Serial.println("Test input " + String(input) + ": expected " +
                       String(expectedOutput) + ", got " + String(actualOutput));
        delay(100);
    }
    if (failed)
    {
        Serial.println("Test Failed");
        delay(500);
    }
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
