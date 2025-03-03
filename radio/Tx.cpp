#include "USB-Serial.h"
#include "config.h"
#include "radio-protocol.h"
#include "utility.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <cmath>
#include <esp_system.h>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <vector>

// Support heltec boards
#ifndef LILYGO
#include <heltec_unofficial.h>
#endif // end ifndef LILYGO

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

long int lastWriteTime = 0;

#if PROTOCOL == IBUS
#include "IBUS.h"
// Create an instance of the Ibus class
Ibus ibus;
// Test data list with all control values set to 1700
uint8_t testControlValues[IBUS_CHANNELS_COUNT * 2];
#endif

#if PROTOCOL == SBUS
#include "SBUS.h"
SBUS2 sbus(Serial1); // Use Serial1 for SBUS output
#endif

#include "FHSS.h"

#if PROTOCOL == CRSF
#include "CRSF.h"
// CRSF crsf(Serial1, TXD1, TXD1, 420000); // Use Serial1, TX_PIN, RX_PIN, BAUD_RATE
CRSF2 crsf(Serial1, -1, TXD1);
#endif
// Example usage

#include "radio.h"

#if RUN_TESTS
void testMap11BitTo4Bit();
void testMap4BitTo11Bit();
#endif

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
    // clearSbusData();
#if LORA_RX
    // sbusWrite.Begin();
#endif
#endif

#if PROTOCOL == CRSF
    crsf.begin();
#endif // end CRSF

#if RUN_TESTS
    testMap11BitTo4Bit();
    testMap4BitTo11Bit();
#endif
    // testMap11BitTo4Bit();
    //  Initialize SBUS communication

#if LORA_TX
    // sbusRead.Begin();
#endif
    Serial.println("SBUS write and read are ready");

    heltec_setup();
    startTime = millis();

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

    // Setup Radio
    setupRadio();
}

String toBinary(int num, int bitSize = 4);
std::map<int, int> lastSerialCommands;
void loop()
{
// Read serial input
#if LORA_TX && SERIAL_INPUT
    Serial.println("Waiting for Serial Commands");
    while (true)
    {
        std::map<int, int> serialCommands = readSerialInput();
        if (serialCommands.empty())
        {
            // Serial.println("Waiting for Serial Commands");
            delay(20);
        }
        else
        {
            lastSerialCommands = serialCommands;
            break;
        }
    }
#endif

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
    display.println("I-BUS:" + str);

    ibus.setControlValuesList(testControlValues);
    ibus.sendPacket();
    delay(20);
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
    // writeSbusData(sbusSend);
    sbus.send(sbusSend);
    display.println("SBUS");
    Serial.println("SBUS: " + String(sbusSend[0]) + "," + String(sbusSend[1]) + "," +
                   String(sbusSend[2]) + "," + String(sbusSend[3]) + "," +
                   String(sbusSend[4]) + "," + String(sbusSend[5]) + "," +
                   String(sbusSend[6]) + "," + String(sbusSend[7]) + "," +
                   String(sbusSend[8]) + "," + String(sbusSend[9]) + "," +
                   String(sbusSend[10]) + "," + String(sbusSend[11]));
    delay(30);
    //   Read data for test purpose
    //  readSbusData();
#endif // end SBUS

#if PROTOCOL == CRSF
    display.println("CRSF");
    uint32_t seed = esp_random() ^ millis();

    randomSeed(seed);
    //  Set all control values to 1700

    uint16_t sbusSend[16] = INIT_SBUS_ARRAY;
    // Example: Set channel values
    uint16_t channels[CRSF_MAX_CHANNELS] = {random(1200, 1900),
                                            random(1200, 1900),
                                            random(1200, 1900),
                                            random(1200, 1900),
                                            random(1200, 1900),
                                            random(1200, 1900),
                                            random(1200, 1900),
                                            1435,
                                            1450,
                                            1800,
                                            1300,
                                            1000,
                                            1500,
                                            2000,
                                            1200,
                                            1300};

    crsf.sendRCFrame(channels);
    Serial.println("CRSF: " + String(channels[0]) + "," + String(channels[1]) + "," +
                   String(channels[2]) + "," + String(channels[3]));
    delay(50);
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
    // readSbusData();

    val1 = convertTo4Bit(testChannels[4]);
    val2 = convertTo4Bit(testChannels[1]);
    // 4 byte packet
    val3 = convertTo4Bit(testChannels[2]);
    val4 = convertTo4Bit(testChannels[3]);
    if (val3 != 8 && val4 != 8)
    {
        radio.setBandwidth(62.5);
        // RC 1 1500
        sendLoRaRCPacket(cmd1, val1, cmd2, val2, cmd3, val3, cmd4, val4);
        delay(500);
        radio.setBandwidth(500);
        sendLoRaRCPacket(cmd1, val1, cmd2, val2, cmd3, val3, cmd4, val4);
    }
    else
    {
        radio.setBandwidth(62.5);
        sendLoRaRCPacket(cmd1, val1, cmd2, val2);
        delay(500);
        radio.setBandwidth(500);
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
