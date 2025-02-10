#include <Arduino.h>
#include <RadioLib.h>

#ifndef LORA_SF
// Sets LoRa spreading factor. Allowed values range from 5 to 12.
#define LORA_SF 5
#endif

#ifndef LORA_CR
// Sets LoRa coding rate denominator. Allowed values range from 5 to 8.
#define LORA_CR 5
#endif

#ifndef LORA_BASE_FREQ
// Sets LoRa coding rate denominator. Allowed values range from 5 to 8.
#define LORA_BASE_FREQ 915
#endif

#ifndef LORA_BW
// Sets LoRa bandwidth. Allowed values are 62.5, 125.0, 250.0 and 500.0 kHz. (default,
// high = false)
#define LORA_BW 62.5 // 125.0 // 62.5
#endif

#ifndef LORA_DATA_BYTE
#define LORA_DATA_BYTE 2
#endif

#ifndef LORA_PREAMBLE
// 8 is default
#define LORA_PREAMBLE 8
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
#define SYNC_FREQUENCY 915.000

#define MAX_HOP_CHANNELS 5000              // 20 MHz range with 10 kHz step
#define PACKET_SEND_DURATION 1 * 60 * 1000 // 1 minutes in milliseconds

float hopTable[MAX_HOP_CHANNELS];

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
int packetNumber = 0;
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
        packetNumber = hopIndex;
        currentFreq = hopTable[hopIndex];
        radio.setFrequency(hopTable[hopIndex]);

        lastHopTime = currentTime;
    }
}

// Function to send 2-byte LoRa packet
void sendLoRaPacket(uint8_t cmd1, uint8_t val1, uint8_t cmd2, uint8_t val2)
{
    uint8_t paddedData[LORA_DATA_BYTE] = {0}; // Initialize with zeros
    uint16_t packet = (cmd1 << 12) | (val1 << 8) | (cmd2 << 4) | val2;

    uint8_t data[2];
    data[0] = (packet >> 8) & 0xFF; // High byte
    data[1] = packet & 0xFF;        // Low byte

    /*Serial.printf("Sending LoRa Packet: CMD1=%d, VAL1=%d, CMD2=%d, VAL2=%d\n", cmd1,
       val1, cmd2, val2);*/
    size_t dataSize = sizeof(data) / sizeof(data[0]);
    memcpy(paddedData, data, min(dataSize, (size_t)LORA_DATA_BYTE));

    int status = radio.transmit(data, LORA_DATA_BYTE);
    if (status == RADIOLIB_ERR_NONE)
    {
        Serial.println("LoRa Packet Sent!");
    }
    else
    {
        Serial.println("LoRa Transmission Failed.");
    }
}

long int startTime = 0;
void setup()
{
    Serial.begin(115200);
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
        Serial.println(String(i) + ": " + hopTable[i] + " MHz\n");
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
    radio.implicitHeader(LORA_DATA_BYTE);
    radio.setCodingRate(LORA_CR);
    radio.setPreambleLength(LORA_PREAMBLE);
    radio.forceLDRO(true);
    radio.setCRC(2);
    radio.setOutputPower(22);
}

String toBinary(int num, int bitSize = 4);

void loop()
{
    uint8_t cmd1 = 2;  // Example command 1
    uint8_t val1 = 5;  // Example value 1
    uint8_t cmd2 = 4;  // Example command 2
    uint8_t val2 = 10; // Example value 2
    long int start = millis();
    updateFrequency();
    sendLoRaPacket(cmd1, val1, cmd2, val2);
    long int end = millis();

    long int currentTime = millis();
    if (currentTime - startTime < PACKET_SEND_DURATION)
    {                       // Check if within first 5 minutes
        char packetData[2]; // 2-byte array

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

    Serial.printf("Hopping [%s] to: %.3f MHz\n", String(packetNumber), currentFreq);
    display.printf("FHSS: %.3fMHz\n", currentFreq);
    display.println("Time in the Air: " + String((end - start)));
    display.println("P:" + String(cmd1) + ":" + String(val1) + ":" + String(cmd2) + ":" +
                    String(val2));
    display.println("BP:" + toBinary(cmd1) + ":" + toBinary(val1) + ":" + toBinary(cmd2) +
                    ":" + toBinary(val2));
    Serial.println("P:" + String(cmd1) + ":" + String(val1) + ":" + String(cmd2) + ":" +
                   String(val2));
    Serial.println("BP:" + toBinary(cmd1) + ":" + toBinary(val1) + ":" + toBinary(cmd2) +
                   ":" + toBinary(val2));
    // display.println("Packet Sent");
    // delay(1000);
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
