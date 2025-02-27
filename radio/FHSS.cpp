#include "FHSS.h"

float hopTable[MAX_HOP_CHANNELS];
uint64_t packetNumber = 0;
long int receivedPacketCounter = 0;

uint32_t syncWord = 0x1A2B3C4D; // Example sync word (can be any 32-bit value)
int numChannels = 0;

// Get the next frequency from the hopping table
int hopIndex = 0;
unsigned long lastHopTime = 0;
unsigned long dwellTime = 500; // 500ms dwell time
float currentFreq = 999;

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
