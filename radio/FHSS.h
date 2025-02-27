#include <Arduino.h>
#include <LiLyGo.h>
#include <LoRaBoards.h>

#define SYNC_FREQUENCY 915.000

#define MAX_HOP_CHANNELS 5000              // 20 MHz range with 10 kHz step
#define PACKET_SEND_DURATION 1 * 60 * 1000 // 1 minutes in milliseconds

extern float hopTable[MAX_HOP_CHANNELS];
extern uint64_t packetNumber;
extern long int receivedPacketCounter;

extern uint32_t syncWord; // Example sync word (can be any 32-bit value)
extern int numChannels;

// Get the next frequency from the hopping table
extern int hopIndex;
extern unsigned long lastHopTime;
extern unsigned long dwellTime; // 500ms dwell time
extern float currentFreq;

// Function to generate a frequency hopping table, adapting if channels are fewer
int generateFrequencies(uint32_t syncWord, float startFreq, float stepKHz,
                        float maxWidthMHz);

// Function to print the generated table (for debugging)
void printHopTable(int numChannels);

void updateFrequency();
