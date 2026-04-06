#pragma once

#include <Arduino.h>
#include <radio.h>

extern bool radioIsScan;
extern RadioModule *radio2;

int16_t initForScan(float freq);
bool setFrequency(float curr_freq);
void init_radio();
float getRSSI(void *param);
float getCAD(void *param);

#ifdef USING_LR1121
void setLRFreq(float freq);
#endif
