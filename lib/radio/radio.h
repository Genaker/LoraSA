#pragma once

#include <RadioLib.h>
#include <config.h>

struct RadioModule
{
    RadioModule() {};
    virtual ~RadioModule() {};

    virtual int16_t beginScan(float init_freq, float bw, uint8_t shaping) = 0;
    virtual int16_t setFrequency(float freq) = 0;
    virtual int16_t setRxBandwidth(float bw) = 0;
    virtual float getRSSI() = 0;
};

#ifndef USING_SX1262_no
struct SX1262Module : RadioModule
{
    SX1262 *_radio;

    SX1262Module(RadioModuleSPIConfig cfg);
    ~SX1262Module() override;

    int16_t beginScan(float init_freq, float bw, uint8_t shaping) override;
    int16_t setFrequency(float freq) override;
    int16_t setRxBandwidth(float bw) override;

    float getRSSI() override;
};
#endif
