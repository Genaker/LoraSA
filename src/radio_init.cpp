#define RADIOLIB_LOW_LEVEL (1)
#define RADIOLIB_GODMODE (1)
#define RADIOLIB_CHECK_PARAMS (0)

#include "radio_init.h"

#include <config.h>
#include <scan.h>

#if defined(LILYGO)
#include <LoRaBoards.h>
#include <LiLyGo.h>
#else
// For Heltec: we cannot include heltec_unofficial.h again (it defines globals).
// Include the convenience header for RADIOLIB_OR_HALT, and forward-declare the rest.
#include <RadioLib_convenience.h>
#include <SSD1306Wire.h>
extern SSD1306Wire display;
// PrintSplitter is defined in heltec_unofficial.h; we only need Print interface
extern Print &both;
extern void heltec_delay(int ms);
#ifdef METHOD_SPECTRAL
#include "modules/SX126x/patches/SX126x_patch_scan.h"
#endif
#endif

#include "global_config.h"
#include "ui.h"

#include <comms.h>

bool radioIsScan = false;
RadioModule *radio2 = NULL;

extern Scan r;
extern int state;
extern Config config;
extern uint64_t CONF_FREQ_BEGIN, CONF_FREQ_END;

#ifdef USING_LR1121
void setLRFreq(float freq)
{
    state = radio.setFrequency(freq);
}

static int16_t setLRFreqWithStatus(float freq)
{
    return radio.setFrequency(freq);
}
#endif

float getRSSI(void *param)
{
    Scan *r = (Scan *)param;
#if defined(USING_SX1280PA)
    radio.getRSSI(false);
#elif defined(USING_LR1121)
    float rssi;
    radio.getRssiInst(&rssi);
    return rssi;
#else
    return radio.getRSSI(false);
#endif
}

float getCAD(void *param)
{
    Scan *r = (Scan *)param;

    int16_t err = radio.scanChannel();
    if (err != RADIOLIB_ERR_NONE)
    {
        return -999;
    }

#ifdef USING_LR1121
    return radio.getRSSI();
#else
    return radio.getRSSI(true);
#endif
}

int16_t initForScan(float freq)
{
    int16_t state;

#if defined(USING_SX1280PA)
    state = radio.beginGFSK(freq);
#elif defined(USING_LR1121)
    state = radio.beginGFSK(freq, 4.8F, 5.0F, 156.2F, 10, 16U, 1.6F);

    static const uint32_t rfswitch_dio_pins[] = {RADIOLIB_LR11X0_DIO5,
                                                 RADIOLIB_LR11X0_DIO6, RADIOLIB_NC,
                                                 RADIOLIB_NC, RADIOLIB_NC};

    static const Module::RfSwitchMode_t rfswitch_table[] = {
        {LR11x0::MODE_STBY, {LOW, LOW}},  {LR11x0::MODE_RX, {HIGH, LOW}},
        {LR11x0::MODE_TX, {LOW, HIGH}},   {LR11x0::MODE_TX_HP, {LOW, HIGH}},
        {LR11x0::MODE_TX_HF, {LOW, LOW}}, {LR11x0::MODE_GNSS, {LOW, LOW}},
        {LR11x0::MODE_WIFI, {LOW, LOW}},  END_OF_MODE_TABLE,
    };
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);

    radio.setTCXO(3.0);
    heltec_delay(1000);
#else
    state = radio.beginFSK(freq);
#endif

    int gotoAcounter = 0;
#ifdef METHOD_RSSI
    while (true)
    {
#ifdef USING_SX1280PA
        state = radio.startReceive(RADIOLIB_SX128X_RX_TIMEOUT_NONE);
#elif USING_LR1121
        state = radio.startReceive(RADIOLIB_LR11X0_RX_TIMEOUT_NONE);
#else
        state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
#endif

        if (state == RADIOLIB_ERR_NONE)
            break;

        Serial.print(F("Failed to start receive mode, error code: "));
        display.drawString(0, 64 - 10, "E:startReceive");
        display.display();
        heltec_delay(2000);
        Serial.println(state);
        gotoAcounter++;
        if (gotoAcounter >= 5)
            break;
    }
#endif

    return state;
}

bool setFrequency(float curr_freq)
{
    r.current_frequency = curr_freq;
    LOG("setFrequency:%f\n", r.current_frequency);

    int16_t state;
#ifdef USING_SX1280PA
    int16_t state1 =
        radio.setFrequency(r.current_frequency);

    state = radio.startReceive(RADIOLIB_SX128X_RX_TIMEOUT_INF);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.println("Error:startReceive:" + String(state));
    }

    state = state1;
#elif USING_SX1276
    state = radio.setFrequency(r.current_frequency);
#elif USING_LR1121
    state = setLRFreqWithStatus(r.current_frequency);
#else
    state = radio.setFrequency(r.current_frequency, true);
#endif
    if (state != RADIOLIB_ERR_NONE)
    {
        display.drawString(0, 64 - 10,
                           "E(" + String(state) +
                               "):setFrequency:" + String(r.current_frequency));
        Serial.println("E(" + String(state) +
                       "):setFrequency:" + String(r.current_frequency));
        display.display();
        return false;
    }

    return true;
}

void init_radio()
{
    both.println("Init radio");
#ifndef INIT_FREQ
    state = initForScan(CONF_FREQ_BEGIN);
#else
    state = initForScan(INIT_FREQ);
#endif
    if (state == RADIOLIB_ERR_NONE)
    {
        radioIsScan = true;
        Serial.println(F("success!"));
    }
    else
    {
        display.println("Error:" + String(state));
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
        {
            delay(5);
        }
    }

#ifdef METHOD_SPECTRAL
    both.println("Upload SX1262 patch");
    RADIOLIB_OR_HALT(radio.uploadPatch(sx126x_patch_scan, sizeof(sx126x_patch_scan)));
#endif

    both.println("Setting up radio");
#ifdef USING_SX1280PA
#elif USING_SX1276
    RADIOLIB_OR_HALT(radio.setRxBandwidth(250));
#else
    RADIOLIB_OR_HALT(radio.setRxBandwidth(BANDWIDTH));
#endif

    state = radio.setDataShaping(RADIOLIB_SHAPING_NONE);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.println("Error:setDataShaping:" + String(state));
    }
    both.println("Starting scanning...");

    setFrequency(CONF_FREQ_BEGIN);

    delay(100);

#ifdef USING_SX1262
    if (config.radio2.enabled && config.radio2.module.equalsIgnoreCase("SX1262"))
    {
        radio2 = new SX1262Module(config.radio2);
        state = radio2->beginScan(CONF_FREQ_BEGIN, BANDWIDTH, RADIOLIB_SHAPING_NONE);
        if (state == RADIOLIB_ERR_NONE)
        {
            both.println("Initialized additional module OK");
            radio2->setRxBandwidth(BANDWIDTH);
        }
        else
        {
            Serial.printf("Error initializing additional module: %d\n", state);
            if (state == RADIOLIB_ERR_CHIP_NOT_FOUND)
            {
                Serial.println("Radio2: CHIP NOT FOUND");
            }
        }
    }
#endif
}
