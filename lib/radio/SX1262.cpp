#include "radio.h"
#include <LoRaBoards.h>
#include <bus.h>

#ifdef USING_SX1262
SX1262Module::SX1262Module(RadioModuleSPIConfig radio2) : RadioModule()
{
    _radio = new SX1262(new Module(
        radio2.cs, radio2.dio1, radio2.rst, radio2.busy, radio2.bus_num == 1 ? hspi : SPI,
        SPISettings(radio2.clock_freq, radio2.msb_first ? MSBFIRST : LSBFIRST,
                    radio2.spi_mode)));
    Serial.printf("Initialized Radio2: %s\n", radio2.toStr().c_str());
}

SX1262Module::~SX1262Module()
{
    if (_radio != nullptr)
    {
        Module *mod = _radio->getMod();
        delete _radio;
        delete mod;
        _radio = nullptr;
    }
}

int16_t SX1262Module::beginScan(float init_freq, float bw, uint8_t shaping)
{
    int16_t status = _radio->beginFSK(init_freq);
    if (status != RADIOLIB_ERR_NONE)
    {
        Serial.printf("Radio2: Failed beginFSK: %d\n", status);
        return status;
    }

    status = _radio->startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);

    if (status != RADIOLIB_ERR_NONE)
    {
        Serial.printf("Radio2: Failed startReceive: %d\n", status);
        return status;
    }

    status = setFrequency(init_freq);
    if (status != RADIOLIB_ERR_NONE)
    {
        Serial.printf("Radio2: Failed setFrequency: %d\n", status);
        return status;
    }

    return status;
}

int16_t SX1262Module::setFrequency(float freq)
{
    return _radio->setFrequency(freq,
                                true); // false = calibration is needed here
}

int16_t SX1262Module::setRxBandwidth(float bw) { return _radio->setRxBandwidth(bw); }

float SX1262Module::getRSSI() { return _radio->getRSSI(false); }
#endif
