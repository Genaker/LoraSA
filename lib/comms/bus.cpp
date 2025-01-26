#include "bus.h"
#include <Wire.h>

bool initUARTs(Config &config)
{
    if (config.uart0.enabled)
    {
        Uart0.end();
        Uart0.begin(config.uart0.clock_freq, SERIAL_8N1, config.uart0.rx,
                    config.uart0.tx);
    }

    if (config.uart1.enabled)
    {
        Uart1.end();
        Uart1.begin(config.uart1.clock_freq, SERIAL_8N1, config.uart1.rx,
                    config.uart1.tx);
    }

    return true;
}

SPIClass &hspi = *(new SPIClass(HSPI)); // not usable until initSPIs

bool initSPIs(Config &config)
{
    if (config.spi1.enabled)
    {
        delete (&hspi);
        hspi = *(new SPIClass(config.spi1.bus_num));
        if (config.spi1.clock_freq > 0)
        {
            hspi.setFrequency(config.spi1.clock_freq);
        }

        // if all the pins are -1, then will use the default for SPI bus_num
        hspi.begin(config.spi1.clk, config.spi1.miso, config.spi1.mosi);
        Serial.printf("Initialized SPI%d: SC:%d MISO:%d MOSI:%d clock:%d\n",
                      (int)config.spi1.bus_num, (int)config.spi1.clk,
                      (int)config.spi1.miso, (int)config.spi1.mosi,
                      (int)config.spi1.clock_freq);
    }

    return true;
}

uint8_t _scanSupportedDevicesOnWire(TwoWire &w, int bus_num);

uint8_t wireDevices;
uint8_t wire1Devices;

bool initWires(Config &config)
{
    wireDevices = _scanSupportedDevicesOnWire(Wire, 0);

    if (config.wire1.enabled)
    {
#if SOC_I2C_NUM > 1
        // if you want to use default pins, configure -1
        // if you want to use default clock speed, configure 0
        if (!Wire1.begin(config.wire1.sda, config.wire1.scl, config.wire1.clock_freq))
        {
            Serial.println("Failed to initialize Wire1");
            return false;
        }

        wire1Devices = _scanSupportedDevicesOnWire(Wire1, 1);
#endif
    }

    return true;
}

uint8_t _scanSupportedDevicesOnWire(TwoWire &w, int bus_num)
{
    uint8_t res = 0;

    for (int i = 0; known_i2c_devices[i].address > 0; i++)
    {
        w.beginTransmission(known_i2c_devices[i].address);
        delay(2);
        if (w.endTransmission() == 0)
        {
            Serial.printf("Found supported device on Wire%d: %s(%X)\n", bus_num,
                          known_i2c_devices[i].name.c_str(),
                          (int)known_i2c_devices[i].address);
            res |= 1 << i;
        }
    }

    return res;
}
