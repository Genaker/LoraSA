#include "bus.h"

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

#ifndef HELTEC
SPIClass &hspi = *(new SPIClass(HSPI)); // not usable until initSPIs
#endif

bool initSPIs(Config &config)
{
    if (config.spi1.enabled)
    {
#ifndef HELTEC
        delete (&hspi);
        hspi = *(new SPIClass(config.spi1.bus_num));
        if (config.spi1.clock_freq > 0)
        {
            hspi.setFrequency(config.spi1.clock_freq);
        }

        // if all the pins are -1, then will use the default for SPI bus_num
        hspi.begin(config.spi1.clk, config.spi1.miso, config.spi1.mosi);

        Serial.printf("Initialized SPI%d @ %x: SC:%d MISO:%d MOSI:%d clock:%d\n",
                      (int)config.spi1.bus_num, (void *)&hspi, (int)config.spi1.clk,
                      (int)config.spi1.miso, (int)config.spi1.mosi,
                      (int)config.spi1.clock_freq);
#else
        Serial.println("Custom SPI initializer not supported on Heltec");
#endif
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

uint8_t _write_register(TwoWire &wire, uint8_t addr, uint8_t reg, uint8_t value,
                        bool skipValue)
{
    wire.beginTransmission(addr);
    size_t s = wire.write(reg);
    if (s == 1 && !skipValue)
    {
        s = wire.write(value);
    }

    size_t s1 = wire.endTransmission();
    if (s != 1 && s1 == 0)
    {
        return 1; // "data too long to fit in transmit buffer"
    }

    return s1;
}

uint8_t _write_registers(TwoWire &wire, uint8_t addr, uint8_t reg, size_t sz,
                         const uint8_t *value)
{
    wire.beginTransmission(addr);
    size_t s = wire.write(reg);
    if (s == 1 && sz > 0)
    {
        s = wire.write(value, sz);
    }

    size_t s1 = wire.endTransmission();
    if (s != sz && s1 == 0)
    {
        return 1; // "data too long to fit in transmit buffer"
    }

    return s1;
}

int8_t I2Cdev::writeBytes(TwoWire &w, uint8_t addr, uint8_t reg, size_t sz,
                          const uint8_t *buf)
{
    return _write_registers(w, addr, reg, sz, buf);
}

int8_t I2Cdev::writeWords(TwoWire &w, uint8_t addr, uint8_t reg, size_t sz,
                          const uint16_t *buf16)
{
    uint8_t *buf = (uint8_t *)malloc(sz * 2);
    uint8_t *p = buf;
    for (int i = 0; i < sz; i++)
    {
        *(p++) = buf16[i] >> 8;
        *(p++) = (uint8_t)buf16[i];
    }
    uint8_t r = writeBytes(w, addr, reg, sz * 2, buf);
    free(buf);
    return r;
}

int8_t I2Cdev::writeBit(TwoWire &wireObj, uint8_t addr, uint8_t reg, size_t bitNum,
                        uint8_t data)
{
    uint8_t b;
    int8_t r = readBytes(wireObj, addr, reg, 1, &b, I2Cdev::readTimeout);
    if (r != 1)
    {
        return r;
    }

    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeBytes(wireObj, addr, reg, 1, &b);
}

int8_t I2Cdev::writeBits(TwoWire &w, uint8_t addr, uint8_t reg, size_t bitn, size_t sz,
                         uint8_t v)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitn=4, sz=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    int8_t r = readBytes(w, addr, reg, 1, &b, I2Cdev::readTimeout);
    if (r != 1)
    {
        return r;
    }

    uint8_t mask = ((1 << sz) - 1) << (bitn - sz + 1);
    v <<= (bitn - sz + 1); // shift data into correct position
    v &= mask;             // zero all non-important bits in data
    b &= ~(mask);          // zero all important bits in existing byte
    b |= v;                // combine data with existing byte
    return writeBytes(w, addr, reg, 1, &b);
}

int8_t _read_registers(TwoWire &wire, uint8_t addr, uint8_t reg, uint8_t *v, size_t sz,
                       bool skipRegister)
{
    if (!skipRegister)
    {
        uint8_t s = _write_registers(wire, addr, reg, 0, NULL);
        if (s != 0)
        {
            return s;
        }
    }

    uint8_t r = wire.requestFrom(addr, sz);
    r = wire.readBytes(v, r);

    return r - sz;
}

uint8_t _read_register(TwoWire &wire, uint8_t addr, uint8_t reg, uint8_t &v,
                       bool skipRegister)
{
    uint8_t r = _read_registers(wire, addr, reg, &v, 1, skipRegister);
    if (r != 0)
    {
        return 1;
    }

    return 0;
}

int8_t I2Cdev::readBytes(TwoWire &wireObj, uint8_t addr, uint8_t reg, size_t length,
                         uint8_t *data, uint16_t timeout)
{ // timeout is unused right now
    return _read_registers(wireObj, addr, reg, data, length);
}

uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;
