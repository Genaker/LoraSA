#pragma once
#include <Wire.h>
#include <config.h>

struct
{
    String name;
    uint8_t address;
} known_i2c_devices[] = {
    {"HMC5883L", 0x1e}, {"QMC5883L", 0x0d}, {"MPU6050", 0x68}, {" last record ", 0}};

enum I2CDevices
{
    // powers of 2
    HMC5883L = 1,
    QMC5883L = 2,
    MPU6050 = 4
};

extern uint8_t wireDevices;
extern uint8_t wire1Devices;

#ifndef HELTEC
extern SPIClass &hspi;
#endif

// abstract away a reference to Serial vs Serial0 vs Serial1, so it compiles
#ifndef ARDUINO_USB_CDC_ON_BOOT
#define Uart0 Serial
#else
#define Uart0 Serial0
#endif

#if SOC_UART_NUM > 1
#define Uart1 Serial1
#else
#define Uart1 Uart0
#endif

bool initSPIs(Config &config);

bool initUARTs(Config &config);

bool initWires(Config &config);

#define I2CDEV_DEFAULT_READ_TIMEOUT 0

struct I2Cdev
{
    static uint16_t readTimeout;

    static int8_t writeBit(TwoWire &w, uint8_t addr, uint8_t reg, size_t bitn, uint8_t v);
    static int8_t writeBits(TwoWire &w, uint8_t addr, uint8_t reg, size_t bitn, size_t sz,
                            uint8_t v);
    static int8_t writeBytes(TwoWire &w, uint8_t addr, uint8_t reg, size_t sz,
                             const uint8_t *buf);
    static int8_t writeWords(TwoWire &w, uint8_t addr, uint8_t reg, size_t sz,
                             const uint16_t *buf);

    static int8_t readBytes(TwoWire &wireObj, uint8_t addr, uint8_t reg, size_t length,
                            uint8_t *data, uint16_t timeout = I2Cdev::readTimeout);
};

uint8_t _read_register(TwoWire &wire, uint8_t addr, uint8_t reg, uint8_t &v,
                       bool skipRegister = false);
int8_t _read_registers(TwoWire &wire, uint8_t addr, uint8_t reg, uint8_t *v, size_t sz,
                       bool skipRegister = false);
uint8_t _write_register(TwoWire &wire, uint8_t addr, uint8_t reg, uint8_t value,
                        bool skipValue = false);
uint8_t _write_registers(TwoWire &wire, uint8_t addr, uint8_t reg, size_t sz,
                         const uint8_t *value);
