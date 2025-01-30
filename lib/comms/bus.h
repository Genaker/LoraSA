#pragma once
#include <config.h>

struct
{
    String name;
    uint8_t address;
} known_i2c_devices[] = {{"HMC5883L", 0x1e}, {"QMC5883L", 0x0d}, {" last record ", 0}};

enum I2CDevices
{
    // powers of 2
    HMC5883L = 1,
    QMC5883L = 2
};

extern uint8_t wireDevices;
extern uint8_t wire1Devices;

extern SPIClass &hspi;

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
