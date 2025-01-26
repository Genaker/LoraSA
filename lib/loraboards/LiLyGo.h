#ifndef __LILY_GO_H
#define __LILY_GO_H

#include "RadioLib.h"

#ifdef HELTEC

#ifdef HELTEC_NO_DISPLAY
#define HELTEC_NO_DISPLAY_INSTANCE
#else
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#include "SSD1306Wire.h"
#endif

#define RADIO_TYPE SX1262

#ifndef HELTEC_NO_RADIO_INSTANCE
#ifndef ARDUINO_heltec_wifi_32_lora_V3
// Assume MISO and MOSI being wrong when not using Heltec's board definition
// and use vspi to make it work anyway. See heltec_setup() for the actual SPI setup.
#include <SPI.h>
extern SPIClass *vspi;
#define RADIO_MODULE_INIT() new Module(SS, DIO1, RST_LoRa, BUSY_LoRa, *vspi);
#else  // ARDUINO_heltec_wifi_32_lora_V3
#endif // end ARDUINO_heltec_wifi_32_lora_V3
#endif // end HELTEC_NO_RADIO_INSTANCE

extern RADIO_TYPE radio;

#ifndef HELTEC_NO_DISPLAY_INSTANCE
#ifdef HELTEC_WIRELESS_STICK
#define DISPLAY_GEOMETRY GEOMETRY_64_32
#else
#define DISPLAY_GEOMETRY GEOMETRY_128_64
#endif
#define SCREEN_ADDRESS 0x3C

#define DISPLAY_TYPE SSD1306Wire
#define DISPLAY_INIT() SSD1306Wire(SCREEN_ADDRESS, I2C_SDA, I2C_SCL, DISPLAY_GEOMETRY)
// SH1106Wire display(0x3c, I2C_SDA, I2C_SCL, DISPLAY_GEOMETRY);
#else
#define DISPLAY_TYPE void *
#endif

// This file contains a binary patch for the SX1262
#include "modules/SX126x/patches/SX126x_patch_scan.h"

#else
// Define for our code
#define RST_OLED UNUSED_PIN
#define LED BOARD_LED

#include "RadioLib.h"
// make sure the power off button works when using RADIOLIB_OR_HALT
// (See RadioLib_convenience.h)
#define RADIOLIB_DO_DURING_HALT heltec_delay(10)
#include "RadioLib_convenience.h"

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
// #include "OLEDDisplayUi.h"
// #include "SH1106Wire.h"
// #include "SSD1306Brzo.h"
#include "SSD1306Wire.h"

#ifdef USING_SX1280PA
#define RADIO_TYPE SX1280
#define RADIO_MODULE_INIT()                                                              \
    new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif // end USING_SX1280PA
#ifdef USING_SX1262
// Default SPI on pins from pins_arduino.h
#define RADIO_TYPE SX1262
#define RADIO_MODULE_INIT()                                                              \
    new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN)
#endif // end USING_SX1262
#ifdef USING_LR1121
// Default SPI on pins from pins_arduino.h
#define RADIO_TYPE LR1121
#define RADIO_MODULE_INIT()                                                              \
    new Module(RADIO_CS_PIN, RADIO_DIO9_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif // end USING_LR1121
#ifdef USING_SX1276
// Default SPI on pins from pins_arduino.h
#define RADIO_TYPE SX1276
#define RADIO_MODULE_INIT()                                                              \
    new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif // end USING_SX1276

extern RADIO_TYPE radio;

void heltec_led(int led);

void heltec_deep_sleep(int sleep_seconds = 0);

void heltec_delay(int millisec);

/**
 * @class PrintSplitter
 * @brief A class that splits the output of the Print class to two different
 *        Print objects.
 *
 * The PrintSplitter class is used to split the output of the Print class to two
 * different Print objects. It overrides the write() function to write the data
 * to both Print objects.
 */
class PrintSplitter : public Print
{
  public:
    PrintSplitter(Print &_a, Print &_b) : a(_a), b(_b) {}
    size_t write(uint8_t c)
    {
        a.write(c);
        return b.write(c);
    }
    size_t write(const char *str)
    {
        a.write(str);
        return b.write(str);
    }

  private:
    Print &a;
    Print &b;
};

#define DISPLAY_GEOMETRY GEOMETRY_128_64
#define SCREEN_ADDRESS 0x3C

#define DISPLAY_TYPE SSD1306Wire
#define DISPLAY_INIT() SSD1306Wire(SCREEN_ADDRESS, I2C_SDA, I2C_SCL, DISPLAY_GEOMETRY)
// SH1106Wire display(0x3c, I2C_SDA, I2C_SCL, DISPLAY_GEOMETRY);

#define BOTH_TYPE PrintSplitter
#define BOTH_INIT() PrintSplitter(Serial, display)

extern DISPLAY_TYPE display;
extern BOTH_TYPE both;

// some fake pin
#ifdef T3_V1_6_SX1276
#define BUTTON_PIN 22
#endif

// This file contains a binary patch for the SX1262
#include "modules/SX126x/patches/SX126x_patch_scan.h"

#define BUTTON BUTTON_PIN
#include "HotButton.h"
extern HotButton button;

void heltec_loop();

void heltec_display_power(bool on);

void heltec_setup();
#endif
#endif
