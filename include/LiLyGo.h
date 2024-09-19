
#define UNUSED_PIN (0)
// LilyGo defined

// Check this LiLyGo file LoraSA2\include\utilities.h

#define I2C_SDA 18
#define I2C_SCL 17
#define OLED_RST UNUSED_PIN

#define RADIO_SCLK_PIN 5
#define RADIO_MISO_PIN 3
#define RADIO_MOSI_PIN 6
#define RADIO_CS_PIN 7

#define SDCARD_MOSI 11
#define SDCARD_MISO 2
#define SDCARD_SCLK 14
#define SDCARD_CS 13

#define BOARD_LED 37
#define LED_ON HIGH

#define BUTTON_PIN 0
#define ADC_PIN 1

#define RADIO_RST_PIN 8

#define RADIO_DIO1_PIN 33
#define RADIO_BUSY_PIN 34

#ifdef USING_SX1280PA
#define RADIO_DIO1_PIN 9  // SX1280 DIO1 = IO9
#define RADIO_BUSY_PIN 36 // SX1280 BUSY = IO36
#define RADIO_RX_PIN 21
#define RADIO_TX_PIN 10
#define BUTTON_PIN 0
#endif

// Define for our code
#define RST_OLED UNUSED_PIN
#define LED BOARD_LED

#include "RadioLib.h"
// make sure the power off button works when using RADIOLIB_OR_HALT
// (See RadioLib_convenience.h)
#define RADIOLIB_DO_DURING_HALT heltec_delay(10)
#include "RadioLib_convenience.h"
#ifdef HELTEC_NO_DISPLAY
#define HELTEC_NO_DISPLAY_INSTANCE
#else
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#include "OLEDDisplayUi.h"
#include "SSD1306Wire.h"
#endif
#define ARDUINO_heltec_wifi_32_lora_V3
#ifndef HELTEC_NO_RADIO_INSTANCE
#ifndef ARDUINO_heltec_wifi_32_lora_V3
// Assume MISO and MOSI being wrong when not using Heltec's board definition
// and use hspi to make it work anyway. See heltec_setup() for the actual SPI setup.
#include <SPI.h>
SPIClass *hspi = new SPIClass(2);
SX1262 radio = new Module(SS, DIO1, RST_LoRa, BUSY_LoRa, *hspi);
#else // ARDUINO_heltec_wifi_32_lora_V3
#ifdef USING_SX1280PA
SX1280 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif // end USING_SX1280PA
#ifdef USING_SX1262
// Default SPI on pins from pins_arduino.h
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif // end USING_SX1262
#endif // end ARDUINO_heltec_wifi_32_lora_V3
#endif // end HELTEC_NO_RADIO_INSTANCE

void heltec_loop() {}

void heltec_led(int led) {}

void heltec_deep_sleep() {}

void heltec_delay(int millisec) { delay(millisec); }

#ifndef HELTEC_NO_DISPLAY_INSTANCE
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

#ifdef HELTEC_WIRELESS_STICK
#define DISPLAY_GEOMETRY GEOMETRY_64_32
#else
#define DISPLAY_GEOMETRY GEOMETRY_128_64
#endif
SSD1306Wire display(0x3c, 18, 17, DISPLAY_GEOMETRY);
PrintSplitter both(Serial, display);
#else
Print &both = Serial;
#endif
// some fake pin
#define BUTTON BUTTON_PIN
#include "HotButton.h"
HotButton button(BUTTON);

// This file contains a binary patch for the SX1262
#include "modules/SX126x/patches/SX126x_patch_scan.h"

void heltec_display_power(bool on)
{
#ifndef HELTEC_NO_DISPLAY_INSTANCE
    if (on)
    {
#ifdef HELTEC_WIRELESS_STICK
        // They hooked the display to "external" power, and didn't tell anyone
        heltec_ve(true);
        delay(5);
#endif
        pinMode(RST_OLED, OUTPUT);
        digitalWrite(RST_OLED, HIGH);
        delay(1);
        digitalWrite(RST_OLED, LOW);
        delay(20);
        digitalWrite(RST_OLED, HIGH);
    }
    else
    {
#ifdef HELTEC_WIRELESS_STICK
        heltec_ve(false);
#else
        display.displayOff();
#endif
    }
#endif
}
void heltec_setup()
{
    Serial.begin(115200);
    Serial.println("LILYGO BOARD");

#if defined(ARDUINO_ARCH_ESP32)
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
#elif defined(ARDUINO_ARCH_STM32)
    SPI.setMISO(RADIO_MISO_PIN);
    SPI.setMOSI(RADIO_MOSI_PIN);
    SPI.setSCLK(RADIO_SCLK_PIN);
    SPI.begin();
#endif

#ifndef ARDUINO_heltec_wifi_32_lora_V3
    hspi->begin(SCK, MISO, MOSI, SS);
#endif
#ifndef HELTEC_NO_DISPLAY_INSTANCE
    heltec_display_power(true);
    display.init();
    display.setContrast(200);
    display.flipScreenVertically();
#endif
}
