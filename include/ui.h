#pragma once

#ifdef Vision_Master_E290
#include "HT_DEPG0290BxS800FxX_BW.h"
#else
#include "Adafruit_SSD1306.h"
// #include "OLEDDisplayUi.h"
#include <Arduino.h>
#endif

#include <charts.h>
#include <scan.h>

// #include <heltec_unofficial.h>

// (optional) major and minor tick-marks at x MHz
#define MAJOR_TICKS 10
#define MINOR_TICKS 5

#define ONE_MILLISEC 1
#define ONE_SEC_MIL 1000
#define ONE_MINUTE_MIL 60 * 1000

#define MAJOR_TICK_LENGTH 2
#define MINOR_TICK_LENGTH 1
// WEIGHT of the x-axis line
#define X_AXIS_WEIGHT 1

#define ROW_STATUS_TEXT (64 - 10)

// The number of the spectrum screen lines = width of screen
// Resolution of the scan is limited by 128-pixel screen
#define STEPS 128

#define SCREEN_HEIGHT 64 // ???? not used

extern void UI_Init(Display_t *);
extern void UI_clearPlotter(void);
extern void UI_clearTopStatus(void);
extern void UI_drawCursor(int16_t);

struct StatusBar : Chart
{
    Scan &r;
    bool ui_initialized;
    uint16_t scan_progress_count;

    StatusBar(Display_t &d, uint16_t x, uint16_t y, uint16_t w, Scan &r)
        : Chart(d, x, y, w, LABEL_HEIGHT), r(r), ui_initialized(false),
          scan_progress_count(0) {};

    virtual void clearStatus();
    virtual void draw() override;
};
