#pragma once

#ifdef Vision_Master_E290
#include "HT_DEPG0290BxS800FxX_BW.h"
#else
#include "OLEDDisplayUi.h"
#include "SSD1306Wire.h"
#include <Arduino.h>
#endif

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

// publish functions
#ifdef Vision_Master_E290
extern void UI_Init(DEPG0290BxS800FxX_BW *);
#else
extern void UI_Init(SSD1306Wire *);
#endif
extern void UI_displayDecorate(int, int, bool);
extern void UI_setLedFlag(bool);
extern void UI_clearPlotter(void);
extern void UI_clearTopStatus(void);
extern void UI_drawCursor(int16_t);
