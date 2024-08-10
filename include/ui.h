
#ifndef __UI_H__
#define __UI_H__

#include "OLEDDisplayUi.h"
#include "SSD1306Wire.h"
#include <Arduino.h>

// #include <heltec_unofficial.h>

// (optional) major and minor tick-marks at x MHz
#define MAJOR_TICKS 10
#define MINOR_TICKS 5

#define ONE_MILLISEC 1

// Prints debug information and the scan measurement bins from the SX1262 in
// hex Change spectrum plot values at once or by line
#define ANIMATED_RELOAD false

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
extern void UI_Init(SSD1306Wire *);
extern void UI_displayDecorate(int, int, bool);
extern void UI_setLedFlag(bool);
extern void UI_clearPlotter(void);
extern void UI_drawCursor(int16_t);

#endif // __UI_H__
