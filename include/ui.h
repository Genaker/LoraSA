
#ifndef __UI_H__
#define __UI_H__

#include <Arduino.h>
#include "SSD1306Wire.h"
#include "OLEDDisplayUi.h"

// #include <heltec_unofficial.h>

// (optional) major and minor tickmarks at x MHz
#define MAJOR_TICKS 10
#define MINOR_TICKS 5

#define ONE_MILLISEC 1

// Prints debug information and the scan measurement bins from the SX1262 in hex
//#define PRINT_DEBUG
// Change spectrum plot values at once or by line
#define ANIMATED_RELOAD true

#define MAJOR_TICK_LENGTH 2
#define MINOR_TICK_LENGTH 1
// WEIGHT of the x-asix line
#define X_AXIS_WEIGHT 1

#define STATUS_TEXT_TOP (64 - 10)


// The number of the spectrum screen lines = width of screen
// Resolution of the scan is limited by 128-pixel screen
#define STEPS 128


#define SCREAN_HEIGHT 64

// publish functions 
extern void UI_Init(SSD1306Wire*);
extern void UI_displayDecorate(int  , int  , bool );
extern void UI_setLedFlag( bool);
extern void UI_clearPlotter(void);
extern void UI_drawCurrsor(int16_t);

#endif // __UI_H__

