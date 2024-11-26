#ifndef __GLOBAL_CONFIG_H__
#define __GLOBAL_CONFIG_H__

#include "utilities.h"

#ifndef FREQ_BEGIN
// frequency range in MHz to scan
#define FREQ_BEGIN 850
#endif

#ifndef FREQ_END
// TODO: if % RANGE_PER_PAGE  != 0
#define FREQ_END 950
#endif

// Measurement bandwidth. Allowed bandwidth values (in kHz) are:
// 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0, 46.9, 58.6,
// 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6 and 467.0
#define BANDWIDTH 467.0
#define BANDWIDTH_SX1280 406.

// Detection level from the 33 levels. The higher number is more sensitive
#define DEFAULT_DRONE_DETECTION_LEVEL 18

#define BUZZER_PIN 41

#ifdef LILYGO
#define BUZZER_PIN 45
#endif
#ifdef T3_V1_6_SX1276
#define BUZZER_PIN 35
#endif

// REB trigger PIN
#define REB_PIN 42
#ifdef T3_V1_6_SX1276
#define REB_PIN 35
#endif

#define WATERFALL_ENABLED true
#define WATERFALL_START 37

#ifdef LILYGO
#define LED BOARD_LED
#endif // end not LILYGO
#ifdef T3_V1_6_SX1276
#define LED BOARD_LED
#endif

#endif
