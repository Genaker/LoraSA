#ifndef __GLOBAL_CONFIG_H__
#define __GLOBAL_CONFIG_H__

// frequency range in MHz to scan
#define FREQ_BEGIN 850
// TODO: if % RANGE_PER_PAGE  != 0
#define FREQ_END 950

// Measurement bandwidth. Allowed bandwidth values (in kHz) are:
// 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0, 46.9, 58.6,
// 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6 and 467.0
#define BANDWIDTH     467.0 

// Detection level from the 33 levels. The higher number is more sensitive
#define DEFAULT_DRONE_DETECTION_LEVEL 21


#define BUZZER_PIN  41
// REB trigger PIN
#define REB_PIN     42

#define WATERFALL_ENABLED true
#define WATERFALL_START 37

#endif