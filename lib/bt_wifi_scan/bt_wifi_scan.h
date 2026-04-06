#pragma once

#include <Arduino.h>

#ifdef OSD_ENABLED
#include "DFRobot_OSD.h"

#ifdef WIFI_SCANNING_ENABLED
void scanWiFi(DFRobot_OSD &osd);
#endif

#ifdef BT_SCANNING_ENABLED
void scanBT(DFRobot_OSD &osd);
#endif

#endif // OSD_ENABLED
