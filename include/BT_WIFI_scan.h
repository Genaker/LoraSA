#pragma once

#ifdef WIFI_SCANNING_ENABLED
#include "WiFi.h"
#endif
#ifdef BT_SCANNING_ENABLED
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>
#endif
#include "DFRobot_OSD.h"

void setOSD() {}
// TODO: Make Async Scan
// https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/scan-examples.html#async-scan
void scanWiFi(void)
{
    osd.clear();
    osd.displayString(14, 2, "Scanning WiFi..");
    int n = WiFi.scanNetworks();
#ifdef PRINT_DEBUG
    Serial.println("scan done");
    if (n == 0)
    {
        Serial.println("no networks found");
    }
#endif
    if (n > 0)
    {
#ifdef PRINT_DEBUG
        Serial.print(n);
        Serial.println(" networks found");
#endif
        for (int i = 0; i < n; ++i)
        {
// Print SSID and RSSI for each network found
#ifdef PRINT_DEBUG
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
#endif
            osd.displayString(i + 1, 1,
                              "WF:" + String((WiFi.SSID(i) + ":" + WiFi.RSSI(i))));
        }
    }
    osd.displayChar(14, 1, 0x10f);
}
