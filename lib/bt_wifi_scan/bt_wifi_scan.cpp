#include "bt_wifi_scan.h"

#ifdef OSD_ENABLED

// These globals are defined in main.cpp
extern long cycleCnt;
extern bool present;
extern bool scanFinished;

#ifdef WIFI_SCANNING_ENABLED
#include "WiFi.h"

void scanWiFi(DFRobot_OSD &osd)
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
#endif // WIFI_SCANNING_ENABLED

#ifdef BT_SCANNING_ENABLED
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>

void scanBT(DFRobot_OSD &osd)
{
    osd.clear();
    osd.displayString(14, 2, "Scanning BT...");
    cycleCnt++;

    BLEDevice::init("");
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(0x50);
    pBLEScan->setWindow(0x30);

#ifdef SERIAL_PRINT
    Serial.printf("Start BLE scan for %d seconds...\n", BT_SCAN_TIME);
#endif

    BLEScanResults foundDevices = pBLEScan->start(BT_SCAN_TIME);
    int count = foundDevices.getCount();
#ifdef PRINT_DEBUG
    Serial.printf("Found devices: %d  \n", count);
#endif
    present = false;
    for (int i = 0; i < count; i++)
    {
        BLEAdvertisedDevice device = foundDevices.getDevice(i);
        String currDevAddr = device.getAddress().toString().c_str();
        String deviceName;
        if (device.haveName())
        {
            deviceName = device.getName().c_str();
        }
        else
        {
            deviceName = currDevAddr;
        }

#ifdef PRINT_DEBUG
        Serial.printf("Found device #%s/%s with RSSI: %d \n", currDevAddr, deviceName,
                      device.getRSSI());
#endif
        osd.displayString(i + 1, 1,
                          "BT:" + deviceName + ":" + String(device.getRSSI()) + " \n");
    }
#ifdef PRINT_DEBUG
    Serial.println("Scan done!");
    Serial.printf("Cycle counter: %d, Free heap: %d \n", cycleCnt, ESP.getFreeHeap());
#endif
    osd.displayChar(14, 1, 0x10f);
    scanFinished = true;
}
#endif // BT_SCANNING_ENABLED

#endif // OSD_ENABLED
