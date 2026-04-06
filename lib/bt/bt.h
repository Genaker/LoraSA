#pragma once

#include <Arduino.h>
#include <atomic>
#include <comms.h>

extern std::atomic<bool> bleDeviceConnected;

#ifdef BT_MOBILE
void initBT();

#ifndef BT_NM
#include <BLEDevice.h>
#include <BLEServer.h>
extern BLEServer *pServer;
extern BLECharacteristic *pCharacteristic;
extern BLEAdvertising *pAdvertising;
#else
#include <NimBLEDevice.h>
extern NimBLEServer *pServer;
extern NimBLECharacteristic *pCharacteristic;
extern NimBLEAdvertising *pAdvertising;
#endif

#endif // BT_MOBILE

void sendBTData(float heading, float rssi);
void sendBTData(Message &msg);
