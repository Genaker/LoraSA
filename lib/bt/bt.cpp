#include "bt.h"

std::atomic<bool> bleDeviceConnected{false};

#ifdef BT_MOBILE
#define SERVICE_UUID "00001234-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00001234-0000-1000-8000-00805f9b34ac"

#ifndef BT_NM
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLEAdvertising *pAdvertising = NULL;

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer) { bleDeviceConnected = true; };

    void onDisconnect(BLEServer *pServer)
    {
        bleDeviceConnected = false;
        BLEDevice::startAdvertising();
    }
};

#else
#include <NimBLEDevice.h>

NimBLEServer *pServer = nullptr;
NimBLECharacteristic *pCharacteristic = nullptr;
NimBLEAdvertising *pAdvertising = nullptr;

class BTServerCallbacks : public NimBLEServerCallbacks
{
  public:
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
    {
        bleDeviceConnected = true;
        Serial.printf("Device Connected | Free Heap: %d kByte\n",
                      ESP.getFreeHeap() / 1000);
        Serial.printf("Client address: %s\n", connInfo.getAddress().toString().c_str());

        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
    }

    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo,
                      int reason) override
    {
        bleDeviceConnected = false;
        Serial.println("Device Disconnected");
        NimBLEDevice::startAdvertising();
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override
    {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU,
                      connInfo.getConnHandle());
    }
} BTServerCallbacks;

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
    {
        Serial.printf("%s : onRead(), value: %s\n",
                      pCharacteristic->getUUID().toString().c_str(),
                      pCharacteristic->getValue().c_str());
    }

    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
    {
        Serial.printf("%s : onWrite(), value: %s\n",
                      pCharacteristic->getUUID().toString().c_str(),
                      pCharacteristic->getValue().c_str());
    }

    void onStatus(NimBLECharacteristic *pCharacteristic, int code) override
    {
#ifdef COMPASS_DEBUG
        Serial.printf("Notification/Indication return code: %d, %s\n", code,
                      NimBLEUtils::returnCodeToString(code));
#endif
    }

    void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo,
                     uint16_t subValue) override
    {
        std::string str = "Client ID: ";
        str += connInfo.getConnHandle();
        str += " Address: ";
        str += connInfo.getAddress().toString();
        if (subValue == 0)
        {
            str += " Unsubscribed to ";
        }
        else if (subValue == 1)
        {
            str += " Subscribed to notifications for ";
        }
        else if (subValue == 2)
        {
            str += " Subscribed to indications for ";
        }
        else if (subValue == 3)
        {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID());

        Serial.printf("%s\n", str.c_str());
    }
} chrCallbacks;

class DescriptorCallbacks : public NimBLEDescriptorCallbacks
{
    void onWrite(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override
    {
        std::string dscVal = pDescriptor->getValue();
        Serial.printf("Descriptor written value: %s\n", dscVal.c_str());
    }

    void onRead(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override
    {
        Serial.printf("%s Descriptor read\n", pDescriptor->getUUID().toString().c_str());
    }
} dscCallbacks;

#endif // BT_NM

void initBT()
{
#ifdef BT_NM
    NimBLEDevice::init("RSSI_Radar");

    String macAddress = NimBLEDevice::getAddress().toString().c_str();
    Serial.println("Bluetooth MAC Address: " + macAddress);

    pServer = NimBLEDevice::createServer();

    if (!pServer)
    {
        Serial.println("Failed to create BLE Server");
    }

    pServer->setCallbacks(&BTServerCallbacks);

    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pCharacteristic->setCallbacks(&chrCallbacks);

    pService->start();

    pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setName("ESP32_RSSI_Radar");
    pAdvertising->enableScanResponse(true);

    pAdvertising->start();

    Serial.println("BLE server started.");
#else
    BLEDevice::init("ESP32_RADAR");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ |
                                 BLECharacteristic::PROPERTY_WRITE |
                                 BLECharacteristic::PROPERTY_NOTIFY);

    pCharacteristic->setValue("Hello from ESP32");
    pService->start();

    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinInterval(300);
    pAdvertising->setMaxInterval(350);
    BLEDevice::startAdvertising();

    Serial.println("BLE server is ready!");
#endif
}

#endif // BT_MOBILE

void sendBTData(float heading, float rssi)
{
    String data =
        "RSSI_HEADING: '{H:" + String(heading) + ",RSSI:-" + String(rssi) + "}'";

#ifdef COMPASS_DEBUG
    Serial.println("Sending data: " + data);
#endif
#ifdef BT_MOBILE
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();
#endif
}

void sendBTData(Message &msg)
{
    if (msg.type != SCAN_HEADING_MAX && msg.type != SCAN_MAX_RESULT &&
        msg.type != SCAN_RESULT)
    {
        Serial.println("Unsupported message type: " + String(msg.type));
        return;
    }

    String data = "{\"SCAN_RESULT\":{\"Hmin\":" + String(msg.payload.dump.heading_min) +
                  ",\"Hmax\":" + String(msg.payload.dump.heading_max) + ",\"Spectrum\":[";

    for (int i = 0; i < msg.payload.dump.sz; i++)
    {
        data += String(i == 0 ? "" : ",") +
                "{\"F\":" + String(msg.payload.dump.freqs_khz[i]) +
                ",\"R\":" + String(msg.payload.dump.rssis[i]) +
                (msg.payload.dump.rssis2 == NULL
                     ? ""
                     : ",\"R2\":" + String(msg.payload.dump.rssis2[i])) +
                "}";
    }

    data += "]}}";
#ifdef COMPASS_DEBUG
    Serial.println("Sending data: " + data);
#endif
#ifdef BT_MOBILE
    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();
#endif
}
