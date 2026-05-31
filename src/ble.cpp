#include "defines.h"

#ifdef ENABLE_BLE

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

static BLEServer *s_server = nullptr;
static BLECharacteristic *s_cmd_char = nullptr;
static BLECharacteristic *s_status_char = nullptr;
static bool s_connected = false;

class ServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *) override
    {
        s_connected = true;
        Serial.println("[BLE] Client connected");
    }

    void onDisconnect(BLEServer *srv) override
    {
        s_connected = false;
        Serial.println("[BLE] Client disconnected — restarting advertising");
        srv->startAdvertising();
    }
};

class CmdCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *c) override
    {
        std::string val = c->getValue();
        if (!val.empty())
        {
            Serial.printf("[BLE] CMD received: %s\n", val.c_str());
        }
    }
};

void BLE_Init()
{
    BLEDevice::init(DEVICE_ID);

    s_server = BLEDevice::createServer();
    s_server->setCallbacks(new ServerCallbacks());

    BLEService *svc = s_server->createService(BLE_SERVICE_UUID);

    s_cmd_char = svc->createCharacteristic(
        BLE_CMD_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE);
    s_cmd_char->setCallbacks(new CmdCallbacks());

    s_status_char = svc->createCharacteristic(
        BLE_STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY);
    s_status_char->addDescriptor(new BLE2902());

    svc->start();

    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.printf("[BLE] Advertising as \"%s\"\n", DEVICE_ID);
}

bool BLE_IsConnected() { return s_connected; }

void BLE_Notify(const char *payload)
{
    if (!s_connected || !s_status_char)
        return;
    s_status_char->setValue(payload);
    s_status_char->notify();
}

#endif
