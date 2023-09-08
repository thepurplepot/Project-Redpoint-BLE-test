#include "BLEServiceManager.h"

#include <HardwareSerial.h>

#include <algorithm>

BLEServiceManager::BLEServiceManager() : bleServer(*BLEDevice::createServer()) {
    bleServer.setCallbacks(this);
    adv_ = bleServer.getAdvertising();
}

uint32_t BLEServiceManager::getConnectedCount() { return bleServer.getConnectedCount(); }

void BLEServiceManager::addServiceHandler(BLEServiceHandler &handler) {
    serviceHandlers_.push_back(&handler);
    Serial.printf("Advertising %s\n", handler.uuid.c_str());
    adv_->addServiceUUID(handler.uuid.c_str());
}

void BLEServiceManager::start() {
    std::for_each(serviceHandlers_.begin(), serviceHandlers_.end(),
                    std::mem_fun(&BLEServiceHandler::start));
    adv_->start();
}

void BLEServiceManager::tick() {
    uint32_t connectionCount = bleServer.getConnectedCount();
    if (connectionCount > 0) {
        std::for_each(serviceHandlers_.begin(), serviceHandlers_.end(),
                    std::mem_fun(&BLEServiceHandler::tick));
    }
    if (connectionCount == 0 && hasBeenConnected_) {
        delay(500);
        Serial.println("Restart BLE advertising");
        bleServer.startAdvertising();
    }
}

void BLEServiceManager::onConnect(BLEServer *server) {
    Serial.println("BLE connected");
    hasBeenConnected_ = true;
};

void BLEServiceManager::onDisconnect(BLEServer *server) { Serial.println("BLE disconnected"); }

