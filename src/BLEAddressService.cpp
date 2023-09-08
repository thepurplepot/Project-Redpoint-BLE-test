#include "BLEAddressService.h"

#include <Arduino.h>
#include <BLE2902.h>

#include <string>
#include <sstream>

#include "Config.h"
#include "utils.h"

void BLEDeviceNameCallbacks::onWrite(BLECharacteristic *ch) {
    std::string deviceName = ch->getValue();
    Config::getInstance().setBLEDeviceName(deviceName);
    esp_err_t errRc = ::esp_ble_gap_set_device_name(deviceName.c_str());
    if (errRc != ESP_OK) {
        log_e("esp_ble_gap_set_device_name: rc=%d", errRc);
    }
    ch->setValue((uint8_t *)deviceName.data(), deviceName.length());
    ch->notify();
}

BLEAddressServiceHandler::BLEAddressServiceHandler(BLEServiceManager &manager)
    : BLEServiceHandler(manager, BLE_ADDRESS_SERVICE_UUID) {
    auto *addressChar = bleService_->createCharacteristic(
        BLE_ADDRESS_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
    addressChar->addDescriptor(new BLE2902());

    std::string address = getBTAddress();
    Serial.printf("BT address = %s\n", address.c_str());
    addressChar->setValue((uint8_t *)address.data(), address.length());

    auto *deviceNameChar = bleService_->createCharacteristic(
        BLE_DEVICE_NAME_CHAR_UUID, BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_NOTIFY);
    std::string deviceName = Config::getInstance().getBLEDeviceName("");
    Serial.printf("Device name = %s\n", deviceName.c_str());
    deviceNameChar->setValue((uint8_t *)deviceName.data(), deviceName.length());
    deviceNameChar->setCallbacks(new BLEDeviceNameCallbacks());
}

