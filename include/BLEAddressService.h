#pragma once
#ifndef _BLEADDRESSSERVICE_H_
#define _BLEADDRESSSERVICE_H_

#include "BLEServiceManager.h"

const char BLE_ADDRESS_SERVICE_UUID[] =
    "709F0001-37E3-439E-A338-23F00067988B";
const char BLE_ADDRESS_CHAR_UUID[] = "709F0002-37E3-439E-A338-23F00067988B";
const char BLE_DEVICE_NAME_CHAR_UUID[] = "709F0003-37E3-439E-A338-23F00067988B";

class BLEDeviceNameCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *ch);
};

class BLEAddressServiceHandler : public BLEServiceHandler {
  public:
    BLEAddressServiceHandler(BLEServiceManager &manager);
};

#endif  // _BLEADDRESSSERVICE_H_
