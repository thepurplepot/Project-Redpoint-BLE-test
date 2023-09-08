#include <Arduino.h>
// BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEAdvertising.h>
// TFT
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
// Battery Monitor
#include <Adafruit_LC709203F.h>
// IMU
#include <Adafruit_BNO055.h>

#include "BLEIMUService.h"
#include "BLEAddressService.h"
#include "BLEServiceManager.h"
#include "BLEUARTService.h"
#include "Config.h"
#include "TFTDisplay.h"
#include "BatteryMonitor.h"
#include "constants.h"

static Adafruit_BNO055 bno055;
static BLEServiceManager* bleServiceManager;
static TFTDisplay tft(TFT_CS, TFT_DC, TFT_RST);
static BatteryMonitor bm;

#define SPICLK 22
#define SPIDAT 23

void setup() {
    Serial.begin(115200);

    // Setup battery monitor
    bm.begin();

    // Setup TFT
    tft.begin();

    // Get IMU
    Wire.begin(SPIDAT, SPICLK);
    bno055.begin();

    // Create the BLE Device
    std::string bleDeviceName =
        Config::getInstance().getBLEDeviceName(BLE_ADV_NAME);

    BLEDevice::init(bleDeviceName.c_str());
    bleServiceManager = new BLEServiceManager();
    new BLEIMUServiceHandler(*bleServiceManager, bno055);
    new BLEAddressServiceHandler(*bleServiceManager);
    new BLEUARTServiceHandler(*bleServiceManager);

    Serial.printf("Starting BLE (device name=%s)\n", bleDeviceName.c_str());
    bleServiceManager->start();
}

void loop() {
    bleServiceManager->tick();
    tft.tick(bm, *bleServiceManager, bno055);
}
