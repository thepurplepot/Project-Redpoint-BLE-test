#pragma once
#ifndef _TFTDISPLAY_H
#define _TFTDISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include "BatteryMonitor.h"
#include "BLEServiceManager.h"

#define DRAW_INTERVAL 100

class TFTDisplay {
  public:
    TFTDisplay(uint8_t cs, uint8_t dc, uint8_t rst);
    void begin();
    void tick(BatteryMonitor bm, BLEServiceManager sm, Adafruit_BNO055 imu);
    void drawScreen(BatteryMonitor bm, BLEServiceManager sm, Adafruit_BNO055 imu);
    void clearScreen();
    void drawBattery(float cellPercent);
    void drawIMUData(Adafruit_BNO055 imu);
    void drawBLEData(BLEServiceManager sm);

  private:
    Adafruit_ST7789 tft_;
    unsigned long nextUpdate_ = 0;
};

#endif  // _TFTDISPLAY_H