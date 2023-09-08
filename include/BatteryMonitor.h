#pragma once
#ifndef _BATTERYMONITOR_H
#define _BATTERYMONITOR_H

#include <Adafruit_LC709203F.h>

class BatteryMonitor {
 public:
  BatteryMonitor();
  void begin();
  float getVoltage();
  float getPercentage();

 private:
  Adafruit_LC709203F lc;
};

#endif  // _BATTERYMONITOR_H