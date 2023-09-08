#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor() {}

void BatteryMonitor::begin() {
    lc.begin();
    lc.setThermistorB(3950);
    lc.setPackSize(LC709203F_APA_500MAH);
    lc.setAlarmVoltage(3.8);
    Serial.println("Battery Monitor Setup");
}

float BatteryMonitor::getVoltage() {
    return lc.cellVoltage();
}

float BatteryMonitor::getPercentage() {
    return lc.cellPercent();
}





