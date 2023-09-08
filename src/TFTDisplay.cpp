#include "TFTDisplay.h"

#include "Config.h"
#include "constants.h"

TFTDisplay::TFTDisplay(uint8_t cs, uint8_t dc, uint8_t rst)
    : tft_(Adafruit_ST7789(cs, dc, rst)) {};

void TFTDisplay::begin() {
    // turn on backlite
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);

    // turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(10);

    // initialize TFT
    tft_.init(135, 240); // Init ST7789 240x135
    tft_.setRotation(1);
    tft_.fillScreen(ST77XX_BLACK);

    Serial.println("TFT Initialized");
}

void TFTDisplay::tick(BatteryMonitor bm, BLEServiceManager sm, Adafruit_BNO055 imu) {
    //  Only update screen every 100ms
    unsigned long now = millis();
    if (now > nextUpdate_) {
        nextUpdate_ = now + DRAW_INTERVAL;
        //drawScreen(bm, sm, imu);
    }
}

void TFTDisplay::drawScreen(BatteryMonitor bm, BLEServiceManager sm, Adafruit_BNO055 imu) {
    float cellPercent = bm.getPercentage();
    tft_.setCursor(0, 0);
    tft_.setTextWrap(false);
    drawBattery(cellPercent);
    drawBLEData(sm);
    drawIMUData(imu);
}

void TFTDisplay::clearScreen() {
    tft_.fillScreen(ST77XX_BLACK);
}

void TFTDisplay::drawBattery(float cellPercent) {
    uint16_t battColor = ST77XX_GREEN;
    if (cellPercent < 25) {
        battColor = ST77XX_RED;
    } else if (cellPercent < 50) {
        battColor = ST77XX_ORANGE;
    }
    tft_.drawRect(200, 120, 30, 10, ST77XX_WHITE);
    tft_.fillRect(201, 121, cellPercent / 100.0 * 28, 8, battColor);
    tft_.fillRect(230, 123, 3, 4, ST77XX_WHITE);
}

void TFTDisplay::drawIMUData(Adafruit_BNO055 imu) {
    tft_.setTextSize(1);
    tft_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft_.printf("X: %d, Accell: %.2f\n", (int)imu.getVector(Adafruit_BNO055::VECTOR_EULER).x(), 
                                     imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).x());
    tft_.printf("Y: %d, Accell: %.2f\n", (int)imu.getVector(Adafruit_BNO055::VECTOR_EULER).y(), 
                                     imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).y());
    tft_.printf("Z: %d, Accell: %.2f\n", (int)imu.getVector(Adafruit_BNO055::VECTOR_EULER).z(), 
                                     imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER).z());
}

void TFTDisplay::drawBLEData(BLEServiceManager sm) {
    bool connected;
    if (sm.getConnectedCount() > 0) {
        connected = true;
    } else {
        connected = false;
    }

    tft_.setTextSize(2);
    tft_.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    //TODO Slow
    //tft_.printf("%s\n", Config::getInstance().getBLEDeviceName(BLE_ADV_NAME).c_str());
    if (connected) {
        tft_.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
        tft_.println("Connected    ");
    } else {
        tft_.setTextColor(ST77XX_RED, ST77XX_BLACK);
        tft_.println("Disconnected");
    }
}
