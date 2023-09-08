#include "Config.h"

#include "FS.h"
#include "SPIFFS.h"

#define FORMAT_SPIFFS_IF_FAILED true

Config& Config::getInstance() {
    static Config instance;
    return instance;
}

std::string Config::getBLEDeviceName(const std::string defaultValue) {
    if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
        Serial.println("Unable to mount SPIFFS");
        return defaultValue;
    }
    File file = SPIFFS.open(BLE_DEVICE_NAME_FILE);
    if (!file) return defaultValue;

    std::string value;
    while (file.available()) {
        value.append(1, file.read());
    }
    file.close();
    return value.length() > 0 ? value : defaultValue;
}

void Config::setBLEDeviceName(const std::string value) {
    File file = SPIFFS.open(BLE_DEVICE_NAME_FILE, FILE_WRITE);
    if (file) {
        file.print(value.c_str());
        file.close();
        Serial.print("Set BLE device name: ");
        Serial.println(value.c_str());
    } else {
        Serial.println("Unable to open BLE device name file");
    }
}
