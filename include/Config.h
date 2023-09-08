#pragma once
#ifndef _CONFIG_H
#define _CONFIG_H

#include <string>

class Config {
 public:
  static Config& getInstance();
  Config(Config const&) = delete;

  void operator=(Config const&) = delete;
  std::string getBLEDeviceName(const std::string defaultValue);
  void setBLEDeviceName(const std::string value);

 private:
  Config() {}

  static constexpr const char* BLE_DEVICE_NAME_FILE = "/ble-name.txt";
};

#endif  // _CONFIG_H
