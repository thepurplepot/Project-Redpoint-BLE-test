#pragma once
#ifndef _BLEIMUSERVICE_H
#define _BLEIMUSERVICE_H

#include <Adafruit_BNO055.h>
#include <stdint.h>

#include <vector>

#include "BLEServiceHandler.h"

const char BLE_IMU_SERVICE_UUID[] = "509B0001-EBE1-4AA5-BC51-11004B78D5CB";
const char BLE_IMU_SENSOR_CHAR_UUID[] = "509B0002-EBE1-4AA5-BC51-11004B78D5CB";
const char BLE_IMU_CALIBRATION_CHAR_UUID[] =
    "509B0003-EBE1-4AA5-BC51-11004B78D5CB";

const uint8_t BLE_IMU_MESSAGE_VERSION = 1;

enum BLE_IMUFieldBits {
  BLE_IMU_ACCEL_FLAG = 0x01,
  BLE_IMU_MAG_FLAG = 0x02,
  BLE_IMU_GYRO_FLAG = 0x04,
  BLE_IMU_CALIBRATION_FLAG = 0x08,
  BLE_IMU_QUATERNION_FLAG = 0x10,
  BLE_IMU_LINEAR_ACCEL_FLAG = 0x20,
  BLE_IMU_POSITION_FLAG = 0x40,
  BLE_IMU_GRAVITY_FLAG = 0x80
};

class BLE_IMUMessage {
 public:
  BLE_IMUMessage(unsigned long timestamp);

  void setAccelerometer(const imu::Vector<3> &vec);
  void setGyroscope(imu::Vector<3> vec);
  void setMagnetometer(const imu::Vector<3> &vec);
  void setLinearAcceleration(const imu::Vector<3> &vec);
  void setPosition(const float pos[3]); //Not Reliable?
  void setPosition(const imu::Vector<3> &vec);
  void setVelocity(const float vel[3]);
  void setVelocity(const imu::Vector<3> &vec);
  void setQuaternion(const float quat[4]);
  void setQuaternion(const double quat[4]);
  void setQuaternion(double w, double x, double y, double z);
  std::vector<uint8_t> getPayload();

 private:
  uint8_t flags_ = 0;
  unsigned long timestamp_;
  float quat_[4];
  float pos_[3], vel_[3] = {0, 0, 0};
  imu::Vector<3> accel_, gyro_, mag_, linear_;

  uint8_t *appendVector_(uint8_t *buf, size_t size, uint8_t *p,
                         const imu::Vector<3> &vec);
};

// 60 fps, with headroom
#define BLE_IMU_TX_FREQ 60
static const int BLE_IMU_TX_DELAY = (1000 - 10) / BLE_IMU_TX_FREQ;
static const double BLE_IMU_VEL_TRANSITION = (double)BLE_IMU_TX_DELAY / 1000.0;
static const double BLE_IMU_POS_TRANSITION = (double)0.5 * BLE_IMU_VEL_TRANSITION * BLE_IMU_VEL_TRANSITION;

class BLEIMUServiceHandler : public BLEServiceHandler {
 public:
  const bool INCLUDE_ALL_VALUES = true;

  BLEIMUServiceHandler(BLEServiceManager &manager, Adafruit_BNO055 &sensor);

  void start();
  void tick();

 private:
  BLECharacteristic *imuSensorValueChar_;
  BLECharacteristic *imuCalibrationChar_;
  Adafruit_BNO055 &bno_;
  std::array<uint8_t, 4> calibration_ = {{0, 0, 0, 0}};
  unsigned long nextTxTimeMs_ = 0;

  void setCalibrationValue();
};

#endif /* _BLEIMUSERVICE_H */
