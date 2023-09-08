#include "BLEIMUService.h"

#include <BLE2902.h>

#include <cassert>

BLE_IMUMessage::BLE_IMUMessage(unsigned long timestamp) : timestamp_(timestamp){};

void BLE_IMUMessage::setAccelerometer(const imu::Vector<3> &vec) {
    accel_ = vec;
    flags_ |= BLE_IMU_ACCEL_FLAG;
}

void BLE_IMUMessage::setGyroscope(imu::Vector<3> vec) {
    gyro_ = vec;
    flags_ |= BLE_IMU_GYRO_FLAG;
}

void BLE_IMUMessage::setMagnetometer(const imu::Vector<3> &vec) {
    mag_ = vec;
    flags_ |= BLE_IMU_MAG_FLAG;
}

void BLE_IMUMessage::setLinearAcceleration(const imu::Vector<3> &vec) {
    linear_ = vec;
    flags_ |= BLE_IMU_LINEAR_ACCEL_FLAG;
}

void BLE_IMUMessage::setPosition(const float pos[3]) {
    memcpy(pos_, pos, sizeof pos_);
    flags_ |= BLE_IMU_POSITION_FLAG;
}

void BLE_IMUMessage::setPosition(const imu::Vector<3> &vec) {
    float p[3]; //x,y,z
    p[0] = pos_[0] + BLE_IMU_POS_TRANSITION * static_cast<float>(vec[0]);
    p[1] = pos_[1] + BLE_IMU_POS_TRANSITION * static_cast<float>(vec[1]);
    p[2] = pos_[2] + BLE_IMU_POS_TRANSITION * static_cast<float>(vec[2]);
    setPosition(p);
}

void BLE_IMUMessage::setVelocity(const float vel[3]) {
    memcpy(vel_, vel, sizeof vel_);
    flags_ |= BLE_IMU_POSITION_FLAG;
}

void BLE_IMUMessage::setVelocity(const imu::Vector<3> &vec) {
    float v[3]; //x,y,z
    v[0] = vel_[0] + BLE_IMU_VEL_TRANSITION * static_cast<float>(vec[0]);
    v[1] = vel_[1] + BLE_IMU_VEL_TRANSITION * static_cast<float>(vec[1]);
    v[2] = vel_[2] + BLE_IMU_VEL_TRANSITION * static_cast<float>(vec[2]);
    setVelocity(v);
}

void BLE_IMUMessage::setQuaternion(const float quat[4]) {
    memcpy(quat_, quat, sizeof quat_);
    flags_ |= BLE_IMU_QUATERNION_FLAG;
}

void BLE_IMUMessage::setQuaternion(const double quat[4]) {
    float q[4] = {
        static_cast<float>(quat[0]),
        static_cast<float>(quat[1]),
        static_cast<float>(quat[2]),
        static_cast<float>(quat[3]),
    };
    setQuaternion(q);
}

void BLE_IMUMessage::setQuaternion(double w, double x, double y, double z) {
    double q[] = {w, x, y, z};
    setQuaternion(q);
}

std::vector<uint8_t> BLE_IMUMessage::getPayload() {
    uint8_t buf[240] = {
        BLE_IMU_MESSAGE_VERSION,
        flags_,
        static_cast<uint8_t>(timestamp_),
        static_cast<uint8_t>(timestamp_ >> 8),
    };
    uint8_t *p = &buf[4];
    if (flags_ & BLE_IMU_QUATERNION_FLAG) {
        assert(p - buf + sizeof quat_ <= sizeof buf);
        memcpy(p, quat_, sizeof quat_);
        p += sizeof quat_;
    }
    if (flags_ & BLE_IMU_POSITION_FLAG) {
        // assert(p - buf + sizeof pos_ <= sizeof buf);
        // Serial.printf("Pos: %f,%f,%f\n", pos_[0], pos_[1],pos_[2]); //DEBUG
        // memcpy(p, pos_, sizeof pos_);
        // p += sizeof pos_;
        assert(p - buf + sizeof vel_ <= sizeof buf);
        Serial.printf("Vel Mag: %f\n", sqrt(vel_[0]*vel_[0] + vel_[1]*vel_[1] + vel_[2]*vel_[2])); //DEBUG
        memcpy(p, vel_, sizeof vel_);
        p += sizeof vel_;
    }
    if (flags_ & BLE_IMU_ACCEL_FLAG) {
        p = this->appendVector_(buf, sizeof buf, p, accel_);
    }
    if (flags_ & BLE_IMU_GYRO_FLAG) {
        p = this->appendVector_(buf, sizeof buf, p, gyro_);
    }
    if (flags_ & BLE_IMU_MAG_FLAG) {
        p = this->appendVector_(buf, sizeof buf, p, mag_);
    }
    if (flags_ & BLE_IMU_LINEAR_ACCEL_FLAG) {
        p = this->appendVector_(buf, sizeof buf, p, linear_);
    }
    std::vector<uint8_t> vec(buf, p);
    return vec;
}

uint8_t *BLE_IMUMessage::appendVector_(uint8_t *buf, size_t size, uint8_t *p,
                        const imu::Vector<3> &vec) {
    float v[3] = {
        static_cast<float>(vec.x()),
        static_cast<float>(vec.y()),
        static_cast<float>(vec.z()),
    };
    assert(p - buf + sizeof v <= size);
    memcpy(p, v, sizeof v);
    p += sizeof v;
    return p;
}


BLEIMUServiceHandler::BLEIMUServiceHandler(BLEServiceManager &manager, Adafruit_BNO055 &sensor)
    : BLEServiceHandler(manager, BLE_IMU_SERVICE_UUID), bno_(sensor) {
    imuSensorValueChar_ = bleService_->createCharacteristic(
        BLE_IMU_SENSOR_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    imuSensorValueChar_->addDescriptor(new BLE2902());

    imuCalibrationChar_ = bleService_->createCharacteristic(
        BLE_IMU_CALIBRATION_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    imuCalibrationChar_->addDescriptor(new BLE2902());
}

void BLEIMUServiceHandler::start() {
    BLEServiceHandler::start();
    setCalibrationValue();
}

void BLEIMUServiceHandler::tick() {
    unsigned long now = millis();
    // Doesn't account for time wraparound
    if (now > nextTxTimeMs_) {
        std::array<uint8_t, 4> calibration;
        bno_.getCalibration(&calibration[0], &calibration[1], &calibration[2],
                            &calibration[3]);
        if (calibration != calibration_) {
            setCalibrationValue();
            imuCalibrationChar_->notify();
        }

        BLE_IMUMessage message(now);
        auto quat = bno_.getQuat();
        message.setQuaternion(quat.w(), quat.x(), quat.y(), quat.z());
        //message.setPosition(bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL));
        message.setVelocity(bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL));
        if (INCLUDE_ALL_VALUES) {
            message.setAccelerometer(
                bno_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER));
            message.setGyroscope(bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE));
            message.setMagnetometer(
                bno_.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER));
            message.setLinearAcceleration(
                bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL));
        }

        std::vector<uint8_t> payload = message.getPayload();
        imuSensorValueChar_->setValue(payload.data(), payload.size());
        imuSensorValueChar_->notify();

        nextTxTimeMs_ = now + BLE_IMU_TX_DELAY;
    }
}

void BLEIMUServiceHandler::setCalibrationValue() {
    std::array<uint8_t, 4> calibration;
    bno_.getCalibration(&calibration[0], &calibration[1], &calibration[2],
                        &calibration[3]);
    imuCalibrationChar_->setValue(calibration.data(), calibration.size());
    calibration_ = calibration;
}
