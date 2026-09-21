// BSD 3-Clause License
//
// Copyright (c) 2026, Ekumen Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "andino/bsp/imu_arduino.h"

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#endif
#include <Adafruit_BNO055.h>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <Wire.h>

/// Adafruit BNO055 IMU sensor instance.
static Adafruit_BNO055 g_bno055_imu(55, BNO055_ADDRESS_A, &Wire);

namespace andino {

bool ImuArduino::begin() const {
  // IMUPLUS: the sensor fuses its gyroscope and accelerometer only. The library's default, NDOF,
  // also steers heading by the magnetometer — on an indoor robot that means a heading referenced
  // to whatever steel and motors are nearby, and one that is not trustworthy until the sensor has
  // been calibrated by waving it in a figure of eight, which a floor robot never does (and the
  // BNO055 forgets at every power-off). Here heading is RELATIVE to power-on and smooth; an
  // absolute reference, where one is wanted, belongs to the localisation layer.
  //
  // The other side of that argument is a build flag away, -DANDINO_IMU_NDOF: the magnetometer is the
  // only ABSOLUTE heading reference on the robot, and gyro-only heading measurably drifts with
  // turning (27 degrees over a 106 m, 6000-degree drive, 2026-09-21). Whether the indoor field is
  // usable is an experiment, not an opinion — `c` (calibration status) says which regime a run
  // was in. The firmware version string carries the mode.
#ifdef ANDINO_IMU_NDOF
  constexpr auto kMode = OPERATION_MODE_NDOF;
#else
  constexpr auto kMode = OPERATION_MODE_IMUPLUS;
#endif
  if (!g_bno055_imu.begin(kMode)) {
    return false;
  }
  g_bno055_imu.setExtCrystalUse(true);
  return true;
}

Imu::Orientation ImuArduino::get_orientation() const {
  // See https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview for
  // further information.
  imu::Quaternion orientation = g_bno055_imu.getQuat();
  return Orientation{orientation.x(), orientation.y(), orientation.z(), orientation.w()};
}

Imu::Vector3 ImuArduino::get_angular_velocity() const {
  // See https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview for
  // further information.
  // Adafruit_BNO055 leaves the sensor in its default unit selection and scales the gyroscope to
  // degrees per second; the HAL contract is rad/s.
  static constexpr double kDegToRad{0.017453292519943295};
  imu::Vector<3> angular_velocity = g_bno055_imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  return Vector3{angular_velocity.x() * kDegToRad, angular_velocity.y() * kDegToRad,
                 angular_velocity.z() * kDegToRad};
}

Imu::Vector3 ImuArduino::get_linear_acceleration() const {
  // See https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview for
  // further information.
  imu::Vector<3> linear_acceleration = g_bno055_imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  return Vector3{linear_acceleration.x(), linear_acceleration.y(), linear_acceleration.z()};
}

Imu::CalibrationStatus ImuArduino::get_calibration_status() const {
  uint8_t system = 0, gyroscope = 0, accelerometer = 0, magnetometer = 0;
  g_bno055_imu.getCalibration(&system, &gyroscope, &accelerometer, &magnetometer);
  CalibrationStatus status;
  status.system = system;
  status.gyroscope = gyroscope;
  status.accelerometer = accelerometer;
  status.magnetometer = magnetometer;
  return status;
}

}  // namespace andino
