// BSD 3-Clause License
//
// Copyright (c) 2023, Ekumen Inc.
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
#pragma once

#include <chrono>
#include <string>

#include <libserial/SerialPort.h>

#include "andino_base/mcu.h"

namespace andino_base {

/// \brief This class provides a serial implementation of the MCU interface.
class SerialMcu : public Mcu {
 public:
  /// @brief Default constructor.
  SerialMcu() = default;

  /// @brief Configures the serial communication.
  /// @param[in] serial_device Path to the serial device(eg. /dev/ttyACM0)
  /// @param[in] baud_rate Baud rate of the serial connection(eg. 57600)
  /// @param[in] timeout_ms Timeout in milliseconds.
  void setup(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms);

  /// @brief Implements Mcu interface class API.
  bool is_connected() const override;

  /// @brief Implements Mcu interface class API.
  void reset_encoders() override;

  /// @brief Implements Mcu interface class API.
  EncodersData read_encoders() override;

  /// @brief Implements Mcu interface class API.
  bool is_imu_available() override;

  /// @brief Implements Mcu interface class API.
  EncodersAndImuData read_encoders_and_imu() override;

  /// @brief The IMU's calibration status: {system, gyroscope, accelerometer, magnetometer}, 0-3 each.
  /// Not part of the Mcu interface: a diagnostic of THIS link's firmware (command `c`). Firmware
  /// without the command answers something else; that is reported as {-1, -1, -1, -1}, never parsed.
  std::array<int, 4> read_imu_calibration();

  /// @brief Implements Mcu interface class API.
  void set_motors_speed(int left_motor_speed, int right_motor_speed) override;

  /// @brief Implements Mcu interface class API.
  void set_motors_pwm(int left_motor_pwm, int right_motor_pwm) override;

  /// @brief Implements Mcu interface class API.
  void set_pid_tuning_gains(float kp, float kd, float ki, float ko) override;

  /// @brief Gets back in step with the microcontroller after a reply was missed.
  /// The protocol is strictly one reply per command with nothing to tell replies apart, so a reply
  /// that arrives after its command timed out is read as the answer to the NEXT command — and to
  /// every command after it: the exchange stays one reply behind forever. Waits for stragglers to
  /// land, then drops them.
  void resync();

 private:
  /// @brief Sends a message to the microcontroller and reads the response.
  /// @param msg Message to send to the microcontroller.
  /// @returns The response from the microcontroller.
  /// @param[in] log_timeout Whether to report a timed out response on stderr.
  std::string send_message(const std::string& msg, bool log_timeout = true);

  /// @brief Blocks until the microcontroller answers, or the deadline passes.
  /// @returns True if it answered.
  bool wait_until_ready(std::chrono::milliseconds deadline);

  // Underlying serial connection.
  LibSerial::SerialPort serial_port_;

  int32_t timeout_ms_{25};
};

}  // namespace andino_base
