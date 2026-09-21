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
#include "andino_base/serial_mcu.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace andino_base {
namespace {

// Maps a baud rate in bits per second to LibSerial's enumeration.
// @throws std::invalid_argument for a rate the firmware cannot be built with.
LibSerial::BaudRate to_libserial_baud_rate(int32_t baud_rate) {
  switch (baud_rate) {
    case 9600:
      return LibSerial::BaudRate::BAUD_9600;
    case 19200:
      return LibSerial::BaudRate::BAUD_19200;
    case 38400:
      return LibSerial::BaudRate::BAUD_38400;
    case 57600:
      return LibSerial::BaudRate::BAUD_57600;
    case 115200:
      return LibSerial::BaudRate::BAUD_115200;
    case 230400:
      return LibSerial::BaudRate::BAUD_230400;
    default:
      throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud_rate));
  }
}

}  // namespace

void SerialMcu::setup(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms) {
  timeout_ms_ = timeout_ms;
  try {
    serial_port_.Open(serial_device);
  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  }

  std::cout << "Waiting 2 seconds for the Microcontroller to be ready..." << std::endl;
  // When the Microcontroller is reset, it takes a few seconds to be ready.
  // And tipically when the serial port is opened, the Microcontroller is reset.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Configure the serial port. It must match Constants::kBaudrate in andino_firmware.
  serial_port_.SetBaudRate(to_libserial_baud_rate(baud_rate));
  serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
  serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  serial_port_.SetDTR(false);
  serial_port_.SetRTS(false);
  // Flush buffers.
  serial_port_.FlushIOBuffers();

  // The wait above is a guess, not a guarantee, so ask until the board answers. Measured on an
  // Arduino Nano (Optiboot) with a BNO055: polled from the moment the port opens, it first answers
  // after ~1.2 s — yet with this function's exact sequence a single command sent at 2.0 s came
  // back 140 ms late twice and never once (cause not established). Either way, one blind command
  // after a fixed sleep is not a handshake. `e` is read-only and always answered.
  if (!wait_until_ready(std::chrono::seconds(5))) {
    std::cerr << "The Microcontroller did not answer within 5 seconds of opening " << serial_device << "."
              << std::endl;
  }
}

void SerialMcu::resync() {
  // Longer than the slowest reply (encoders + IMU: ~14 ms at 115200 baud), shorter than anyone notices.
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  serial_port_.FlushIOBuffers();
}

bool SerialMcu::wait_until_ready(std::chrono::milliseconds deadline) {
  const auto end = std::chrono::steady_clock::now() + deadline;
  while (std::chrono::steady_clock::now() < end) {
    if (!send_message("e", /* log_timeout */ false).empty()) {
      // The UART is up before the firmware's shell is: the polls sent during those last few
      // hundred milliseconds were queued on the board, and are now all answered at once. Let that
      // burst land and drop it, or the NEXT command reads one of those stale replies as its own
      // (seen on hardware: `h` -> "0 0", parsed as "no IMU").
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      serial_port_.FlushIOBuffers();
      return true;
    }
  }
  return false;
}

bool SerialMcu::is_connected() const { return serial_port_.IsOpen(); }

void SerialMcu::reset_encoders() { send_message("r"); }

SerialMcu::EncodersData SerialMcu::read_encoders() {
  static const std::string delimiter = " ";

  const std::string response = send_message("e");
  const size_t del_pos = response.find(delimiter);
  const std::string token_1 = response.substr(0, del_pos).c_str();
  const std::string token_2 = response.substr(del_pos + delimiter.length()).c_str();
  return {std::atoi(token_1.c_str()), std::atoi(token_2.c_str())};
}

bool SerialMcu::is_imu_available() {
  // Decided once and lived with for the whole session, so one lost exchange must not decide it.
  // And strictly: the only valid answers are "0" and "1", so anything else is a reply to some
  // other command that was still in flight — try again rather than parse it.
  for (int attempt = 0; attempt < 5; ++attempt) {
    std::string response = send_message("h", /* log_timeout */ false);
    response.erase(std::remove_if(response.begin(), response.end(), [](unsigned char c) { return std::isspace(c); }),
                   response.end());
    if (response == "1") {
      return true;
    }
    if (response == "0") {
      return false;
    }
  }
  std::cerr << "No valid response to h after 5 attempts." << std::endl;
  return false;
}

std::array<int, 4> SerialMcu::read_imu_calibration() {
  const std::string response = send_message("c", /* log_timeout */ false);
  std::istringstream iss(response);
  std::array<int, 4> status{-1, -1, -1, -1};
  std::string rest;
  if (!(iss >> status[0] >> status[1] >> status[2] >> status[3]) || (iss >> rest) ||
      std::any_of(status.begin(), status.end(), [](int v) { return v < 0 || v > 3; })) {
    return {-1, -1, -1, -1};  // an older firmware's "invalid command", or somebody else's reply
  }
  return status;
}

SerialMcu::EncodersAndImuData SerialMcu::read_encoders_and_imu() {
  static const std::string delimiter = " ";

  const std::string response = send_message("i");

  std::istringstream iss(response);
  EncodersAndImuData encoders_and_imu_data;
  iss >> encoders_and_imu_data.encoders_data[0] >> encoders_and_imu_data.encoders_data[1];
  iss >> encoders_and_imu_data.imu_data.orientation[0] >> encoders_and_imu_data.imu_data.orientation[1] >>
      encoders_and_imu_data.imu_data.orientation[2] >> encoders_and_imu_data.imu_data.orientation[3];
  iss >> encoders_and_imu_data.imu_data.angular_velocity[0] >> encoders_and_imu_data.imu_data.angular_velocity[1] >>
      encoders_and_imu_data.imu_data.angular_velocity[2];
  iss >> encoders_and_imu_data.imu_data.linear_acceleration[0] >>
      encoders_and_imu_data.imu_data.linear_acceleration[1] >> encoders_and_imu_data.imu_data.linear_acceleration[2];
  return encoders_and_imu_data;
}

void SerialMcu::set_motors_speed(int left_motor_speed, int right_motor_speed) {
  std::stringstream ss;
  ss << "m " << left_motor_speed << " " << right_motor_speed;
  send_message(ss.str());
}

void SerialMcu::set_motors_pwm(int left_motor_pwm, int right_motor_pwm) {
  std::stringstream ss;
  ss << "o " << left_motor_pwm << " " << right_motor_pwm;
  send_message(ss.str());
}

void SerialMcu::set_pid_tuning_gains(float kp, float kd, float ki, float ko) {
  std::stringstream ss;
  ss << "u " << kp << " " << kd << " " << ki << " " << ko;
  send_message(ss.str());
}

std::string SerialMcu::send_message(const std::string& msg, bool log_timeout) {
  // Add carriage return to the message.
  const std::string msg_to_send = msg + '\r';
  // Send the message.
  serial_port_.Write(msg_to_send);

  // Get response from the microcontroller.
  std::string response;
  try {
    serial_port_.ReadLine(response, '\n', timeout_ms_);
  } catch (LibSerial::ReadTimeout&) {
    if (log_timeout) {
      std::cerr << "Response to " << msg << " timed out." << std::endl;
    }
    // Whatever that reply was, it must not be read as the answer to the next command.
    resync();
  }
  return response;
}

}  // namespace andino_base
