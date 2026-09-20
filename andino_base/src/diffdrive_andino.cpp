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
#include "andino_base/diffdrive_andino.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace andino_base {

hardware_interface::CallbackReturn DiffDriveAndino::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "On init...");

  config_.left_wheel_name = info_.hardware_parameters[kLeftWheelNameParam];
  RCLCPP_DEBUG(logger_, (kLeftWheelNameParam + static_cast<std::string>(": ") + config_.left_wheel_name).c_str());
  config_.right_wheel_name = info_.hardware_parameters[kRightWheelNameParam];
  RCLCPP_DEBUG(logger_, (kRightWheelNameParam + static_cast<std::string>(": ") + config_.right_wheel_name).c_str());
  config_.serial_device = info_.hardware_parameters[kSerialDeviceParam];
  RCLCPP_DEBUG(logger_, (kSerialDeviceParam + static_cast<std::string>(": ") + config_.serial_device).c_str());
  config_.baud_rate = std::stoi(info_.hardware_parameters[kBaudRateParam]);
  RCLCPP_DEBUG(logger_,
               (kBaudRateParam + static_cast<std::string>(": ") + info_.hardware_parameters[kBaudRateParam]).c_str());
  config_.timeout = std::stoi(info_.hardware_parameters[kTimeoutParam]);
  RCLCPP_DEBUG(logger_,
               (kTimeoutParam + static_cast<std::string>(": ") + info_.hardware_parameters[kTimeoutParam]).c_str());
  config_.enc_ticks_per_rev = std::stoi(info_.hardware_parameters[kEncTicksPerRevParam]);
  RCLCPP_DEBUG(logger_,
               (kEncTicksPerRevParam + static_cast<std::string>(": ") + info_.hardware_parameters[kEncTicksPerRevParam])
                   .c_str());

  for (const hardware_interface::ComponentInfo& joint : info.joints) {
    // DiffDriveAndino has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(logger_, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(logger_, "Joint '%s' has %zu state interfaces found. 1 expected.", joint.name.c_str(),
                   joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // An IMU is optional: it exists for this driver only if the description declares a <sensor>.
  if (info.sensors.size() > 1) {
    RCLCPP_FATAL(logger_, "%zu sensors declared. At most 1 (the IMU) is supported.", info.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info.sensors.size() == 1) {
    const hardware_interface::ComponentInfo& sensor = info.sensors.front();
    for (const std::string& name : kImuInterfaceNames) {
      const bool declared =
          std::any_of(sensor.state_interfaces.begin(), sensor.state_interfaces.end(),
                      [&name](const hardware_interface::InterfaceInfo& interface) { return interface.name == name; });
      if (!declared) {
        RCLCPP_FATAL(logger_, "Sensor '%s' does not declare the state interface '%s'.", sensor.name.c_str(),
                     name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    imu_sensor_name_ = sensor.name;
  }

  // Set up the wheels
  left_wheel_.Setup(config_.left_wheel_name, config_.enc_ticks_per_rev);
  right_wheel_.Setup(config_.right_wheel_name, config_.enc_ticks_per_rev);

  RCLCPP_INFO(logger_, "Finished On init.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "On configure...");

  // Set up communication with the microcontroller.
  try {
    serial_mcu_.setup(config_.serial_device, config_.baud_rate, config_.timeout);
  } catch (const std::invalid_argument& e) {
    RCLCPP_FATAL(logger_, "%s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!imu_sensor_name_.empty()) {
    imu_available_ = serial_mcu_.is_imu_available();
    if (imu_available_) {
      RCLCPP_INFO(logger_, "IMU detected: '%s' is live.", imu_sensor_name_.c_str());
    } else {
      RCLCPP_WARN(logger_, "Sensor '%s' is declared but the microcontroller reports no IMU: its state stays at rest.",
                  imu_sensor_name_.c_str());
    }
  }

  RCLCPP_INFO(logger_, "Finished Configuration");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveAndino::export_state_interfaces() {
  // We need to set up a position and a velocity interface for each wheel
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // TODO(francocipollone): Probably we could use the information obtained via info_ variable about the joint name
  // directly.
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(left_wheel_.name_, hardware_interface::HW_IF_POSITION, &left_wheel_.pos_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(right_wheel_.name_, hardware_interface::HW_IF_POSITION, &right_wheel_.pos_));

  if (!imu_sensor_name_.empty()) {
    for (size_t i = 0; i < kImuInterfaceNames.size(); ++i) {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(imu_sensor_name_, kImuInterfaceNames[i], &imu_state_[i]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveAndino::export_command_interfaces() {
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd_));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger_, "On activate...");
  RCLCPP_INFO(logger_, "Finished Activation");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveAndino::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_INFO(logger_, "On deactivate...");
  RCLCPP_INFO(logger_, "Finished Deactivation");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveAndino::read(const rclcpp::Time& /* time */, const rclcpp::Duration& period) {
  const double delta_secs = period.seconds();

  if (!serial_mcu_.is_connected()) {
    RCLCPP_ERROR(logger_, "MCU is not connected.");
    return hardware_interface::return_type::ERROR;
  }

  if (imu_available_) {
    // One exchange for both: the serial link is the bottleneck of this loop.
    const SerialMcu::EncodersAndImuData data = serial_mcu_.read_encoders_and_imu();
    const auto& q = data.imu_data.orientation;
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    // A unit quaternion doubles as an integrity check of the whole line: a truncated or timed-out
    // reply parses as zeros. Keep the whole previous state rather than report a robot that
    // teleported its wheels back to zero; the next good reading carries the missed ticks.
    if (norm < 0.9 || norm > 1.1) {
      RCLCPP_WARN_THROTTLE(logger_, throttle_clock_, 5000, "Discarding a malformed reply from the microcontroller.");
      return hardware_interface::return_type::OK;
    }
    left_wheel_.enc_ = data.encoders_data[0];
    right_wheel_.enc_ = data.encoders_data[1];
    std::copy(q.begin(), q.end(), imu_state_.begin());
    std::copy(data.imu_data.angular_velocity.begin(), data.imu_data.angular_velocity.end(), imu_state_.begin() + 4);
    std::copy(data.imu_data.linear_acceleration.begin(), data.imu_data.linear_acceleration.end(),
              imu_state_.begin() + 7);
  } else {
    const SerialMcu::EncodersData encoders = serial_mcu_.read_encoders();
    left_wheel_.enc_ = encoders[0];
    right_wheel_.enc_ = encoders[1];
  }

  const double left_pos_prev = left_wheel_.pos_;
  left_wheel_.pos_ = left_wheel_.Angle();
  left_wheel_.vel_ = (left_wheel_.pos_ - left_pos_prev) / delta_secs;

  const double right_pos_prev = right_wheel_.pos_;
  right_wheel_.pos_ = right_wheel_.Angle();
  right_wheel_.vel_ = (right_wheel_.pos_ - right_pos_prev) / delta_secs;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveAndino::write(const rclcpp::Time& /* time */,
                                                       const rclcpp::Duration& /* period */) {
  if (!serial_mcu_.is_connected()) {
    RCLCPP_ERROR(logger_, "MCU is not connected.");
    return hardware_interface::return_type::ERROR;
  }

  // The command is in rad/sec (rps), we need to convert it to ticks/sec (tps)
  // Using the rads per tick(rpt) of the motor information
  // Formula: ticks/sec = rads/sec / rads/tick

  const int left_value_target = static_cast<int>(left_wheel_.cmd_ / left_wheel_.rads_per_tick_);
  const int right_value_target = static_cast<int>(right_wheel_.cmd_ / right_wheel_.rads_per_tick_);
  serial_mcu_.set_motors_speed(left_value_target, right_value_target);

  return hardware_interface::return_type::OK;
}

}  // namespace andino_base

PLUGINLIB_EXPORT_CLASS(andino_base::DiffDriveAndino, hardware_interface::SystemInterface)
