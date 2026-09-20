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
#include "andino/app/pid.h"

#include <gtest/gtest.h>

namespace andino {
namespace test {
namespace {

TEST(PidTest, Initialize) {
  andino::Pid pid_controller(1, 0, 0, 1, -100, 100);
  int output = 0;

  // Controller is disabled by default, so output variable should remain unchanged.
  pid_controller.compute(5, output);
  EXPECT_EQ(output, 0);
}

TEST(PidTest, ComputeOutputProportionalGain) {
  andino::Pid pid_controller(3, 0, 0, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint(15);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 30);

  pid_controller.compute(10, output);
  EXPECT_EQ(output, 60);
}

TEST(PidTest, ComputeOutputProportionalAndDerivativeGain) {
  andino::Pid pid_controller(3, 2, 0, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint(15);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 20);

  pid_controller.compute(10, output);
  EXPECT_EQ(output, 50);
}

TEST(PidTest, ComputeOutputProportionalAndIntegralGain) {
  andino::Pid pid_controller(3, 0, 1, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint(15);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 30);

  pid_controller.compute(10, output);
  EXPECT_EQ(output, 70);
}

TEST(PidTest, ComputeOutputProportionalDerivativeAndIntegralGain) {
  andino::Pid pid_controller(3, 2, 1, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint(15);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 20);

  pid_controller.compute(10, output);
  EXPECT_EQ(output, 60);
}

TEST(PidTest, Reset) {
  andino::Pid pid_controller(3, 0, 0, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint(15);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 30);

  // Controller reset, obtained output should be the same as before.
  pid_controller.reset(0);
  pid_controller.set_setpoint(15);
  pid_controller.compute(5, output);
  EXPECT_EQ(output, 30);
}

TEST(PidTest, SetTunings) {
  andino::Pid pid_controller(3, 0, 0, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint(15);
  pid_controller.enable();

  pid_controller.compute(5, output);
  EXPECT_EQ(output, 30);

  // Proportional gain doubled, obtained output should be doubled.
  pid_controller.reset(0);
  pid_controller.set_setpoint(15);
  pid_controller.set_tunings(6, 0, 0, 1);
  pid_controller.compute(5, output);
  EXPECT_EQ(output, 60);
}

TEST(PidTest, SetpointRateIsExactOnAverage) {
  // 266 ticks/s at a nominal 30 Hz is 8.87 ticks per period. A whole-tick setpoint could only say 8.
  andino::Pid pid_controller(1, 0, 0, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint_rate(266);
  pid_controller.enable();

  long total = 0;
  for (int period = 0; period < 300; ++period) {
    pid_controller.compute(0, 33UL, output);
    EXPECT_GE(pid_controller.setpoint(), 8);
    EXPECT_LE(pid_controller.setpoint(), 9);
    total += pid_controller.setpoint();
  }
  // 300 periods of 33 ms are 9.9 s: 266 * 9.9 = 2633.4 ticks.
  EXPECT_EQ(total, 2633);
}

TEST(PidTest, SetpointRateIsSymmetricInReverse) {
  andino::Pid pid_controller(1, 0, 0, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint_rate(-146);
  pid_controller.enable();

  long total = 0;
  for (int period = 0; period < 300; ++period) {
    pid_controller.compute(0, 33UL, output);
    EXPECT_LE(pid_controller.setpoint(), -4);
    EXPECT_GE(pid_controller.setpoint(), -5);
    total += pid_controller.setpoint();
  }
  // -146 * 9.9 = -1445.4 ticks.
  EXPECT_EQ(total, -1445);
}

TEST(PidTest, SetpointRateFollowsTheTimeThatElapsed) {
  // A period that ran long is asked for proportionally more ticks, not for a nominal period's worth.
  andino::Pid pid_controller(1, 0, 0, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint_rate(300);
  pid_controller.enable();

  pid_controller.compute(0, 30UL, output);
  EXPECT_EQ(pid_controller.setpoint(), 9);
  pid_controller.compute(0, 60UL, output);
  EXPECT_EQ(pid_controller.setpoint(), 18);
}

TEST(PidTest, ResetDropsTheCarriedRemainder) {
  andino::Pid pid_controller(1, 0, 0, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint_rate(29);
  pid_controller.enable();

  // 29 ticks/s * 33 ms = 0.957 ticks: nothing yet, all of it carried.
  pid_controller.compute(0, 33UL, output);
  EXPECT_EQ(pid_controller.setpoint(), 0);
  pid_controller.reset(0);
  // Without the reset the carry would make this period's setpoint 1.
  pid_controller.compute(0, 33UL, output);
  EXPECT_EQ(pid_controller.setpoint(), 0);
}

TEST(PidTest, DisabledControllerAccruesNothing) {
  andino::Pid pid_controller(1, 0, 0, 1, -100, 100);
  int output = 0;
  pid_controller.set_setpoint_rate(3000);

  pid_controller.compute(0, 1000UL, output);
  EXPECT_EQ(pid_controller.setpoint(), 0);
  EXPECT_EQ(output, 0);
}

}  // namespace
}  // namespace test
}  // namespace andino

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (RUN_ALL_TESTS()) {
  }

  // Always return zero-code and allow PlatformIO to parse results.
  return 0;
}
