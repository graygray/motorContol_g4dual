// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT
// Standalone deterministic tests: no ROS installation or motor hardware required.
#include "motor_control_g4dual/motion_test.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using motor_control_g4dual::MotionTest;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

void simulate(const std::string & kind, double angle)
{
  MotionTest test;
  MotionTest::Config config;
  config.angle = angle;
  MotionTest::Sample sample;
  // Nonzero origin and heading exercise relative distance and continuous yaw.
  sample.x = 2.0;
  sample.y = -3.0;
  sample.yaw = 3.0;
  test.start(kind, config, sample);
  MotionTest::Command previous;
  int segment_count = 0;
  for (int i = 0; i < 10000 && test.active(); ++i) {
    sample.time += 0.05;
    const int segment_before = test.segment();
    const auto state_before = test.state();
    const auto command = test.update(sample);
    if (test.segment() != segment_before) {
      ++segment_count;
      require(command.linear == 0.0 && command.angular == 0.0,
        "segment transition must start at standstill");
    }
    if (state_before == "running") {
      require(std::abs(command.linear - previous.linear) <= config.linear_accel * 0.05 + 1e-9,
        "linear acceleration limit");
      require(std::abs(command.angular - previous.angular) <= config.angular_accel * 0.05 + 1e-9,
        "angular acceleration limit");
    }
    require(std::abs(command.linear) <= config.linear_speed + 1e-9, "linear speed limit");
    require(std::abs(command.angular) <= config.angular_speed + 1e-9, "angular speed limit");
    sample.x += command.linear * std::cos(sample.yaw) * 0.05;
    sample.y += command.linear * std::sin(sample.yaw) * 0.05;
    sample.yaw += command.angular * 0.05;
    sample.left_rpm = (command.linear - command.angular * 0.4762 / 2.0) * 60.0 /
      (6.283185307179586 * 0.085);
    sample.right_rpm = (command.linear + command.angular * 0.4762 / 2.0) * 60.0 /
      (6.283185307179586 * 0.085);
    previous = command;
  }
  require(test.state() == "completed", "sequence must complete");
  require(segment_count == 6, "three pairs of segments");
  const double goal = kind == "rotate" ? angle : config.distance;
  const double tolerance = kind == "rotate" ? config.angle_tolerance : config.distance_tolerance;
  require(std::abs(test.last_segment_progress() - goal) < tolerance * 2.0,
    "stopped segment retains distance or full continuous turn");
  require(std::hypot(sample.x - 2.0, sample.y + 3.0) < 0.03, "return displacement");
  require(std::abs(sample.yaw - 3.0) < 0.03, "return heading");
  require(test.update(sample).linear == 0.0, "terminal state cannot command motion");
}

void guards()
{
  MotionTest test;
  MotionTest::Config config;
  MotionTest::Sample sample;
  test.start("straight", config, sample);
  // A moving wheel prevents initial settling even if the other wheel is stopped.
  sample.right_rpm = 1.0;
  for (int i = 0; i < 20; ++i) {
    sample.time += 0.05;
    require(test.update(sample).linear == 0.0, "initial standstill required");
  }
  require(test.segment() == 0, "must not advance while moving");
  test.abort("cancel");
  require(test.state() == "aborted" && test.reason() == "cancel", "cancel reason");
  require(test.update(sample).angular == 0.0, "cancel clears command");

  for (int stale = 0; stale < 2; ++stale) {
    sample = {};
    test.start("rotate", config, sample);
    sample.time = 0.05;
    if (stale == 0) {sample.odom_age = 0.6;} else {sample.speed_age = 0.6;}
    test.update(sample);
    require(test.state() == "aborted", "both feedback streams independently watched");
  }
  sample = {};
  test.start("straight", config, sample);
  sample.time = 0.6;
  test.update(sample);
  require(test.reason() == "control loop deadline missed", "timer gap aborts");

  sample = {};
  test.start("straight", config, sample);
  for (int i = 0; i < 150 && test.active(); ++i) {
    sample.time += 0.05;
    test.update(sample);
  }
  require(test.reason() == "no forward progress in commanded direction", "stall aborts");

  sample = {};
  test.start("straight", config, sample);
  sample.left_rpm = 2.0;
  for (int i = 0; i < 1300 && test.active(); ++i) {
    sample.time += 0.05;
    test.update(sample);
  }
  require(test.reason() == "segment or standstill timeout", "never stops timeout");

  // Opposite motion cannot satisfy a distance target or refresh directed progress.
  sample = {};
  test.start("straight", config, sample);
  for (int i = 0; i < 150 && test.active(); ++i) {
    sample.time += 0.05;
    sample.x -= 0.01;
    test.update(sample);
  }
  require(test.state() == "aborted", "wrong direction aborts");

  // Reaching the target does not start reverse while wheel feedback is moving.
  sample = {};
  test.start("straight", config, sample);
  while (test.segment() == 0) {
    sample.time += 0.05;
    test.update(sample);
  }
  sample.x = config.distance + 0.04;
  sample.left_rpm = 1.0;
  for (int i = 0; i < 20; ++i) {
    sample.time += 0.05;
    test.update(sample);
  }
  require(test.state() == "settling" && test.segment() == 1,
    "target reached is not standstill");
  sample.left_rpm = 0.0;
  for (int i = 0; i < 5; ++i) {
    sample.time += 0.05;
    test.update(sample);
  }
  sample.left_rpm = 1.0;
  sample.time += 0.05;
  test.update(sample);
  sample.left_rpm = 0.0;
  for (int i = 0; i < 5; ++i) {
    sample.time += 0.05;
    test.update(sample);
  }
  require(test.segment() == 1, "movement resets standstill duration");
  for (int i = 0; i < 10; ++i) {
    sample.time += 0.05;
    test.update(sample);
  }
  require(test.segment() == 2, "reverse starts after continuous standstill");
  require(std::abs(test.last_segment_progress() - 1.04) < 1e-9,
    "stopped result includes overshoot");
  test.abort("cancel during reverse");
  require(test.update(sample).linear == 0.0, "cancel running segment");

  for (int invalid = 0; invalid < 3; ++invalid) {
    auto bad = config;
    if (invalid == 0) {bad.linear_speed = std::numeric_limits<double>::quiet_NaN();}
    if (invalid == 1) {bad.repetitions = 0;}
    if (invalid == 2) {bad.distance = bad.distance_tolerance;}
    bool threw = false;
    try {MotionTest::validate(bad);} catch (const std::invalid_argument &) {threw = true;}
    require(threw, "invalid configuration rejected");
  }
}

int main()
{
  try {
    simulate("straight", 1.5707963267948966);
    simulate("rotate", 1.5707963267948966);
    simulate("rotate", 6.283185307179586);
    guards();
    std::cout << "Motion test scenarios passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
