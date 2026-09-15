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

void simulate_shape(const std::string & kind, bool clockwise, double radius)
{
  MotionTest test;
  MotionTest::Config c;
  c.clockwise = clockwise;
  c.circle_radius = radius;
  c.repetitions = 2;
  c.circle_timeout = 240.0;
  MotionTest::Sample s;
  s.x = 2.0;
  s.y = -1.0;
  s.yaw = 3.05;
  test.start(kind, c, s);
  int segments = 0;
  MotionTest::Command previous;
  for (int i = 0; i < 12000 && test.active(); ++i) {
    s.time += 0.05;
    const int before = test.segment();
    const auto command = test.update(s);
    if (test.segment() != before) {
      ++segments;
      require(command.linear == 0.0 && command.angular == 0.0, "shape starts segment stopped");
    }
    require(std::abs(command.linear - previous.linear) <= c.linear_accel * 0.05 + 1e-9,
      "shape linear acceleration and deceleration");
    require(std::abs(command.angular - previous.angular) <= c.angular_accel * 0.05 + 1e-9,
      "shape angular acceleration and deceleration");
    require(command.linear >= 0.0 && command.linear <= c.linear_speed + 1e-9,
      "shape always drives forward within speed limit");
    require(std::abs(command.angular) <= c.angular_speed + 1e-9, "angular speed cap");
    require(clockwise ? command.angular <= 0.0 : command.angular >= 0.0, "turn direction");
    if (kind == "circle") {
      require(std::abs(command.linear - radius * std::abs(command.angular)) < 1e-9,
        "circle preserves radius during acceleration and braking");
      require(std::abs(test.path_error(s)) < 1e-8, "ideal circle radial error");
    } else {
      require(command.linear == 0.0 || command.angular == 0.0, "square has stationary corners");
      if (test.segment() > 0) {
        require(test.segment_type() == (test.segment() % 2 ? "line" : "turn"),
          "square alternates line and turn");
      }
    }
    const double next_yaw = s.yaw + command.angular * 0.05;
    if (std::abs(command.angular) > 1e-12) {
      s.x += command.linear / command.angular * (std::sin(next_yaw) - std::sin(s.yaw));
      s.y += command.linear / command.angular * (std::cos(s.yaw) - std::cos(next_yaw));
    } else {
      s.x += command.linear * std::cos(s.yaw) * 0.05;
      s.y += command.linear * std::sin(s.yaw) * 0.05;
    }
    s.yaw = next_yaw;
    s.left_rpm = (command.linear - command.angular * 0.4762 / 2.0) * 60.0 /
      (6.283185307179586 * 0.085);
    s.right_rpm = (command.linear + command.angular * 0.4762 / 2.0) * 60.0 /
      (6.283185307179586 * 0.085);
    require(std::max(std::abs(s.left_rpm), std::abs(s.right_rpm)) <=
      MotionTest::maximum_wheel_speed(kind, c, 0.4762) * 60.0 /
      (6.283185307179586 * 0.085) + 1e-9, "wheel limit accounts for outer wheel");
    previous = command;
  }
  require(test.state() == "completed", "shape must complete");
  require(segments == (kind == "circle" ? 2 : 16), "shape repetitions and segment count");
  const double signed_turn = (clockwise ? -1.0 : 1.0) * (s.yaw - 3.05);
  require(std::abs(signed_turn - 4.0 * 3.141592653589793) < 0.2, "two complete revolutions");
  require(std::hypot(s.x - 2.0, s.y + 1.0) < 0.15, "ideal shape closure");
  require(test.last_segment_type() == (kind == "circle" ? "arc" : "turn"),
    "last stopped segment units identifiable");
}

void shape_guards()
{
  MotionTest::Config c;
  require(MotionTest::supported("circle") && MotionTest::supported("square"), "supported v2");
  require(MotionTest::supported("s"), "S shape supported");
  for (const auto * kind : {"circle", "square"}) {
    MotionTest test;
    MotionTest::Sample s;
    test.start(kind, c, s);
    for (int i = 0; i < 150 && test.active(); ++i) {
      s.time += 0.05;
      test.update(s);
    }
    require(test.state() == "aborted", "stalled shape aborts");
    test.start(kind, c, s);
    s.time += 0.05;
    s.speed_age = 0.6;
    test.update(s);
    require(test.state() == "aborted", "shape feedback timeout");
  }
  require(std::abs(MotionTest::maximum_wheel_speed("circle", c, 0.4762) - 0.12381) < 1e-9,
    "default circle outer wheel speed is 0.12381 m/s");
  // Fresh, progressing feedback must not defeat the per-circle deadline.
  MotionTest timed;
  c.stall_timeout = 1.0;
  c.circle_timeout = 2.0;
  MotionTest::Sample sample;
  timed.start("circle", c, sample);
  for (int i = 0; i < 100 && timed.active(); ++i) {
    sample.time += 0.05;
    const auto command = timed.update(sample);
    sample.yaw += command.angular * 0.05;
  }
  require(timed.reason() == "segment or standstill timeout", "circle deadline enforced");

  for (int which = 0; which < 4; ++which) {
    auto bad = c;
    if (which == 0) {bad.circle_radius = 0.0;}
    if (which == 1) {bad.circle_radius = std::numeric_limits<double>::infinity();}
    if (which == 2) {bad.square_side = bad.distance_tolerance;}
    if (which == 3) {bad.circle_timeout = bad.stall_timeout;}
    bool rejected = false;
    try {MotionTest::validate(bad);} catch (const std::invalid_argument &) {rejected = true;}
    require(rejected, "invalid shape config rejected");
  }
}

double simulate_s(bool clockwise, bool correction, bool disturbance, int repetitions = 2)
{
  MotionTest test;
  MotionTest::Config c;
  c.clockwise = clockwise;
  c.repetitions = repetitions;
  if (!correction) {c.s_heading_gain = c.s_lateral_gain = 0.0;}
  MotionTest::Sample s;
  s.x = 1.0;
  s.y = -2.0;
  s.yaw = 3.1;
  test.start("s", c, s);
  MotionTest::Command previous;
  int started = 0, middle_crossings = 0;
  bool injected = false, first_lobe = false, second_lobe = false;
  double last_progress = 0.0;
  for (int i = 0; i < 6000 && test.active(); ++i) {
    s.time += 0.05;
    const int before = test.segment();
    const auto command = test.update(s);
    if (test.segment() != before) {
      ++started;
      last_progress = 0.0;
      require(command.linear == 0.0 && command.angular == 0.0, "S starts from standstill");
    }
    const double progress = test.progress(s);
    if (last_progress < c.s_length / 2.0 && progress >= c.s_length / 2.0) {
      ++middle_crossings;
      require(test.state() == "running" && command.linear > 0.05, "S midpoint never stops");
    }
    if (progress > 0.9 && progress < 1.1) {
      first_lobe = first_lobe || (clockwise ? command.angular < -0.05 : command.angular > 0.05);
    }
    if (progress > 2.9 && progress < 3.1) {
      second_lobe = second_lobe || (clockwise ? command.angular > 0.05 : command.angular < -0.05);
    }
    require(command.linear >= 0.0 && command.linear <= c.linear_speed + 1e-9, "S speed cap");
    require(std::abs(command.angular) <= c.angular_speed + 1e-9, "S angular speed cap");
    require(std::abs(command.linear - previous.linear) <= c.linear_accel * 0.05 + 1e-9,
      "S linear slew including final stop");
    require(std::abs(command.angular - previous.angular) <= c.angular_accel * 0.05 + 1e-9,
      "S angular slew through sign reversal and stop");
    const double next_yaw = s.yaw + command.angular * 0.05;
    if (std::abs(command.angular) > 1e-12) {
      s.x += command.linear / command.angular * (std::sin(next_yaw) - std::sin(s.yaw));
      s.y += command.linear / command.angular * (std::cos(s.yaw) - std::cos(next_yaw));
    } else {
      s.x += command.linear * std::cos(s.yaw) * 0.05;
      s.y += command.linear * std::sin(s.yaw) * 0.05;
    }
    s.yaw = next_yaw;
    // A heading disturbance represents asymmetric motor response/odometry yaw error.
    if (disturbance && !injected && progress > 0.8) {
      s.yaw += 0.2;
      injected = true;
    }
    s.left_rpm = (command.linear - command.angular * 0.4762 / 2.0) * 60.0 /
      (6.283185307179586 * 0.085);
    s.right_rpm = (command.linear + command.angular * 0.4762 / 2.0) * 60.0 /
      (6.283185307179586 * 0.085);
    require(std::max(std::abs(s.left_rpm), std::abs(s.right_rpm)) <=
      MotionTest::maximum_wheel_speed("s", c, 0.4762) * 60.0 /
      (6.283185307179586 * 0.085) + 1e-9, "S outer wheel bound includes correction");
    previous = command;
    last_progress = progress;
  }
  require(test.state() == "completed", "S finishes");
  require(started == repetitions && middle_crossings == repetitions, "S repetitions");
  require(first_lobe && second_lobe, "S bends in both directions");
  require(test.last_segment_type() == "s_curve", "S report type");
  require(std::abs(test.last_segment_progress() - c.s_length) < 0.03, "S distance termination");
  require(std::abs(test.reference_heading() - (s.yaw + test.heading_error(s))) < 1e-8,
    "continuous reference heading crosses pi correctly");
  return std::abs(test.path_error(s)) + std::abs(test.heading_error(s));
}

void s_guards()
{
  MotionTest::Config c;
  for (int failure = 0; failure < 4; ++failure) {
    MotionTest test;
    MotionTest::Sample s;
    test.start("s", c, s);
    for (int i = 0; i < 150 && test.active(); ++i) {
      s.time += 0.05;
      if (failure == 1) {s.x -= 0.01;}
      if (failure == 2) {s.yaw += 0.01;}
      if (failure == 3) {s.odom_age = 0.6;}
      test.update(s);
    }
    require(test.state() == "aborted", "S stalls, reverse, spin or stale data aborts");
  }
  for (int invalid = 0; invalid < 4; ++invalid) {
    auto bad = c;
    if (invalid == 0) {bad.s_radius = 0.0;}
    if (invalid == 1) {bad.s_length = bad.distance_tolerance;}
    if (invalid == 2) {bad.s_heading_gain = -1.0;}
    if (invalid == 3) {bad.s_lateral_gain = std::numeric_limits<double>::quiet_NaN();}
    bool rejected = false;
    try {MotionTest::validate(bad);} catch (const std::invalid_argument &) {rejected = true;}
    require(rejected, "S config validation");
  }
  require(std::abs(MotionTest::maximum_wheel_speed("s", c, 0.4762) - 0.14762) < 1e-9,
    "S maximum wheel speed with correction reserve");
}

int main()
{
  try {
    simulate("straight", 1.5707963267948966);
    simulate("rotate", 1.5707963267948966);
    simulate("rotate", 6.283185307179586);
    guards();
    for (bool clockwise : {false, true}) {
      simulate_shape("circle", clockwise, 0.2);
      simulate_shape("circle", clockwise, 1.0);
      simulate_shape("circle", clockwise, 2.0);
      simulate_shape("square", clockwise, 1.0);
    }
    shape_guards();
    for (bool clockwise : {false, true}) {
      require(simulate_s(clockwise, false, false) < 0.05, "nominal S feedforward error");
      require(simulate_s(clockwise, true, false) < 0.05, "nominal S corrected error");
      const double baseline = simulate_s(clockwise, false, true, 1);
      const double corrected = simulate_s(clockwise, true, true, 1);
      require(corrected < baseline * 0.5, "S correction reduces disturbance error");
    }
    s_guards();
    std::cout << "Motion test scenarios passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
