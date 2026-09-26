#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

#include "fr10_pose_controller/joint_otg.hpp"
#include "test_support.hpp"

namespace {
using fr10_motion::kDegToRad;

struct Metrics {
  bool ok{true};
  bool finished{false};
  int samples{0};
  double max_velocity{0.0};
  double max_acceleration{0.0};
  double max_jerk{0.0};
  double max_delta_q{0.0};
  double max_delta_qdot{0.0};
  double max_delta_qddot{0.0};
  std::array<double, 6> final_position{};
};

Metrics run(const std::array<double, 6> &initial,
            const std::array<double, 6> &target_position, double rate_hz) {
  const double dt = 1.0 / rate_hz;
  const auto limits = fr10_motion_test::limits();
  fr10_motion::JointOtg otg(dt, limits, 10.0);
  otg.reset(initial);
  fr10_motion::JointTarget target;
  target.position_rad = target_position;
  target.revision = 1;
  Metrics metrics;
  metrics.ok = otg.submitTarget(target);
  fr10_motion::RobotState no_feedback;
  std::array<double, 6> q = initial, qd{}, qdd{};
  auto previous_q = q;
  auto previous_qd = qd;
  auto previous_qdd = qdd;
  while (metrics.ok && !otg.isFinished() && metrics.samples < 20000) {
    metrics.ok = otg.step(no_feedback, q, qd, qdd);
    for (std::size_t axis = 0; axis < 6; ++axis) {
      metrics.max_velocity = std::max(metrics.max_velocity, std::abs(qd[axis]));
      metrics.max_acceleration =
          std::max(metrics.max_acceleration, std::abs(qdd[axis]));
      metrics.max_jerk = std::max(
          metrics.max_jerk, std::abs(qdd[axis] - previous_qdd[axis]) / dt);
      metrics.max_delta_q =
          std::max(metrics.max_delta_q, std::abs(q[axis] - previous_q[axis]));
      metrics.max_delta_qdot = std::max(
          metrics.max_delta_qdot, std::abs(qd[axis] - previous_qd[axis]));
      metrics.max_delta_qddot = std::max(
          metrics.max_delta_qddot, std::abs(qdd[axis] - previous_qdd[axis]));
    }
    previous_q = q;
    previous_qd = qd;
    previous_qdd = qdd;
    ++metrics.samples;
  }
  metrics.finished = otg.isFinished();
  metrics.final_position = q;
  return metrics;
}

struct RetargetMetrics : Metrics {
  bool moving_at_retarget{false};
  bool zero_reset{false};
  bool crossed_zero{false};
  double q_before{0.0};
  double q_after{0.0};
  double qdot_before{0.0};
  double qdot_after{0.0};
  double qddot_before{0.0};
  double qddot_after{0.0};
};

RetargetMetrics retarget(double a_deg, double b_deg, double c_deg,
                         bool use_c) {
  constexpr double dt = 0.008;
  const auto limits = fr10_motion_test::limits();
  fr10_motion::JointOtg otg(dt, limits, 10.0);
  std::array<double, 6> initial{};
  otg.reset(initial);
  fr10_motion::JointTarget target;
  target.position_rad[0] = a_deg * kDegToRad;
  target.revision = 1;
  RetargetMetrics metrics;
  metrics.ok = otg.submitTarget(target);
  fr10_motion::RobotState no_feedback;
  std::array<double, 6> q{}, qd{}, qdd{};
  std::array<double, 6> previous_q{}, previous_qd{}, previous_qdd{};
  bool sent_b = false;
  bool sent_c = false;
  double previous_sign_velocity = 0.0;

  while (metrics.ok && !otg.isFinished() && metrics.samples < 5000) {
    metrics.ok = otg.step(no_feedback, q, qd, qdd);
    if (!sent_b && metrics.samples >= 35 && std::abs(qd[0]) > 0.10) {
      metrics.moving_at_retarget = true;
      metrics.q_before = q[0];
      metrics.qdot_before = qd[0];
      metrics.qddot_before = qdd[0];
      target.position_rad[0] = b_deg * kDegToRad;
      target.revision = 2;
      metrics.ok = metrics.ok && otg.submitTarget(target);
      sent_b = true;
    } else if (sent_b && metrics.q_after == 0.0) {
      metrics.q_after = q[0];
      metrics.qdot_after = qd[0];
      metrics.qddot_after = qdd[0];
      metrics.zero_reset = std::abs(metrics.qdot_before) > 0.05 &&
                           std::abs(metrics.qdot_after) < 1e-6;
    }
    if (use_c && sent_b && !sent_c && metrics.samples >= 75) {
      target.position_rad[0] = c_deg * kDegToRad;
      target.revision = 3;
      metrics.ok = metrics.ok && otg.submitTarget(target);
      sent_c = true;
    }
    if (previous_sign_velocity > 0.0 && qd[0] < 0.0) {
      metrics.crossed_zero = true;
    }
    previous_sign_velocity = qd[0];
    for (std::size_t axis = 0; axis < 6; ++axis) {
      metrics.max_velocity = std::max(metrics.max_velocity, std::abs(qd[axis]));
      metrics.max_acceleration =
          std::max(metrics.max_acceleration, std::abs(qdd[axis]));
      metrics.max_jerk = std::max(
          metrics.max_jerk, std::abs(qdd[axis] - previous_qdd[axis]) / dt);
      metrics.max_delta_q =
          std::max(metrics.max_delta_q, std::abs(q[axis] - previous_q[axis]));
      metrics.max_delta_qdot = std::max(
          metrics.max_delta_qdot, std::abs(qd[axis] - previous_qd[axis]));
      metrics.max_delta_qddot = std::max(
          metrics.max_delta_qddot, std::abs(qdd[axis] - previous_qdd[axis]));
    }
    previous_q = q;
    previous_qd = qd;
    previous_qdd = qdd;
    ++metrics.samples;
  }
  metrics.finished = otg.isFinished();
  metrics.final_position = q;
  return metrics;
}

bool dynamicsValid(const Metrics &metrics, double rate_hz) {
  const auto limits = fr10_motion_test::limits();
  const double numerical_slack = 1e-7;
  return metrics.ok && metrics.finished &&
         metrics.max_velocity <= limits.max_joint_velocity_rad_s[0] + numerical_slack &&
         metrics.max_acceleration <=
             limits.max_joint_acceleration_rad_s2[0] + numerical_slack &&
         metrics.max_jerk <= limits.max_joint_jerk_rad_s3[0] + 1e-5 &&
         metrics.max_delta_qdot <=
             limits.max_joint_acceleration_rad_s2[0] / rate_hz + numerical_slack;
}

bool rateDecoupling(double rate_hz, int &submissions, int &updates) {
  const double dt = 1.0 / rate_hz;
  fr10_motion::JointOtg otg(dt, fr10_motion_test::limits(), 10.0);
  std::array<double, 6> initial{};
  otg.reset(initial);
  fr10_motion::RobotState feedback;
  std::array<double, 6> q{}, qd{}, qdd{}, previous_qdd{};
  submissions = 0;
  updates = 0;
  const int update_count = static_cast<int>(std::round(10.0 * rate_hz));
  double next_submission_s = 0.0;
  for (int step = 0; step < update_count; ++step) {
    const double t = step * dt;
    if (t + 1e-12 >= next_submission_s) {
      fr10_motion::JointTarget target;
      target.revision = static_cast<uint64_t>(++submissions);
      for (std::size_t axis = 0; axis < 6; ++axis) {
        target.position_rad[axis] =
            (10.0 + axis) * kDegToRad * std::sin((0.3 + 0.02 * axis) * t);
      }
      if (!otg.submitTarget(target)) {
        return false;
      }
      next_submission_s += 0.1;
    }
    if (!otg.step(feedback, q, qd, qdd)) {
      return false;
    }
    for (std::size_t axis = 0; axis < 6; ++axis) {
      if (std::abs(qd[axis]) > 0.300001 || std::abs(qdd[axis]) > 0.800001 ||
          std::abs(qdd[axis] - previous_qdd[axis]) / dt > 4.00001) {
        return false;
      }
    }
    previous_qdd = qdd;
    ++updates;
  }
  return submissions == 100 && updates == update_count;
}
} // namespace

int main() {
  fr10_motion_test::Check check;
  std::array<double, 6> zero{};
  for (const auto &[id, degrees] :
       std::array<std::pair<const char *, double>, 5>{{
           {"JOTG_001_STATIONARY", 0.0}, {"JOTG_002_15DEG", 15.0},
           {"JOTG_003_40DEG", 40.0}, {"JOTG_004_70DEG", 70.0},
           {"JOTG_005_100DEG", 100.0}}}) {
    auto target = zero;
    target[0] = degrees * kDegToRad;
    const auto metrics = run(zero, target, 125.0);
    const bool reached = fr10_motion_test::near(metrics.final_position[0],
                                                target[0], 1e-9);
    check.require(dynamicsValid(metrics, 125.0) && reached, id,
                  "target or v/a/j bounds failed");
  }

  auto plus40 = zero;
  plus40[0] = 40.0 * kDegToRad;
  auto minus40 = zero;
  minus40[0] = -40.0 * kDegToRad;
  const auto sign = run(plus40, minus40, 125.0);
  check.require(dynamicsValid(sign, 125.0) &&
                    fr10_motion_test::near(sign.final_position[0], minus40[0], 1e-9),
                "JOTG_006_SIGN_CHANGE", "negative target was not reached");

  const auto same = retarget(90.0, 120.0, 0.0, false);
  check.require(dynamicsValid(same, 125.0) && same.moving_at_retarget &&
                    !same.zero_reset &&
                    fr10_motion_test::near(same.final_position[0], 120.0 * kDegToRad,
                                           1e-9),
                "JOTG_007_SAME_DIRECTION_RETARGET",
                "dynamic state reset or final target missed");
  const auto shorter = retarget(90.0, 40.0, 0.0, false);
  check.require(dynamicsValid(shorter, 125.0) && !shorter.zero_reset &&
                    fr10_motion_test::near(shorter.final_position[0], 40.0 * kDegToRad,
                                           1e-9),
                "JOTG_008_SHORTER_RETARGET", "bounded deceleration failed");
  const auto reverse = retarget(90.0, -30.0, 0.0, false);
  check.require(dynamicsValid(reverse, 125.0) && reverse.crossed_zero &&
                    fr10_motion_test::near(reverse.final_position[0], -30.0 * kDegToRad,
                                           1e-9),
                "JOTG_009_REVERSE_RETARGET", "reverse convergence failed");
  const auto abc = retarget(90.0, -30.0, 15.0, true);
  check.require(dynamicsValid(abc, 125.0) &&
                    fr10_motion_test::near(abc.final_position[0], 15.0 * kDegToRad,
                                           1e-9),
                "JOTG_010_A_B_C", "latest target C did not win");

  std::array<double, 6> multi{{70, -40, 60, 30, -35, 65}};
  for (double &value : multi) {
    value *= kDegToRad;
  }
  const auto multi_metrics = run(zero, multi, 125.0);
  double multi_error = 0.0;
  for (std::size_t axis = 0; axis < 6; ++axis) {
    multi_error = std::max(
        multi_error, std::abs(multi_metrics.final_position[axis] - multi[axis]));
  }
  check.require(dynamicsValid(multi_metrics, 125.0) && multi_error < 1e-9,
                "JOTG_011_MULTIDOF", "six-axis synchronized target failed");

  // Multi-axis reverse retarget test
  {
    constexpr double dt = 0.008;
    fr10_motion::JointOtg otg(dt, fr10_motion_test::limits(), 10.0);
    otg.reset(zero);
    fr10_motion::JointTarget target_a;
    target_a.position_rad = multi;
    target_a.revision = 1;
    bool ok = otg.submitTarget(target_a);
    fr10_motion::RobotState no_fb;
    std::array<double, 6> q{}, qd{}, qdd{};
    for (int step = 0; step < 40 && ok; ++step) {
      ok = otg.step(no_fb, q, qd, qdd);
    }
    // Reverse all axes mid-motion
    fr10_motion::JointTarget target_b;
    for (std::size_t i = 0; i < 6; ++i) {
      target_b.position_rad[i] = -0.5 * multi[i];
    }
    target_b.revision = 2;
    ok = ok && otg.submitTarget(target_b);
    int step = 0;
    while (ok && !otg.isFinished() && step < 5000) {
      ok = otg.step(no_fb, q, qd, qdd);
      ++step;
    }
    double rev_err = 0.0;
    for (std::size_t i = 0; i < 6; ++i) {
      rev_err = std::max(rev_err, std::abs(q[i] - target_b.position_rad[i]));
    }
    check.require(ok && otg.isFinished() && rev_err < 1e-9,
                  "JOTG_012_MULTIAXIS_REVERSE", "6-axis reverse retarget failed");
  }

  for (const double rate : {125.0, 250.0, 500.0}) {
    int submissions = 0;
    int updates = 0;
    const bool ok = rateDecoupling(rate, submissions, updates);
    const std::string id = "JOTG_RATE_10HZ_TO_" + std::to_string((int)rate);
    check.require(ok, id, "10 Hz target stream destabilized low-rate loop");
    std::cout << "METRIC " << id << " submissions=" << submissions
              << " updates=" << updates << "\n";
  }

  std::cout << "METRIC JOINT_CONTINUITY max_delta_q=" << abc.max_delta_q
            << " max_delta_qdot=" << abc.max_delta_qdot
            << " max_delta_qddot=" << abc.max_delta_qddot
            << " max_jerk=" << abc.max_jerk << "\n";
  return check.result();
}
