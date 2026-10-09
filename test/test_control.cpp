// Sign and scale checks for control.hpp, on build03's numbers.
#include <gtest/gtest.h>

#include "../src/control.hpp"

using fc::Quat;
using fc::Vec3;

static fc::Airframe build03()
{
  // Rotor order and spin as quad_mesh_common.xacro / quad_gazebo.xacro.
  return {0.931, Vec3(0.00711, 0.00754, 0.01362).asDiagonal(), 3.7e-6, 0.016, 2300.0,
    {{0.143, -0.143, 1}, {-0.143, 0.143, 1}, {0.143, 0.143, -1}, {-0.143, -0.143, -1}}};
}

TEST(Mixer, HoverIsEqualSpeedsAt785)
{
  const auto af = build03();
  const fc::Mixer mix(af, 0.1);
  EXPECT_NEAR((mix.a * mix.a_inv - Eigen::Matrix4d::Identity()).norm(), 0.0, 1e-9);
  const auto w = mix.rotor_speeds(af.mass * fc::kG, Vec3::Zero());
  for (int i = 0; i < 4; ++i) EXPECT_NEAR(w[i], 785.0, 2.0);
}

TEST(Mixer, TorquesSpeedUpTheRightRotors)
{
  const auto af = build03();
  const fc::Mixer mix(af, 0.1);
  const double t = af.mass * fc::kG;
  // +roll (right side down) needs more thrust on the left: FL(2), RL(1).
  auto w = mix.rotor_speeds(t, Vec3(0.1, 0, 0));
  EXPECT_GT(w[2], w[0]);
  EXPECT_GT(w[1], w[3]);
  // +pitch (nose down in FLU) needs more thrust at the back: RL(1), RR(3).
  w = mix.rotor_speeds(t, Vec3(0, 0.1, 0));
  EXPECT_GT(w[1], w[2]);
  EXPECT_GT(w[3], w[0]);
  // +yaw (ccw) comes from the cw rotors' drag: speed up 2 and 3.
  w = mix.rotor_speeds(t, Vec3(0, 0, 0.01));
  EXPECT_GT(w[2], w[0]);
  EXPECT_GT(w[3], w[1]);
  // Far more yaw than fits: still yaws as hard as it can, not zero.
  w = mix.rotor_speeds(t, Vec3(0, 0, 1.0));
  EXPECT_NEAR(w[0], 230.0, 1.0);  // ccw pair down at idle
  EXPECT_GT(w[2], 1000.0);
}

TEST(Attitude, ErrorGivesRateTowardSetpoint)
{
  const fc::AttitudeControl att;
  const Quat q_sp(Eigen::AngleAxisd(0.2, Vec3::UnitX()));
  EXPECT_GT(att.rate_setpoint(q_sp, Quat::Identity()).x(), 0.0);
  const Quat yaw_sp(Eigen::AngleAxisd(-0.3, Vec3::UnitZ()));
  EXPECT_LT(att.rate_setpoint(yaw_sp, Quat::Identity()).z(), 0.0);
}

TEST(ThrustAttitude, ForwardAccelTiltsNoseDownWithinLimit)
{
  const double tilt_max = 35.0 * M_PI / 180.0;
  const auto ta = fc::thrust_attitude(
    Vec3(50.0, 0, 0), 0.0, Quat::Identity(), 1.0, tilt_max, 3.0 * fc::kG);
  const Vec3 b3 = ta.q_sp * Vec3::UnitZ();
  EXPECT_GT(b3.x(), 0.0);  // thrust leans forward
  EXPECT_NEAR(std::acos(b3.z()), tilt_max, 1e-6);
  EXPECT_NEAR(ta.collective, 1.0 * fc::kG, 1e-6);  // level now: vertical part only
}
