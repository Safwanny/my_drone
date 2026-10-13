// Convergence checks for estimator.hpp on synthetic, noise-free sensors.
#include <gtest/gtest.h>

#include "my_drone_control/estimator.hpp"

using fc::Quat;
using fc::Vec3;

static const Vec3 kMagRef(0.2162, 0.0095, 0.4295);  // gz at yaw 0, see state_estimator

static double angle_deg(const Quat & a, const Quat & b)
{
  return a.angularDistance(b) * 180.0 / M_PI;
}

TEST(Ekf, InitFindsAttitudeFromAccelAndMag)
{
  const Quat q = Quat(Eigen::AngleAxisd(0.5, Vec3::UnitZ())) *
    Quat(Eigen::AngleAxisd(0.08, Vec3::UnitY()));
  fc::Ekf ekf;
  ekf.init(Vec3::Zero(), q.conjugate() * Vec3(0, 0, fc::kG), q.conjugate() * kMagRef, kMagRef);
  EXPECT_LT(angle_deg(ekf.q, q), 1e-6);
}

// Fly a wobbly circle with biased IMU, slowly yawing (with attitude held
// constant, horizontal accel bias and tilt look the same to every sensor),
// start the filter 5 deg off in yaw and 3 deg in roll, and check it pulls
// everything in from GNSS, baro and mag.
TEST(Ekf, ConvergesInFlight)
{
  const double yaw_rate = 0.1;  // rad/s
  Quat q_true(Eigen::AngleAxisd(0.5, Vec3::UnitZ()));
  const Vec3 ab_true(0.05, -0.03, 0.08), wb_true(0.002, -0.001, 0.003);
  const double dt = 1.0 / 250.0;

  fc::Ekf ekf;
  ekf.init(Vec3::Zero(), q_true.conjugate() * Vec3(0, 0, fc::kG),
    q_true.conjugate() * kMagRef, kMagRef);
  ekf.q = q_true * Quat(Eigen::AngleAxisd(0.087, Vec3::UnitZ())) *
    Quat(Eigen::AngleAxisd(0.052, Vec3::UnitX()));

  Vec3 p = Vec3::Zero(), v(0.5 / 0.5, 0, 0);  // v(0) matches a(t) below
  ekf.v = v;
  for (int k = 0; k < 60 * 250; ++k) {
    const double t = k * dt;
    const Vec3 a(0.5 * std::cos(0.5 * t), -0.5 * std::sin(0.5 * t), 0.2 * std::sin(t));
    const Vec3 f = q_true.conjugate() * (a + fc::kG * Vec3::UnitZ()) + ab_true;
    ekf.predict(f, Vec3(0, 0, yaw_rate) + wb_true, dt);
    q_true = q_true * fc::rotvec(Vec3(0, 0, yaw_rate * dt));
    p += v * dt + 0.5 * a * dt * dt;
    v += a * dt;
    if (k % 25 == 0) ekf.fuse_position(p, 0.1, 0.3);
    if (k % 5 == 0) ekf.fuse_height(p.z(), 0.3);
    if (k % 3 == 0) ekf.fuse_mag(q_true.conjugate() * kMagRef, kMagRef, 0.05);
  }
  EXPECT_LT((ekf.p - p).norm(), 0.05);
  EXPECT_LT((ekf.v - v).norm(), 0.05);
  EXPECT_LT(angle_deg(ekf.q, q_true), 0.2);
  EXPECT_LT((ekf.ab - ab_true).norm(), 0.02);
  EXPECT_LT((ekf.wb - wb_true).norm(), 5e-4);
}

TEST(Gnss, EnuOffsetsAreMetres)
{
  const double lat0 = 47.397971, lon0 = 8.546164;
  const Vec3 e = fc::gnss_to_enu(lat0 + 1e-5, lon0 + 1e-5, 2.0, lat0, lon0, 0.0);
  EXPECT_NEAR(e.y(), 1.1116, 0.001);  // 1e-5 deg of latitude at 47.4 deg
  EXPECT_NEAR(e.x(), 0.7543, 0.001);  // 1e-5 deg of longitude
  EXPECT_DOUBLE_EQ(e.z(), 2.0);
}

TEST(Baro, TwelvePascalsIsAboutAMetre)
{
  EXPECT_NEAR(fc::baro_height(101325.0 - 12.0, 101325.0), 1.0, 0.02);
}
