// State estimator: an error-state EKF in the shape of PX4's EKF2, free of
// ROS so it can be unit tested.
//
//   predict: IMU (specific force + body rates) at its own rate
//   correct: GNSS position, barometer height, magnetometer direction
//
// Nominal state: position p and velocity v (world ENU), attitude q (body
// FLU -> world), accelerometer bias ab and gyro bias wb (body). The filter
// tracks the 15-dim error [dp dv dth dab dwb] with dth a body-frame (local)
// rotation error, as in Sola, "Quaternion kinematics for the error-state
// Kalman filter" (2017), sec. 5 and 7.
#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "my_drone_control/control.hpp"

namespace fc {

inline Mat3 skew(const Vec3 & v)
{
  Mat3 m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

// Rotation by the rotation vector th (axis * angle).
inline Quat rotvec(const Vec3 & th)
{
  const double a = th.norm();
  return a < 1e-12 ? Quat::Identity() : Quat(Eigen::AngleAxisd(a, th / a));
}

class Ekf {
public:
  using Mat15 = Eigen::Matrix<double, 15, 15>;
  using Vec15 = Eigen::Matrix<double, 15, 1>;

  // Process noise densities (per sqrt(s)). Above the x500 IMU's datasheet
  // noise on purpose: they also cover what the model leaves out.
  struct Noise {
    double acc = 0.05;         // m/s^2
    double gyro = 0.005;       // rad/s
    double acc_bias = 1e-3;    // m/s^3, bias random walk
    double gyro_bias = 1e-4;   // rad/s^2
  } noise;

  Vec3 p = Vec3::Zero(), v = Vec3::Zero(), ab = Vec3::Zero(), wb = Vec3::Zero();
  Quat q = Quat::Identity();
  Mat15 P = Mat15::Identity();

  // Start at rest: roll and pitch from the accelerometer (it reads +g, up),
  // yaw from the magnetometer against the field it should see at yaw 0.
  void init(const Vec3 & p0, const Vec3 & acc, const Vec3 & mag, const Vec3 & mag_ref)
  {
    const Quat tilt = Quat::FromTwoVectors(acc, Vec3::UnitZ());
    const Vec3 m = tilt * mag;
    const double yaw = std::atan2(mag_ref.y(), mag_ref.x()) - std::atan2(m.y(), m.x());
    q = (Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ())) * tilt).normalized();
    p = p0;
    v.setZero();
    ab.setZero();
    wb.setZero();
    P.setZero();
    P.block<3, 3>(0, 0).diagonal().setConstant(0.1 * 0.1);     // m
    P.block<3, 3>(3, 3).diagonal().setConstant(0.1 * 0.1);     // m/s
    P.block<3, 3>(6, 6).diagonal().setConstant(0.05 * 0.05);   // rad, ~3 deg
    P.block<3, 3>(9, 9).diagonal().setConstant(0.2 * 0.2);     // m/s^2
    P.block<3, 3>(12, 12).diagonal().setConstant(0.01 * 0.01); // rad/s
  }

  // acc: specific force (m/s^2), gyro: body rates (rad/s), both body frame.
  void predict(const Vec3 & acc, const Vec3 & gyro, double dt)
  {
    const Vec3 f = acc - ab, w = gyro - wb;
    const Mat3 R = q.toRotationMatrix();
    const Vec3 a = R * f - kG * Vec3::UnitZ();
    const Quat dq = rotvec(w * dt);
    p += v * dt + 0.5 * a * dt * dt;
    v += a * dt;
    q = (q * dq).normalized();

    Mat15 F = Mat15::Identity();
    F.block<3, 3>(0, 3) = Mat3::Identity() * dt;
    F.block<3, 3>(3, 6) = -R * skew(f) * dt;
    F.block<3, 3>(3, 9) = -R * dt;
    F.block<3, 3>(6, 6) = dq.toRotationMatrix().transpose();
    F.block<3, 3>(6, 12) = -Mat3::Identity() * dt;
    P = F * P * F.transpose();
    const double n[4] = {noise.acc, noise.gyro, noise.acc_bias, noise.gyro_bias};
    for (int i = 0; i < 4; ++i) {
      P.block<3, 3>(3 + 3 * i, 3 + 3 * i).diagonal().array() += n[i] * n[i] * dt;
    }
  }

  // GNSS (or any) position, world frame. False if rejected by the gate.
  bool fuse_position(const Vec3 & z, double sigma_xy, double sigma_z)
  {
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 0).setIdentity();
    const Vec3 s(sigma_xy, sigma_xy, sigma_z);
    return update<3>(z - p, H, s.cwiseAbs2().asDiagonal());
  }

  // Height (barometer), world z.
  bool fuse_height(double z, double sigma)
  {
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0, 2) = 1.0;
    return update<1>(Eigen::Matrix<double, 1, 1>(z - p.z()), H,
      Eigen::Matrix<double, 1, 1>(sigma * sigma));
  }

  // Magnetometer: only its direction is used, so units don't matter.
  // ponytail: 3D fusion also nudges roll/pitch; fine with a clean sim field,
  // fuse heading only if real motor interference bends the field.
  bool fuse_mag(const Vec3 & mag, const Vec3 & mag_ref, double sigma)
  {
    const Vec3 h = q.conjugate() * mag_ref.normalized();
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, 6) = skew(h);
    return update<3>(mag.normalized() - h, H, Mat3::Identity() * sigma * sigma);
  }

  double gate = 5.0;  // innovation gate, in standard deviations

private:
  template<int M>
  bool update(const Eigen::Matrix<double, M, 1> & r, const Eigen::Matrix<double, M, 15> & H,
    const Eigen::Matrix<double, M, M> & Rm)
  {
    const Eigen::Matrix<double, M, M> S = H * P * H.transpose() + Rm;
    const Eigen::Matrix<double, M, M> S_inv = S.inverse();
    if ((r.transpose() * S_inv * r)(0, 0) > gate * gate * M) return false;
    const Eigen::Matrix<double, 15, M> K = P * H.transpose() * S_inv;
    const Vec15 dx = K * r;
    p += dx.segment<3>(0);
    v += dx.segment<3>(3);
    q = (q * rotvec(dx.segment<3>(6))).normalized();
    ab += dx.segment<3>(9);
    wb += dx.segment<3>(12);
    const Mat15 I_KH = Mat15::Identity() - K * H;
    P = I_KH * P * I_KH.transpose() + K * Rm * K.transpose();  // Joseph form
    P = 0.5 * (P + P.transpose());
    return true;
  }
};

// GNSS fix -> local ENU offset (m) from a reference fix, WGS84 radii.
// Flat-earth: fine within a few km of the reference.
inline Vec3 gnss_to_enu(double lat, double lon, double alt, double lat0, double lon0,
  double alt0)
{
  constexpr double a = 6378137.0, e2 = 6.69437999014e-3, d2r = M_PI / 180.0;
  const double s = std::sin(lat0 * d2r), w = 1.0 - e2 * s * s;
  const double r_n = a / std::sqrt(w);                    // prime vertical
  const double r_m = a * (1.0 - e2) / (w * std::sqrt(w)); // meridian
  return Vec3((lon - lon0) * d2r * r_n * std::cos(lat0 * d2r), (lat - lat0) * d2r * r_m,
    alt - alt0);
}

// Barometric height (m) above where the pressure was p0 (ISA troposphere).
inline double baro_height(double pressure, double p0)
{
  return 44330.77 * (1.0 - std::pow(pressure / p0, 0.190263));
}

}  // namespace fc
