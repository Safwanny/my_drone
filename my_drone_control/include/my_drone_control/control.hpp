// Multicopter control cascade, PX4 style, in physical units and free of ROS
// so it can be unit tested:
//
//   position P -> velocity PID -> thrust vector (tilt limit) -> attitude P
//   (quaternion) -> body-rate PID -> torques -> mixer -> rotor speeds
//
// Frames: world ENU (x east/forward at spawn, z up), body FLU. Angles in rad.
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

namespace fc {

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;
using Mat3 = Eigen::Matrix3d;

constexpr double kG = 9.80665;

// Three independent PIDs. D acts on the measurement's derivative (no kick on
// setpoint steps); the integrator is clamped to +-i_max (anti-windup).
struct Pid3 {
  Vec3 kp = Vec3::Zero(), ki = Vec3::Zero(), kd = Vec3::Zero(), i_max = Vec3::Zero();
  Vec3 integ = Vec3::Zero();

  Vec3 update(const Vec3 & err, const Vec3 & meas_rate, double dt)
  {
    integ = (integ + ki.cwiseProduct(err) * dt).cwiseMax(-i_max).cwiseMin(i_max);
    return kp.cwiseProduct(err) + integ - kd.cwiseProduct(meas_rate);
  }
  void reset() { integ.setZero(); }
};

struct Rotor {
  double x, y;    // m from the CoM, body frame
  int direction;  // +1 ccw, -1 cw seen from above (gz turningDirection)
};

struct Airframe {
  double mass;                // kg
  Mat3 inertia;               // kg m^2 about the CoM
  double motor_constant;      // thrust = k * w^2   (N s^2/rad^2)
  double moment_constant;     // drag torque = c * thrust  (m)
  double max_rot_velocity;    // rad/s
  std::vector<Rotor> rotors;  // index == gz motorNumber
};

// ---------------------------------------------------------------- position
struct PositionControl {
  Vec3 pos_p = Vec3(1.0, 1.0, 1.0);  // (m/s) per m
  double vel_max_xy = 3.0, vel_max_up = 2.0, vel_max_down = 1.0;  // m/s
  Pid3 vel;                          // (m/s^2) per (m/s)

  Vec3 velocity_setpoint(const Vec3 & p_sp, const Vec3 & p) const
  {
    Vec3 v = pos_p.cwiseProduct(p_sp - p);
    const double xy = v.head<2>().norm();
    if (xy > vel_max_xy) v.head<2>() *= vel_max_xy / xy;
    v.z() = std::clamp(v.z(), -vel_max_down, vel_max_up);
    return v;
  }

  // World acceleration the vehicle should have, gravity not included.
  Vec3 accel_setpoint(const Vec3 & v_sp, const Vec3 & v, const Vec3 & a_meas, double dt)
  {
    return vel.update(v_sp - v, a_meas, dt);
  }
};

// --------------------------------------------- acceleration -> thrust/attitude
struct ThrustAttitude {
  double collective;  // N along body z
  Quat q_sp;
};

// PX4's prioritisation: vertical acceleration first, then whatever tilt and
// total thrust allow horizontally. a_max is the largest specific thrust
// (thrust / mass) we let the position loops ask for.
inline ThrustAttitude thrust_attitude(
  const Vec3 & a_sp, double yaw_sp, const Quat & q, double mass,
  double tilt_max, double a_max)
{
  Vec3 f = a_sp + Vec3(0, 0, kG);                   // specific thrust, world
  f.z() = std::clamp(f.z(), 0.1 * kG, a_max);
  const double xy_lim = std::min(
    f.z() * std::tan(tilt_max), std::sqrt(a_max * a_max - f.z() * f.z()));
  const double xy = f.head<2>().norm();
  if (xy > xy_lim) f.head<2>() *= xy_lim / xy;

  const Vec3 b3 = f.normalized();
  const Vec3 c1(std::cos(yaw_sp), std::sin(yaw_sp), 0);
  const Vec3 b2 = b3.cross(c1).normalized();  // b3 never horizontal: tilt limit
  Mat3 r;
  r << b2.cross(b3), b2, b3;

  // Thrust along the body z we have now, not the one we want: tilting
  // toward the target should not climb or sink on the way.
  const Vec3 body_z = q * Vec3::UnitZ();
  return {mass * std::max(0.0, f.dot(body_z)), Quat(r)};
}

// ---------------------------------------------------------------- attitude
struct AttitudeControl {
  Vec3 att_p = Vec3(6.0, 6.0, 3.0);    // (rad/s) per rad
  Vec3 rate_max = Vec3(3.0, 3.0, 2.0);  // rad/s

  // Plain per-axis quaternion P. PX4's reduced-attitude yaw
  // deprioritisation matters for aggressive large-yaw manoeuvres; add then.
  Vec3 rate_setpoint(const Quat & q_sp, const Quat & q) const
  {
    Quat e = q.conjugate() * q_sp;  // error in the body frame
    if (e.w() < 0) e.coeffs() *= -1;  // shortest way round
    const Vec3 w = 2.0 * att_p.cwiseProduct(e.vec());
    return w.cwiseMax(-rate_max).cwiseMin(rate_max);
  }
};

// --------------------------------------------------------------------- rate
struct RateControl {
  Pid3 pid;  // (rad/s^2) per (rad/s)

  Vec3 torque(const Vec3 & w_sp, const Vec3 & w, const Vec3 & w_dot,
              const Mat3 & inertia, double dt)
  {
    const Vec3 alpha = pid.update(w_sp - w, w_dot, dt);
    return inertia * alpha + w.cross(inertia * w);
  }
};

// -------------------------------------------------------------------- mixer
// [T, tx, ty, tz] = A * f, f = per-rotor thrust. Built from geometry, so any
// quad layout (true X, stretched X, ...) works without a hand table.
struct Mixer {
  Eigen::Matrix4d a, a_inv;
  double k, f_min, f_max;

  Mixer(const Airframe & af, double idle_fraction)
  : k(af.motor_constant)
  {
    for (int i = 0; i < 4; ++i) {
      const Rotor & r = af.rotors[i];
      // A ccw rotor (spinning +z) pushes the body the other way: -z torque.
      a.col(i) << 1.0, r.y, -r.x, -r.direction * af.moment_constant;
    }
    a_inv = a.inverse();
    const double w_idle = idle_fraction * af.max_rot_velocity;
    f_min = k * w_idle * w_idle;
    f_max = k * af.max_rot_velocity * af.max_rot_velocity;
  }

  // Yaw is the weakest axis (drag torque only), so it gets what is left:
  // thrust, roll and pitch first, then as much of the yaw torque as fits.
  // Roll/pitch beyond the limits are just clipped. PX4's airmode (shift
  // collective to keep attitude authority) if aggressive flight needs it.
  Eigen::Vector4d rotor_speeds(double collective, const Vec3 & torque) const
  {
    Eigen::Vector4d f = a_inv * Eigen::Vector4d(collective, torque.x(), torque.y(), 0.0);
    const Eigen::Vector4d f_yaw = a_inv.col(3) * torque.z();
    double s = 1.0;
    for (int i = 0; i < 4; ++i) {
      if (f_yaw[i] > 1e-12) s = std::min(s, (f_max - f[i]) / f_yaw[i]);
      if (f_yaw[i] < -1e-12) s = std::min(s, (f_min - f[i]) / f_yaw[i]);
    }
    f += std::clamp(s, 0.0, 1.0) * f_yaw;
    f = f.cwiseMax(f_min).cwiseMin(f_max);
    return (f / k).cwiseSqrt();
  }
};

}  // namespace fc
