// Flight controller node: replaces Gazebo's MulticopterVelocityControl with
// our own cascade (control.hpp) and drives the MulticopterMotorModel rotors
// directly through /motor_speed.
//
// In:  /enable    std_msgs/Bool                 arm / disarm
//      /cmd_vel   geometry_msgs/Twist           velocity mode: heading-frame
//                                               vx vy vz (m/s) + yaw rate
//      /goal_pose geometry_msgs/PoseStamped     position mode (odom frame)
//      /odom      nav_msgs/Odometry             state estimate (50 Hz)
//      /imu       sensor_msgs/Imu               gyro; each sample runs the loop
// Out: /motor_speed actuator_msgs/Actuators     rotor speeds, rad/s
//
// Like a real FC the loop is triggered by the gyro, and attitude is the last
// estimate propagated with the gyro until the next /odom arrives.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <tinyxml2.h>

#include "actuator_msgs/msg/actuators.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"

#include "control.hpp"

using fc::Quat;
using fc::Vec3;

namespace {

// Reads the <quad_physics> element quad_gazebo.xacro puts in the URDF, so
// mass, inertia and rotor layout come from the same numbers Gazebo uses.
fc::Airframe parse_airframe(const std::string & urdf)
{
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("robot_description is not valid XML");
  }
  const auto * root = doc.RootElement();
  const auto * qp = root ? root->FirstChildElement("quad_physics") : nullptr;
  if (!qp) {
    throw std::runtime_error("robot_description has no <quad_physics> element");
  }
  fc::Airframe af;
  af.mass = qp->DoubleAttribute("mass");
  af.inertia = Vec3(qp->DoubleAttribute("ixx"), qp->DoubleAttribute("iyy"),
                    qp->DoubleAttribute("izz")).asDiagonal();
  af.motor_constant = qp->DoubleAttribute("motor_constant");
  af.moment_constant = qp->DoubleAttribute("moment_constant");
  af.max_rot_velocity = qp->DoubleAttribute("max_rot_velocity");
  af.rotors.resize(4);
  int n = 0;
  for (auto * r = qp->FirstChildElement("rotor"); r; r = r->NextSiblingElement("rotor"), ++n) {
    const int i = r->IntAttribute("number", -1);
    if (i < 0 || i > 3) throw std::runtime_error("<rotor number> must be 0..3");
    af.rotors[i] = {r->DoubleAttribute("x"), r->DoubleAttribute("y"), r->IntAttribute("direction")};
  }
  if (n != 4) throw std::runtime_error("<quad_physics> needs exactly 4 <rotor>s");
  return af;
}

double yaw_of(const Quat & q)
{
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

Vec3 v3(const geometry_msgs::msg::Vector3 & v) { return {v.x, v.y, v.z}; }

}  // namespace

class FlightController : public rclcpp::Node
{
public:
  FlightController()
  : Node("flight_controller"),
    af_(parse_airframe(declare_parameter<std::string>("robot_description"))),
    mixer_(af_, declare_parameter("idle_fraction", 0.10))
  {
    auto vec = [this](const std::string & name, const std::vector<double> & def) {
        auto v = declare_parameter(name, def);
        return Vec3(v.at(0), v.at(1), v.at(2));
      };
    pos_.pos_p = vec("pos_p", {1.0, 1.0, 1.0});
    pos_.vel_max_xy = declare_parameter("vel_max_xy", 3.0);
    pos_.vel_max_up = declare_parameter("vel_max_up", 2.0);
    pos_.vel_max_down = declare_parameter("vel_max_down", 1.0);
    pos_.vel.kp = vec("vel_p", {2.0, 2.0, 4.0});
    pos_.vel.ki = vec("vel_i", {0.4, 0.4, 1.0});
    pos_.vel.kd = vec("vel_d", {0.1, 0.1, 0.0});
    pos_.vel.i_max = vec("vel_i_max", {2.0, 2.0, 3.0});
    att_.att_p = vec("att_p", {6.0, 6.0, 3.0});
    att_.rate_max = vec("rate_max", {3.0, 3.0, 2.0});
    rate_.pid.kp = vec("rate_p", {18.0, 18.0, 8.0});
    rate_.pid.ki = vec("rate_i", {4.0, 4.0, 2.0});
    rate_.pid.kd = vec("rate_d", {0.05, 0.05, 0.0});
    rate_.pid.i_max = vec("rate_i_max", {2.0, 2.0, 1.0});
    tilt_max_ = declare_parameter("tilt_max_deg", 35.0) * M_PI / 180.0;
    // Leave headroom above what the position loops may ask for, so the
    // attitude loop still has differential thrust to work with.
    const double a_full = 4.0 * mixer_.f_max / af_.mass;
    a_max_ = declare_parameter("thrust_max_fraction", 0.8) * a_full;
    takeoff_alt_ = declare_parameter("default_goal_alt", 1.5);
    cmd_timeout_ = declare_parameter("cmd_vel_timeout", 0.5);
    odom_timeout_ = declare_parameter("odom_timeout", 0.2);

    RCLCPP_INFO(get_logger(),
      "airframe %.3f kg, hover %.0f rad/s per rotor, thrust/weight %.1f",
      af_.mass, std::sqrt(af_.mass * fc::kG / 4.0 / af_.motor_constant), a_full / fc::kG);

    motor_pub_ = create_publisher<actuator_msgs::msg::Actuators>("/motor_speed", 10);
    create_sub<std_msgs::msg::Bool>("/enable", &FlightController::on_enable);
    create_sub<geometry_msgs::msg::Twist>("/cmd_vel", &FlightController::on_cmd_vel);
    create_sub<geometry_msgs::msg::PoseStamped>("/goal_pose", &FlightController::on_goal);
    create_sub<nav_msgs::msg::Odometry>("/odom", &FlightController::on_odom);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr m) {on_imu(*m);});
  }

private:
  enum class Mode { kPosition, kVelocity };

  template<typename T>
  void create_sub(const std::string & topic, void (FlightController::*cb)(const T &))
  {
    subs_.push_back(create_subscription<T>(
      topic, 10, [this, cb](typename T::SharedPtr m) {(this->*cb)(*m);}));
  }

  void on_enable(const std_msgs::msg::Bool & m)
  {
    if (m.data == armed_) return;
    armed_ = m.data;
    airborne_ = false;
    RCLCPP_INFO(get_logger(), armed_ ? "armed" : "disarmed");
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist & m)
  {
    cmd_vel_ = Vec3(m.linear.x, m.linear.y, m.linear.z);
    cmd_yaw_rate_ = m.angular.z;
    last_cmd_ = now();
    mode_ = Mode::kVelocity;
  }

  void on_goal(const geometry_msgs::msg::PoseStamped & m)
  {
    const auto & p = m.pose.position;
    // RViz's 2D Goal Pose arrives with z = 0: keep the altitude we fly at.
    const double z = p.z > 0.01 ? p.z : (airborne_ ? p_sp_.z() : takeoff_alt_);
    p_sp_ = Vec3(p.x, p.y, z);
    const auto & o = m.pose.orientation;
    if (o.x || o.y || o.z || o.w) yaw_sp_ = yaw_of(Quat(o.w, o.x, o.y, o.z));
    mode_ = Mode::kPosition;
    RCLCPP_INFO(get_logger(), "goal %.2f %.2f %.2f yaw %.2f",
      p_sp_.x(), p_sp_.y(), p_sp_.z(), yaw_sp_);
  }

  void on_odom(const nav_msgs::msg::Odometry & m)
  {
    const auto & o = m.pose.pose.orientation;
    q_ = Quat(o.w, o.x, o.y, o.z).normalized();
    p_ = Vec3(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z);
    // gz OdometryPublisher reports the twist in the body frame.
    const Vec3 v = q_ * v3(m.twist.twist.linear);
    const rclcpp::Time t = m.header.stamp;
    if (have_odom_) {
      const double dt = (t - last_odom_).seconds();
      if (dt > 1e-4) a_meas_ += 0.5 * ((v - v_) / dt - a_meas_);  // light low-pass
    }
    v_ = v;
    last_odom_ = t;
    have_odom_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu & m)
  {
    const rclcpp::Time t = m.header.stamp;
    const Vec3 w = v3(m.angular_velocity);
    const double dt = last_imu_.nanoseconds() ? (t - last_imu_).seconds() : 0.0;
    last_imu_ = t;
    if (dt <= 0.0 || dt > 0.05) return;  // first sample, or a sim jump

    // Propagate attitude with the gyro between /odom updates.
    const double ang = w.norm() * dt;
    if (ang > 1e-9) q_ = (q_ * Quat(Eigen::AngleAxisd(ang, w.normalized()))).normalized();
    w_dot_ += 0.2 * ((w - w_prev_) / dt - w_dot_);  // gyro derivative, filtered
    w_prev_ = w;

    publish(armed_ && have_odom_ ? control(w, t, dt) : Eigen::Vector4d::Zero());
  }

  Eigen::Vector4d control(const Vec3 & w, const rclcpp::Time & t, double dt)
  {
    const double yaw = yaw_of(q_);
    const bool odom_ok = (t - last_odom_).seconds() < odom_timeout_;

    if (!airborne_) {
      // On the ground: idle props, aim to hold here, keep integrators empty.
      // Take off once something asks to go up.
      const bool climb = mode_ == Mode::kVelocity ?
        cmd_vel_.z() > 0.1 && (t - last_cmd_).seconds() < cmd_timeout_ :
        p_sp_.z() > p_.z() + 0.15;
      if (!climb || !odom_ok) {
        if (mode_ == Mode::kVelocity) p_sp_ = p_;
        yaw_sp_ = yaw;
        pos_.vel.reset();
        rate_.pid.reset();
        return mixer_.rotor_speeds(0.0, Vec3::Zero());
      }
      airborne_ = true;
      RCLCPP_INFO(get_logger(), "taking off");
    }

    double collective;
    Quat q_sp;
    if (!odom_ok) {
      // Failsafe: no state estimate. Level out on the gyro-propagated
      // attitude and sink gently until someone disarms.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odom lost: level descent");
      collective = 0.9 * af_.mass * fc::kG;
      q_sp = Quat(Eigen::AngleAxisd(yaw_sp_, Vec3::UnitZ()));
    } else {
      const Vec3 v_sp = velocity_setpoint(t, yaw, dt);
      const Vec3 a_sp = pos_.accel_setpoint(v_sp, v_, a_meas_, dt);
      const auto ta = fc::thrust_attitude(a_sp, yaw_sp_, q_, af_.mass, tilt_max_, a_max_);
      collective = ta.collective;
      q_sp = ta.q_sp;
    }
    const Vec3 w_sp = att_.rate_setpoint(q_sp, q_);
    const Vec3 torque = rate_.torque(w_sp, w, w_dot_, af_.inertia, dt);
    return mixer_.rotor_speeds(collective, torque);
  }

  // Velocity mode flies the stick; when the stick is centred (or silent) it
  // brakes, then latches a position hold, as PX4's Position mode does.
  Vec3 velocity_setpoint(const rclcpp::Time & t, double yaw, double dt)
  {
    if (mode_ == Mode::kVelocity) {
      const bool fresh = (t - last_cmd_).seconds() < cmd_timeout_;
      const bool yaw_stick = fresh && std::abs(cmd_yaw_rate_) > 1e-3;
      if (yaw_stick) {
        // Integrate the yaw-rate command, never far ahead of the real yaw.
        yaw_sp_ = yaw + std::clamp(wrap(yaw_sp_ + cmd_yaw_rate_ * dt - yaw), -0.3, 0.3);
      } else if (yawing_) {
        yaw_sp_ = yaw;  // stick released: stop turning here, don't catch up
      }
      yawing_ = yaw_stick;
      if (yaw_stick || (fresh && cmd_vel_.norm() > 1e-3)) {
        const Vec3 heading(cmd_vel_.x() * std::cos(yaw) - cmd_vel_.y() * std::sin(yaw),
                           cmd_vel_.x() * std::sin(yaw) + cmd_vel_.y() * std::cos(yaw),
                           cmd_vel_.z());
        p_sp_ = p_;
        return heading;
      }
      if (v_.norm() > 0.3) {  // brake first, so the hold point is not behind us
        p_sp_ = p_;
        return Vec3::Zero();
      }
      mode_ = Mode::kPosition;  // latch the hold
      p_sp_ = p_;
    }
    return pos_.velocity_setpoint(p_sp_, p_);
  }

  void publish(const Eigen::Vector4d & w)
  {
    actuator_msgs::msg::Actuators m;
    m.header.stamp = last_imu_;
    m.velocity.assign(w.data(), w.data() + 4);
    motor_pub_->publish(m);
  }

  fc::Airframe af_;
  fc::Mixer mixer_;
  fc::PositionControl pos_;
  fc::AttitudeControl att_;
  fc::RateControl rate_;
  double tilt_max_, a_max_, takeoff_alt_, cmd_timeout_, odom_timeout_;

  bool armed_ = false, airborne_ = false, have_odom_ = false, yawing_ = false;
  Mode mode_ = Mode::kPosition;
  Vec3 p_ = Vec3::Zero(), v_ = Vec3::Zero(), a_meas_ = Vec3::Zero();
  Quat q_ = Quat::Identity();
  Vec3 w_prev_ = Vec3::Zero(), w_dot_ = Vec3::Zero();
  Vec3 p_sp_ = Vec3::Zero(), cmd_vel_ = Vec3::Zero();
  double yaw_sp_ = 0.0, cmd_yaw_rate_ = 0.0;
  rclcpp::Time last_cmd_{0, 0, RCL_ROS_TIME}, last_odom_{0, 0, RCL_ROS_TIME},
  last_imu_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightController>());
  rclcpp::shutdown();
  return 0;
}
