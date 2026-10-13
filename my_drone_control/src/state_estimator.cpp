// State estimator node: the sensors -> /odom, for flight_controller. The
// filter is estimator.hpp.
//
// In:  /imu            sensor_msgs/Imu            predicts (each sample)
//      /navsat         sensor_msgs/NavSatFix      position
//      /air_pressure   sensor_msgs/FluidPressure  height
//      /magnetic_field sensor_msgs/MagneticField  heading (direction only)
// Out: /odom           nav_msgs/Odometry          frame odom, twist in body
//                                                 frame (as gz's odometry)
//
// The odom frame's origin is where the drone sat when the filter started
// (`origin`, default the spawn point at rest), so /odom minus
// /ground_truth/odom is the estimation error. The filter starts once every
// sensor has reported and the IMU has seen the drone still for a moment.
#include <cmath>
#include <memory>
#include <vector>

#include "my_drone_control/estimator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using fc::Vec3;

class StateEstimator : public rclcpp::Node {
public:
  StateEstimator()
  : Node("state_estimator")
  {
    auto vec = [this](const char * name, std::vector<double> def) {
        const auto v = declare_parameter(name, def);
        return Vec3(v.at(0), v.at(1), v.at(2));
      };
    origin_ = vec("origin", {0.0, 0.0, 0.0493});
    mag_ref_ = vec("mag_ref", {0.2162, 0.0095, 0.4295});
    ekf_.noise.acc = declare_parameter("acc_noise", ekf_.noise.acc);
    ekf_.noise.gyro = declare_parameter("gyro_noise", ekf_.noise.gyro);
    ekf_.noise.acc_bias = declare_parameter("acc_bias_noise", ekf_.noise.acc_bias);
    ekf_.noise.gyro_bias = declare_parameter("gyro_bias_noise", ekf_.noise.gyro_bias);
    ekf_.gate = declare_parameter("gate", ekf_.gate);
    gnss_xy_ = declare_parameter("gnss_noise_xy", 0.1);
    gnss_z_ = declare_parameter("gnss_noise_z", 0.3);
    baro_noise_ = declare_parameter("baro_noise", 0.3);
    mag_noise_ = declare_parameter("mag_noise", 0.05);
    period_ = 1.0 / declare_parameter("publish_rate", 50.0);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    const auto qos = rclcpp::SensorDataQoS();
    subs_.push_back(create_subscription<sensor_msgs::msg::Imu>("imu", qos,
      [this](sensor_msgs::msg::Imu::SharedPtr m) {on_imu(*m);}));
    subs_.push_back(create_subscription<sensor_msgs::msg::NavSatFix>("navsat", qos,
      [this](sensor_msgs::msg::NavSatFix::SharedPtr m) {on_navsat(*m);}));
    subs_.push_back(create_subscription<sensor_msgs::msg::FluidPressure>("air_pressure", qos,
      [this](sensor_msgs::msg::FluidPressure::SharedPtr m) {on_baro(*m);}));
    subs_.push_back(create_subscription<sensor_msgs::msg::MagneticField>("magnetic_field", qos,
      [this](sensor_msgs::msg::MagneticField::SharedPtr m) {on_mag(*m);}));
  }

private:
  static Vec3 v3(const geometry_msgs::msg::Vector3 & v) {return Vec3(v.x, v.y, v.z);}

  void on_imu(const sensor_msgs::msg::Imu & m)
  {
    const rclcpp::Time t = m.header.stamp;
    const Vec3 acc = v3(m.linear_acceleration), gyro = v3(m.angular_velocity);
    const double dt = last_imu_.nanoseconds() ? (t - last_imu_).seconds() : 0.0;
    last_imu_ = t;
    if (!started_) {
      try_start(acc, gyro);
      return;
    }
    if (dt <= 0.0 || dt > 0.05) return;  // a sim jump: skip, don't integrate it
    ekf_.predict(acc, gyro, dt);
    gyro_ = gyro;
    if ((t - last_pub_).seconds() >= period_ - 1e-4) {
      publish(t);
      last_pub_ = t;
    }
  }

  // Start when every sensor has reported and the drone has sat still (IMU
  // reads ~1 g, no rotation) for 1 s, so it isn't still falling from the
  // spawn height. The origin's height references are the barometer and GNSS
  // altitude averaged over that second: one barometer sample is +-0.25 m.
  void try_start(const Vec3 & acc, const Vec3 & gyro)
  {
    const bool still = std::abs(acc.norm() - fc::kG) < 0.05 * fc::kG && gyro.norm() < 0.05;
    if (!still) {
      still_count_ = baro_n_ = alt_n_ = 0;
      p0_ = alt0_ = 0.0;
    } else {
      ++still_count_;
    }
    if (!have_mag_ || baro_n_ < 25 || alt_n_ < 5 || still_count_ < 250) return;
    ekf_.init(origin_, acc, mag_, mag_ref_);
    started_ = true;
    RCLCPP_INFO(get_logger(), "started at %.6f %.6f %.2f m, yaw %.1f deg",
      lat0_, lon0_, alt0_, std::atan2(2.0 * (ekf_.q.w() * ekf_.q.z() + ekf_.q.x() * ekf_.q.y()),
      1.0 - 2.0 * (ekf_.q.y() * ekf_.q.y() + ekf_.q.z() * ekf_.q.z())) * 180.0 / M_PI);
  }

  void on_navsat(const sensor_msgs::msg::NavSatFix & m)
  {
    if (m.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return;
    if (!started_) {
      lat0_ = m.latitude, lon0_ = m.longitude;
      if (still_count_ > 0) alt0_ += (m.altitude - alt0_) / ++alt_n_;  // running mean
      return;
    }
    const Vec3 z = origin_ + fc::gnss_to_enu(m.latitude, m.longitude, m.altitude,
      lat0_, lon0_, alt0_);
    if (!ekf_.fuse_position(z, gnss_xy_, gnss_z_)) reject("GNSS");
  }

  void on_baro(const sensor_msgs::msg::FluidPressure & m)
  {
    if (!started_) {
      if (still_count_ > 0) p0_ += (m.fluid_pressure - p0_) / ++baro_n_;  // running mean
      return;
    }
    if (!ekf_.fuse_height(origin_.z() + fc::baro_height(m.fluid_pressure, p0_), baro_noise_)) {
      reject("barometer");
    }
  }

  void on_mag(const sensor_msgs::msg::MagneticField & m)
  {
    mag_ = v3(m.magnetic_field);
    have_mag_ = mag_.norm() > 0.0;
    if (started_ && have_mag_ && !ekf_.fuse_mag(mag_, mag_ref_, mag_noise_)) {
      reject("magnetometer");
    }
  }

  void reject(const char * sensor)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s rejected by the gate", sensor);
  }

  void publish(const rclcpp::Time & t)
  {
    nav_msgs::msg::Odometry o;
    o.header.stamp = t;
    o.header.frame_id = "odom";
    o.child_frame_id = "base_link";
    auto & pose = o.pose.pose;
    pose.position.x = ekf_.p.x();
    pose.position.y = ekf_.p.y();
    pose.position.z = ekf_.p.z();
    pose.orientation.w = ekf_.q.w();
    pose.orientation.x = ekf_.q.x();
    pose.orientation.y = ekf_.q.y();
    pose.orientation.z = ekf_.q.z();
    // Position and attitude variances (attitude error is body-frame, close
    // enough to the world-frame roll/pitch/yaw ROS expects for display).
    for (int i = 0; i < 3; ++i) {
      o.pose.covariance[7 * i] = ekf_.P(i, i);
      o.pose.covariance[21 + 7 * i] = ekf_.P(6 + i, 6 + i);
    }
    const Vec3 vb = ekf_.q.conjugate() * ekf_.v, wb = gyro_ - ekf_.wb;
    o.twist.twist.linear.x = vb.x();
    o.twist.twist.linear.y = vb.y();
    o.twist.twist.linear.z = vb.z();
    o.twist.twist.angular.x = wb.x();
    o.twist.twist.angular.y = wb.y();
    o.twist.twist.angular.z = wb.z();
    odom_pub_->publish(o);
  }

  fc::Ekf ekf_;
  Vec3 origin_, mag_ref_, mag_ = Vec3::Zero(), gyro_ = Vec3::Zero();
  double gnss_xy_, gnss_z_, baro_noise_, mag_noise_, period_;
  double lat0_ = 0.0, lon0_ = 0.0, alt0_ = 0.0, p0_ = 0.0;
  bool started_ = false, have_mag_ = false;
  int still_count_ = 0, baro_n_ = 0, alt_n_ = 0;
  rclcpp::Time last_imu_{0, 0, RCL_ROS_TIME}, last_pub_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}
