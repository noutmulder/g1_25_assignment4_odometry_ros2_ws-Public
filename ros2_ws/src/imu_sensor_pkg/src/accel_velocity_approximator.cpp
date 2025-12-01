#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

class AccelVelocityApproximator : public rclcpp::Node
{
public:
  AccelVelocityApproximator() : Node("accel_velocity_approximator")
  {
    odom_frame_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
    remove_gravity_ = this->declare_parameter<bool>("remove_gravity_z", true);
    use_orientation_gravity_ = this->declare_parameter<bool>("use_orientation_for_gravity", false);
    gravity_m_s2_ = this->declare_parameter<double>("gravity_m_s2", 9.81);
    max_dt_seconds_ = this->declare_parameter<double>("max_dt_seconds", 0.2);

    // Use sensor data QoS for lower latency (best effort, small history).
    auto sensor_qos = rclcpp::SensorDataQoS();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("imu/odom", sensor_qos);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos,
      std::bind(&AccelVelocityApproximator::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "AccelVelocityApproximator ready (integrating /imu/data)");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rclcpp::Time stamp = msg->header.stamp;
    if (stamp.nanoseconds() == 0) {
      stamp = this->now();
    }

    if (!has_last_time_) {
      last_time_ = stamp;
      has_last_time_ = true;
      return;
    }

    double dt = (stamp - last_time_).seconds();
    if (dt <= 0.0) {
      last_time_ = stamp;
      return;
    }
    if (dt > max_dt_seconds_) {
      dt = max_dt_seconds_;
    }

    std::array<double, 3> accel{
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z};

    if (remove_gravity_) {
      if (use_orientation_gravity_) {
        const auto &q = msg->orientation;
        const double w = q.w;
        const double x = q.x;
        const double y = q.y;
        const double z = q.z;

        // Rotate world gravity vector (0,0,gravity) into the sensor frame: q^{-1} * g * q
        // Using quaternion math expanded for speed.
        const double gx = 2.0 * gravity_m_s2_ * (x * z - w * y);
        const double gy = 2.0 * gravity_m_s2_ * (y * z + w * x);
        const double gz = gravity_m_s2_ * (1.0 - 2.0 * (x * x + y * y));

        accel[0] -= gx;
        accel[1] -= gy;
        accel[2] -= gz;
      } else {
        accel[2] -= gravity_m_s2_;
      }
    }

    for (size_t i = 0; i < 3; ++i) {
      velocity_[i] += accel[i] * dt;
      position_[i] += velocity_[i] * dt;
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_[0];
    odom_msg.pose.pose.position.y = position_[1];
    odom_msg.pose.pose.position.z = position_[2];

    odom_msg.twist.twist.linear.x = velocity_[0];
    odom_msg.twist.twist.linear.y = velocity_[1];
    odom_msg.twist.twist.linear.z = velocity_[2];
    odom_msg.twist.twist.angular = msg->angular_velocity;

    odom_pub_->publish(odom_msg);
    last_time_ = stamp;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::string odom_frame_;
  std::string base_frame_;
  bool remove_gravity_;
  bool use_orientation_gravity_;
  double gravity_m_s2_;
  double max_dt_seconds_;

  std::array<double, 3> position_{{0.0, 0.0, 0.0}};
  std::array<double, 3> velocity_{{0.0, 0.0, 0.0}};
  rclcpp::Time last_time_;
  bool has_last_time_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AccelVelocityApproximator>());
  rclcpp::shutdown();
  return 0;
}
