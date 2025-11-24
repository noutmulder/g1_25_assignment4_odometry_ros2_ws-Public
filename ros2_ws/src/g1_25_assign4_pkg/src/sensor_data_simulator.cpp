#include <array>
#include <cmath>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SensorDataSimulator : public rclcpp::Node
{
public:
  SensorDataSimulator() : Node("sensor_data_simulator"), rng_(std::random_device{}())
  {
    rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    accel_amp_ = this->declare_parameter<double>("accel_amp", 0.5);          // m/s^2 amplitude (x)
    accel_bias_ = this->declare_parameter<double>("accel_bias", 0.0);        // m/s^2 bias (x)
    accel_freq_hz_ = this->declare_parameter<double>("accel_freq_hz", 0.2);  // sine frequency
    gyro_z_bias_ = this->declare_parameter<double>("gyro_z_bias", 0.1);      // rad/s bias
    gyro_z_noise_std_ = this->declare_parameter<double>("gyro_z_noise_std", 0.01);
    accel_noise_std_ = this->declare_parameter<double>("accel_noise_std", 0.05);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 20);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("imu/odom", 20);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_),
      std::bind(&SensorDataSimulator::on_timer, this));

    last_time_ = this->now();
    start_time_ = last_time_;

    RCLCPP_INFO(this->get_logger(), "SensorDataSimulator running at %.1f Hz", rate_hz_);
  }

private:
  void on_timer()
  {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) {
      return;
    }
    last_time_ = now;

    double t = (now - start_time_).seconds();

    // Generate deterministic base signals
    double accel_x = accel_bias_ + accel_amp_ * std::sin(2.0 * M_PI * accel_freq_hz_ * t);
    double accel_y = 0.0;
    double accel_z = 0.0;

    double gyro_z = gyro_z_bias_;

    // Add noise
    accel_x += accel_noise_std_ * normal_(rng_);
    accel_y += accel_noise_std_ * normal_(rng_);
    accel_z += accel_noise_std_ * normal_(rng_);
    gyro_z += gyro_z_noise_std_ * normal_(rng_);

    // Integrate to velocity/position (simple flat-world model)
    velocity_[0] += accel_x * dt;
    velocity_[1] += accel_y * dt;
    velocity_[2] += accel_z * dt;

    position_[0] += velocity_[0] * dt;
    position_[1] += velocity_[1] * dt;
    position_[2] += velocity_[2] * dt;

    yaw_ += gyro_z * dt;

    // Prepare IMU message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = frame_id_;
    imu_msg.linear_acceleration.x = accel_x;
    imu_msg.linear_acceleration.y = accel_y;
    imu_msg.linear_acceleration.z = accel_z;
    imu_msg.angular_velocity.z = gyro_z;

    // Simple yaw-only orientation
    double cy = std::cos(yaw_ * 0.5);
    double sy = std::sin(yaw_ * 0.5);
    imu_msg.orientation.w = cy;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = sy;

    imu_pub_->publish(imu_msg);

    // Ground-truth odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = position_[0];
    odom.pose.pose.position.y = position_[1];
    odom.pose.pose.position.z = position_[2];
    odom.pose.pose.orientation = imu_msg.orientation;
    odom.twist.twist.linear.x = velocity_[0];
    odom.twist.twist.linear.y = velocity_[1];
    odom.twist.twist.linear.z = velocity_[2];
    odom.twist.twist.angular = imu_msg.angular_velocity;

    odom_pub_->publish(odom);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  double rate_hz_;
  double accel_amp_;
  double accel_bias_;
  double accel_freq_hz_;
  double gyro_z_bias_;
  double gyro_z_noise_std_;
  double accel_noise_std_;
  std::string frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  std::array<double, 3> velocity_{{0.0, 0.0, 0.0}};
  std::array<double, 3> position_{{0.0, 0.0, 0.0}};
  double yaw_{0.0};

  rclcpp::Time last_time_;
  rclcpp::Time start_time_;

  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataSimulator>());
  rclcpp::shutdown();
  return 0;
}
