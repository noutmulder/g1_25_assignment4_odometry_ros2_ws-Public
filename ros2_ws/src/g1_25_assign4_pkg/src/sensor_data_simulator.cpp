#include <chrono>
#include <cmath>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "g1_25_assign4_interfaces_pkg/msg/wheel_encoder.hpp"

using namespace std::chrono_literals;

// Simple simulator that publishes synthetic IMU and wheel encoder data so downstream nodes
// can be developed/tested without hardware.
class SensorDataSimulator : public rclcpp::Node
{
public:
  SensorDataSimulator() : Node("sensor_data_simulator")
  {
    // Parameters to tweak the simulated motion/noise
    imu_rate_hz_ = this->declare_parameter<int>("imu_rate_hz", 50);
    wheel_rate_hz_ = this->declare_parameter<int>("wheel_rate_hz", 25);
    accel_noise_std_ = this->declare_parameter<double>("accel_noise_std", 0.05);
    gyro_noise_std_ = this->declare_parameter<double>("gyro_noise_std", 0.01);
    wheel_noise_std_ = this->declare_parameter<double>("wheel_noise_std", 0.02);
    forward_accel_ = this->declare_parameter<double>("forward_accel", 0.5);
    turn_rate_ = this->declare_parameter<double>("turn_rate", 0.2); // rad/s yaw
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.05); // meters
    wheel_base_ = this->declare_parameter<double>("wheel_base", 0.20);     // meters (left-right)
    use_mecanum_ = this->declare_parameter<bool>("use_mecanum", false);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    wheel_pub_ = this->create_publisher<g1_25_assign4_interfaces_pkg::msg::WheelEncoder>("/wheel/encoder", 10);

    // IMU timer
    imu_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / std::max(1, imu_rate_hz_)),
      std::bind(&SensorDataSimulator::publish_imu, this));

    // Wheel timer
    wheel_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / std::max(1, wheel_rate_hz_)),
      std::bind(&SensorDataSimulator::publish_wheels, this));

    RCLCPP_INFO(this->get_logger(),
                "SensorDataSimulator started (imu_rate=%d Hz, wheel_rate=%d Hz, mecanum=%s)",
                imu_rate_hz_, wheel_rate_hz_, use_mecanum_ ? "true" : "false");
  }

private:
  void publish_imu()
  {
    auto now = this->now();
    double t = now.seconds();

    // Simple scenario: forward acceleration and a gentle yaw turn.
    sensor_msgs::msg::Imu imu{};
    imu.header.stamp = now;
    imu.header.frame_id = "imu_link";

    imu.linear_acceleration.x = forward_accel_ + noise(accel_noise_std_);
    imu.linear_acceleration.y = 0.0 + noise(accel_noise_std_);
    imu.linear_acceleration.z = 9.81 + noise(accel_noise_std_); // keep gravity in so downstream can choose

    imu.angular_velocity.x = 0.0 + noise(gyro_noise_std_);
    imu.angular_velocity.y = 0.0 + noise(gyro_noise_std_);
    imu.angular_velocity.z = turn_rate_ + noise(gyro_noise_std_);

    // Identity orientation; downstream can choose to ignore/correct gravity.
    imu.orientation.w = 1.0;
    imu.orientation.x = imu.orientation.y = imu.orientation.z = 0.0;

    imu_pub_->publish(imu);
  }

  void publish_wheels()
  {
    auto now = this->now();
    double v_forward = forward_accel_ * 0.5; // crude steady velocity representation
    double w_z = turn_rate_;

    // Differential approximation; for mecanum we keep it simple and mirror differential.
    double left_vel = (2 * v_forward - w_z * wheel_base_) / (2 * wheel_radius_);
    double right_vel = (2 * v_forward + w_z * wheel_base_) / (2 * wheel_radius_);

    g1_25_assign4_interfaces_pkg::msg::WheelEncoder enc{};
    enc.header.stamp = now;
    enc.header.frame_id = "base_link";

    if (use_mecanum_) {
      enc.front_left = left_vel + noise(wheel_noise_std_);
      enc.rear_left = left_vel + noise(wheel_noise_std_);
      enc.front_right = right_vel + noise(wheel_noise_std_);
      enc.rear_right = right_vel + noise(wheel_noise_std_);
    } else {
      enc.front_left = left_vel + noise(wheel_noise_std_);
      enc.rear_left = left_vel + noise(wheel_noise_std_);
      enc.front_right = right_vel + noise(wheel_noise_std_);
      enc.rear_right = right_vel + noise(wheel_noise_std_);
    }

    wheel_pub_->publish(enc);
  }

  double noise(double stddev)
  {
    if (stddev <= 0.0) {
      return 0.0;
    }
    std::normal_distribution<double> dist(0.0, stddev);
    return dist(rng_);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<g1_25_assign4_interfaces_pkg::msg::WheelEncoder>::SharedPtr wheel_pub_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr wheel_timer_;

  int imu_rate_hz_{50};
  int wheel_rate_hz_{25};
  double accel_noise_std_{0.05};
  double gyro_noise_std_{0.01};
  double wheel_noise_std_{0.02};
  double forward_accel_{0.5};
  double turn_rate_{0.2};
  double wheel_radius_{0.05};
  double wheel_base_{0.20};
  bool use_mecanum_{false};

  std::mt19937 rng_{std::random_device{}()};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataSimulator>());
  rclcpp::shutdown();
  return 0;
}
