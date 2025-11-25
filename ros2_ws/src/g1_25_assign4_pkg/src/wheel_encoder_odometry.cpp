#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "g1_25_assign4_interfaces_pkg/msg/wheel_encoder.hpp"

class WheelEncoderOdometry : public rclcpp::Node
{
public:
  WheelEncoderOdometry() : Node("wheel_encoder_odometry")
  {
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.05); // meters
    track_width_ = this->declare_parameter<double>("track_width", 0.20);   // meters between left/right
    odom_frame_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame_id", "base_link");

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel/odom", 10);
    enc_sub_ = this->create_subscription<g1_25_assign4_interfaces_pkg::msg::WheelEncoder>(
      "/wheel/encoder", 10, std::bind(&WheelEncoderOdometry::encoder_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "WheelEncoderOdometry ready (track=%.3f m, radius=%.3f m)",
                track_width_, wheel_radius_);
  }

private:
  void encoder_callback(const g1_25_assign4_interfaces_pkg::msg::WheelEncoder::SharedPtr msg)
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

    // Use front wheel pair as differential; fallback to rear if zeros.
    double left_w = msg->front_left;
    double right_w = msg->front_right;
    if (std::abs(left_w) < 1e-6 && std::abs(right_w) < 1e-6) {
      left_w = msg->rear_left;
      right_w = msg->rear_right;
    }

    // Convert wheel angular velocity to linear velocity.
    double v_left = left_w * wheel_radius_;
    double v_right = right_w * wheel_radius_;

    double v = (v_left + v_right) * 0.5;
    double w = (v_right - v_left) / track_width_;

    // Integrate pose in 2D (x, y, yaw). Keep z=0.
    pose_yaw_ += w * dt;
    pose_x_ += v * std::cos(pose_yaw_) * dt;
    pose_y_ += v * std::sin(pose_yaw_) * dt;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = pose_x_;
    odom.pose.pose.position.y = pose_y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose_yaw_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = v * std::cos(pose_yaw_);
    odom.twist.twist.linear.y = v * std::sin(pose_yaw_);
    odom.twist.twist.angular.z = w;

    odom_pub_->publish(odom);
    last_time_ = stamp;
  }

  rclcpp::Subscription<g1_25_assign4_interfaces_pkg::msg::WheelEncoder>::SharedPtr enc_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  double wheel_radius_{0.05};
  double track_width_{0.20};
  std::string odom_frame_;
  std::string base_frame_;

  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};

  rclcpp::Time last_time_;
  bool has_last_time_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelEncoderOdometry>());
  rclcpp::shutdown();
  return 0;
}
