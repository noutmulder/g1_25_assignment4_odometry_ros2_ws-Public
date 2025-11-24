#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Dense>
#include <deque>

class ImuVectorVisualizer : public rclcpp::Node
{
public:
    ImuVectorVisualizer()
        : Node("imu_vector_visualizer"),
          prev_accel_(Eigen::Vector3f::Zero())
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ImuVectorVisualizer::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMU Vector Visualizer started (gravity compensated + smoothing + trail)");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // --- Bereken zwaartekrachtcorrectie ---
        Eigen::Quaternionf q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        // Gebruik inverse om wereldzwaartekracht (0,0,9.81) naar sensor-frame te brengen
        Eigen::Vector3f gravity = q.inverse() * Eigen::Vector3f(0, 0, 9.81f);

        Eigen::Vector3f accel(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);

        Eigen::Vector3f motion_accel = accel - gravity;

        // --- Low-pass filter / smoothing ---
        float alpha = 0.4f;
        motion_accel = alpha * motion_accel + (1.0f - alpha) * prev_accel_;
        prev_accel_ = motion_accel;

        // Drempel om mini-ruis te onderdrukken
        if (motion_accel.norm() < 0.05f)
            motion_accel.setZero();

        // --- Kleur op basis van magnitude ---
        float magnitude = motion_accel.norm();
        float max_accel = 5.0f;
        float intensity = std::min(magnitude / max_accel, 1.0f);
        float r = intensity;
        float g = 1.0f - intensity;
        float b = 0.0f;

        // --- Maak nieuw trailpunt (richtingvector, niet cumulatief) ---
        double scale = 0.3;
        geometry_msgs::msg::Point p;
        p.x = motion_accel.x() * scale;
        p.y = motion_accel.y() * scale;
        p.z = motion_accel.z() * scale;

        trail_.push_back(p);
        if (trail_.size() > 25)
            trail_.pop_front();

        // --- Maak markers ---
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "imu_link";
        arrow.header.stamp = this->now();
        arrow.ns = "imu_arrow";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point p_start;
        p_start.x = p_start.y = p_start.z = 0.0;
        arrow.points = {p_start, p};
        arrow.scale.x = 0.05;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.color.a = 1.0;
        arrow.color.r = r;
        arrow.color.g = g;
        arrow.color.b = b;

        visualization_msgs::msg::Marker trail;
        trail.header = arrow.header;
        trail.ns = "imu_trail";
        trail.id = 1;
        trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trail.action = visualization_msgs::msg::Marker::ADD;
        trail.scale.x = 0.02;
        trail.color.a = 0.7;
        trail.color.r = r;
        trail.color.g = g;
        trail.color.b = b;
        trail.points.assign(trail_.begin(), trail_.end());

        // --- Combineer in MarkerArray ---
        visualization_msgs::msg::MarkerArray array;
        array.markers.push_back(arrow);
        array.markers.push_back(trail);

        marker_pub_->publish(array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    Eigen::Vector3f prev_accel_;
    std::deque<geometry_msgs::msg::Point> trail_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuVectorVisualizer>());
    rclcpp::shutdown();
    return 0;
}