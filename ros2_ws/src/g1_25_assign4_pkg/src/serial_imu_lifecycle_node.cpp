#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SerialIMULifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    SerialIMULifecycleNode() : LifecycleNode("serial_imu_node"), serial_fd_(-1)
    {
        // Declare parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyAMC1");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<int>("publish_rate_ms", 100);
        
        RCLCPP_INFO(this->get_logger(), "SerialIMULifecycleNode constructed");
    }

    ~SerialIMULifecycleNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Configuring SerialIMULifecycleNode");
        
        // Get parameters
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        
        // Create publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temperature", 10);
        
        RCLCPP_INFO(this->get_logger(), "Configured with port: %s, baud: %d", 
                    serial_port_.c_str(), baud_rate_);
        
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Activating SerialIMULifecycleNode");
        
        // Open serial port
        serial_fd_ = open(serial_port_.c_str(), O_RDONLY | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
            return LifecycleCallbackReturn::ERROR;
        }
        
        // Configure serial port
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            close(serial_fd_);
            serial_fd_ = -1;
            return LifecycleCallbackReturn::ERROR;
        }
        
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 10;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
            close(serial_fd_);
            serial_fd_ = -1;
            return LifecycleCallbackReturn::ERROR;
        }
        
        // Flush any existing data in serial buffer
        tcflush(serial_fd_, TCIOFLUSH);
        buffer_.clear();
        
        RCLCPP_INFO(this->get_logger(), "Serial buffer flushed, waiting for clean data...");
        
        // Activate publishers
        imu_pub_->on_activate();
        temp_pub_->on_activate();
        
        // Start timer
        int rate_ms = this->get_parameter("publish_rate_ms").as_int();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(rate_ms),
            std::bind(&SerialIMULifecycleNode::read_and_publish, this));
        
        RCLCPP_INFO(this->get_logger(), "Serial port opened and activated");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating SerialIMULifecycleNode");
        
        timer_->cancel();
        imu_pub_->on_deactivate();
        temp_pub_->on_deactivate();
        
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up SerialIMULifecycleNode");
        
        imu_pub_.reset();
        temp_pub_.reset();
        
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down SerialIMULifecycleNode");
        
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        
        return LifecycleCallbackReturn::SUCCESS;
    }

private:
    void read_and_publish()
    {
        if (serial_fd_ < 0) return;
        
        // Read line from serial
        std::string line;
        char buf[256];
        ssize_t n = read(serial_fd_, buf, sizeof(buf) - 1);
        
        if (n > 0) {
            buf[n] = '\0';
            buffer_ += std::string(buf);
            
            // Process complete lines
            size_t pos;
            while ((pos = buffer_.find('\n')) != std::string::npos) {
                line = buffer_.substr(0, pos);
                buffer_.erase(0, pos + 1);
                
                // Remove carriage return if present
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }
                
                // Skip empty lines
                if (line.empty()) continue;
                
                // Handle multiple IMU messages in one line (split by "IMU" marker)
                size_t imu_pos = 0;
                while ((imu_pos = line.find("IMU", imu_pos)) != std::string::npos) {
                    // Find the end of this IMU message (next "IMU" or end of string)
                    size_t next_imu = line.find("IMU", imu_pos + 3);
                    std::string imu_data;
                    
                    if (next_imu != std::string::npos) {
                        // Extract until next IMU
                        imu_data = line.substr(imu_pos, next_imu - imu_pos);
                    } else {
                        // Extract until end of line
                        imu_data = line.substr(imu_pos);
                    }
                    
                    // Parse this IMU message
                    if (!imu_data.empty()) {
                        parse_and_publish(imu_data);
                    }
                    
                    // Move to next potential IMU message
                    if (next_imu != std::string::npos) {
                        imu_pos = next_imu;
                    } else {
                        break;
                    }
                }
            }
        }
    }

    void parse_and_publish(const std::string& line)
    {
        std::istringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        // Validate format: IMU,timestamp,ax,ay,az,gx,gy,gz,temp
        if (tokens.size() != 9 || tokens[0] != "IMU") {
            RCLCPP_WARN(this->get_logger(), "Invalid IMU data format (expected 9 fields starting with 'IMU', got %zu fields): %s", 
                        tokens.size(), line.c_str());
            return;
        }
        
        try {
            // Parse data: IMU,timestamp,ax,ay,az,gx,gy,gz,temp
            auto imu_msg = sensor_msgs::msg::Imu();
            auto temp_msg = sensor_msgs::msg::Temperature();

            // Parse sensor-provided timestamp (milliseconds) and convert to ROS time
            uint64_t timestamp_ms = 0;
            try {
                timestamp_ms = std::stoull(tokens[1]);
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse timestamp_ms '%s', using now()", tokens[1].c_str());
                auto now = this->now();
                imu_msg.header.stamp = now;
                imu_msg.header.frame_id = "imu_link";
                temp_msg.header = imu_msg.header;
                // continue parsing values below
            }

            if (timestamp_ms != 0) {
                uint64_t sec = timestamp_ms / 1000u;
                uint64_t nsec = (timestamp_ms % 1000u) * 1000000u;
                imu_msg.header.stamp.sec = static_cast<int32_t>(sec);
                imu_msg.header.stamp.nanosec = static_cast<uint32_t>(nsec);
                imu_msg.header.frame_id = "imu_link";
                temp_msg.header = imu_msg.header;
            }
            
            // Linear acceleration
            imu_msg.linear_acceleration.x = std::stod(tokens[2]);
            imu_msg.linear_acceleration.y = std::stod(tokens[3]);
            imu_msg.linear_acceleration.z = std::stod(tokens[4]);
            
            // Angular velocity
            imu_msg.angular_velocity.x = std::stod(tokens[5]);
            imu_msg.angular_velocity.y = std::stod(tokens[6]);
            imu_msg.angular_velocity.z = std::stod(tokens[7]);
            
            // Temperature
            temp_msg.temperature = std::stod(tokens[8]);
            
            // Publish
            imu_pub_->publish(imu_msg);
            temp_pub_->publish(temp_msg);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing IMU data: %s", e.what());
        }
    }

    std::string serial_port_;
    int baud_rate_;
    int serial_fd_;
    std::string buffer_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialIMULifecycleNode>();

    // Zelf configureren/activeren zodat de lifecycle-node direct publiceert.
    auto configured_state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    if (configured_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_ERROR(node->get_logger(), "Lifecycle configure failed, cannot continue (state id %u)", configured_state.id());
        rclcpp::shutdown();
        return 1;
    }
    auto activated_state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    if (activated_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_ERROR(node->get_logger(), "Lifecycle activate failed, cannot continue (state id %u)", activated_state.id());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
