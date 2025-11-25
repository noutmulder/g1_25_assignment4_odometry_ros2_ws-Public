#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <mosquitto.h>

using namespace std::chrono_literals;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MQTTIMUBridge : public rclcpp_lifecycle::LifecycleNode
{
public:
    MQTTIMUBridge() : LifecycleNode("mqtt_imu_bridge"), mosq_(nullptr)
    {
        // Declare parameters
        this->declare_parameter<std::string>("mqtt_broker", "localhost");
        this->declare_parameter<int>("mqtt_port", 1883);
        this->declare_parameter<std::string>("mqtt_topic", "esp32/imu/data");
        
        RCLCPP_INFO(this->get_logger(), "MQTTIMUBridge constructed");
    }

    ~MQTTIMUBridge()
    {
        if (mosq_) {
            mosquitto_loop_stop(mosq_, true);
            mosquitto_destroy(mosq_);
        }
        mosquitto_lib_cleanup();
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Configuring MQTTIMUBridge");
        
        // Get parameters
        mqtt_broker_ = this->get_parameter("mqtt_broker").as_string();
        mqtt_port_ = this->get_parameter("mqtt_port").as_int();
        mqtt_topic_ = this->get_parameter("mqtt_topic").as_string();
        
        // Create publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temperature", 10);
        
        RCLCPP_INFO(this->get_logger(), "Configured with broker: %s:%d, topic: %s", 
                    mqtt_broker_.c_str(), mqtt_port_, mqtt_topic_.c_str());
        
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Activating MQTTIMUBridge");
        
        // Initialize mosquitto library
        mosquitto_lib_init();
        
        // Create mosquitto instance
        mosq_ = mosquitto_new(nullptr, true, this);
        if (!mosq_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create mosquitto instance");
            return LifecycleCallbackReturn::ERROR;
        }
        
        // Set callbacks
        mosquitto_connect_callback_set(mosq_, on_connect_wrapper);
        mosquitto_message_callback_set(mosq_, on_message_wrapper);
        
        // Connect to broker
        int rc = mosquitto_connect(mosq_, mqtt_broker_.c_str(), mqtt_port_, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", mosquitto_strerror(rc));
            mosquitto_destroy(mosq_);
            mosq_ = nullptr;
            return LifecycleCallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker %s:%d", mqtt_broker_.c_str(), mqtt_port_);
        
        // Start mosquitto loop in separate thread
        mosquitto_loop_start(mosq_);
        
        // Activate publishers
        imu_pub_->on_activate();
        temp_pub_->on_activate();
        
        RCLCPP_INFO(this->get_logger(), "MQTT bridge activated");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating MQTTIMUBridge");
        
        // Deactivate publishers
        imu_pub_->on_deactivate();
        temp_pub_->on_deactivate();
        
        // Disconnect and cleanup MQTT
        if (mosq_) {
            mosquitto_loop_stop(mosq_, true);
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
            mosq_ = nullptr;
        }
        
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up MQTTIMUBridge");
        
        imu_pub_.reset();
        temp_pub_.reset();
        
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down MQTTIMUBridge");
        
        if (mosq_) {
            mosquitto_loop_stop(mosq_, true);
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
            mosq_ = nullptr;
        }
        
        return LifecycleCallbackReturn::SUCCESS;
    }

private:
    static void on_connect_wrapper(struct mosquitto *mosq, void *obj, int rc)
    {
        auto *node = static_cast<MQTTIMUBridge*>(obj);
        node->on_connect(mosq, rc);
    }
    
    static void on_message_wrapper(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
    {
        auto *node = static_cast<MQTTIMUBridge*>(obj);
        node->on_message(mosq, msg);
    }
    
    void on_connect(struct mosquitto *mosq, int rc)
    {
        if (rc == 0) {
            RCLCPP_INFO(this->get_logger(), "MQTT connected successfully");
            // Subscribe to topic
            mosquitto_subscribe(mosq, nullptr, mqtt_topic_.c_str(), 0);
            RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", mqtt_topic_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "MQTT connection failed with code %d", rc);
        }
    }
    
    void on_message(struct mosquitto *mosq, const struct mosquitto_message *msg)
    {
        (void)mosq; // Unused
        
        std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);
        RCLCPP_DEBUG(this->get_logger(), "Received MQTT message: %s", payload.c_str());
        
        // Parse CSV: IMU,timestamp,ax,ay,az,gx,gy,gz,temp
        std::vector<std::string> tokens;
        std::stringstream ss(payload);
        std::string token;
        
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() != 9 || tokens[0] != "IMU") {
            RCLCPP_WARN(this->get_logger(), "Invalid CSV format (expected 9 fields starting with 'IMU', got %zu fields): %s", 
                        tokens.size(), payload.c_str());
            return;
        }
        
        try {
            // Create IMU message
            auto imu_msg = sensor_msgs::msg::Imu();

            // Parse and apply sensor-provided timestamp (milliseconds) when possible
            uint64_t timestamp_ms = 0;
            try {
                timestamp_ms = std::stoull(tokens[1]);
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse timestamp_ms '%s', using now()", tokens[1].c_str());
                auto now = this->now();
                imu_msg.header.stamp = now;
                imu_msg.header.frame_id = "imu_link";
            }

            if (timestamp_ms != 0) {
                uint64_t sec = timestamp_ms / 1000u;
                uint64_t nsec = (timestamp_ms % 1000u) * 1000000u;
                imu_msg.header.stamp.sec = static_cast<int32_t>(sec);
                imu_msg.header.stamp.nanosec = static_cast<uint32_t>(nsec);
                imu_msg.header.frame_id = "imu_link";
            }
            
            // Parse values
            imu_msg.linear_acceleration.x = std::stod(tokens[2]);
            imu_msg.linear_acceleration.y = std::stod(tokens[3]);
            imu_msg.linear_acceleration.z = std::stod(tokens[4]);
            
            imu_msg.angular_velocity.x = std::stod(tokens[5]);
            imu_msg.angular_velocity.y = std::stod(tokens[6]);
            imu_msg.angular_velocity.z = std::stod(tokens[7]);
            
            // Publish IMU data
            imu_pub_->publish(imu_msg);
            
            // Create temperature message
            auto temp_msg = sensor_msgs::msg::Temperature();
            temp_msg.header.stamp = imu_msg.header.stamp;
            temp_msg.header.frame_id = "imu_link";
            temp_msg.temperature = std::stod(tokens[8]);
            
            // Publish temperature
            temp_pub_->publish(temp_msg);
            
            RCLCPP_DEBUG(this->get_logger(), "Published IMU data: ax=%.3f, ay=%.3f, az=%.3f", 
                        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
            
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse CSV: %s", e.what());
        }
    }
    
    std::string mqtt_broker_;
    int mqtt_port_;
    std::string mqtt_topic_;
    
    struct mosquitto *mosq_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MQTTIMUBridge>();

    // Zelf configureren/activeren zodat de lifecycle-node meteen data kan verwerken.
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
