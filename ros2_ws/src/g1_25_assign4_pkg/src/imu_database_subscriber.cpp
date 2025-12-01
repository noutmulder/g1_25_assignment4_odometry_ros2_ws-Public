#include <chrono>
#include <memory>
#include <string>
#include <sqlite3.h>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"

class IMUDatabaseSubscriber : public rclcpp::Node
{
public:
    IMUDatabaseSubscriber() : Node("imu_database_node"), db_(nullptr)
    {
        // Declare parameters
        this->declare_parameter<std::string>("database_path", "imu_data.db");
        
        std::string db_path = this->get_parameter("database_path").as_string();
        
        // Open database
        int rc = sqlite3_open(db_path.c_str(), &db_);
        if (rc) {
            RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
            return;
        }
        
        // Create table
        const char* sql = 
            "CREATE TABLE IF NOT EXISTS imu_data ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT,"
            "timestamp_sec INTEGER,"
            "timestamp_nanosec INTEGER,"
            "accel_x REAL,"
            "accel_y REAL,"
            "accel_z REAL,"
            "gyro_x REAL,"
            "gyro_y REAL,"
            "gyro_z REAL,"
            "temperature REAL"
            ");";
        
        char* err_msg = nullptr;
        rc = sqlite3_exec(db_, sql, nullptr, nullptr, &err_msg);
        if (rc != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "SQL error: %s", err_msg);
            sqlite3_free(err_msg);
            sqlite3_close(db_);
            db_ = nullptr;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Database opened: %s", db_path.c_str());
        
        // Create subscriptions
        auto sensor_qos = rclcpp::SensorDataQoS();
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", sensor_qos,
            std::bind(&IMUDatabaseSubscriber::imu_callback, this, std::placeholders::_1));
        
        temp_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
            "/imu/temperature", sensor_qos,
            std::bind(&IMUDatabaseSubscriber::temp_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "IMUDatabaseSubscriber initialized");
    }

    ~IMUDatabaseSubscriber()
    {
        if (db_) {
            sqlite3_close(db_);
        }
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Store latest IMU message and attempt DB write if temperature already available
        std::lock_guard<std::mutex> lock(db_mutex_);
        last_imu_data_ = msg;

        if (last_temperature_data_) {
            store_to_database_unlocked();
        }
    }

    void temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg)
    {
        // Store latest temperature message and attempt DB write if IMU already available
        std::lock_guard<std::mutex> lock(db_mutex_);
        last_temperature_data_ = msg;

        if (last_imu_data_) {
            store_to_database_unlocked();
        }
    }

    // Caller should hold db_mutex_
    void store_to_database_unlocked()
    {
        if (!db_ || !last_imu_data_ || !last_temperature_data_) return;

        const char* sql = 
            "INSERT INTO imu_data (timestamp_sec, timestamp_nanosec, accel_x, accel_y, accel_z, "
            "gyro_x, gyro_y, gyro_z, temperature) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);";

        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
        if (rc != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to prepare statement: %s", sqlite3_errmsg(db_));
            return;
        }

        // Bind values
        sqlite3_bind_int64(stmt, 1, last_imu_data_->header.stamp.sec);
        sqlite3_bind_int64(stmt, 2, last_imu_data_->header.stamp.nanosec);
        sqlite3_bind_double(stmt, 3, last_imu_data_->linear_acceleration.x);
        sqlite3_bind_double(stmt, 4, last_imu_data_->linear_acceleration.y);
        sqlite3_bind_double(stmt, 5, last_imu_data_->linear_acceleration.z);
        sqlite3_bind_double(stmt, 6, last_imu_data_->angular_velocity.x);
        sqlite3_bind_double(stmt, 7, last_imu_data_->angular_velocity.y);
        sqlite3_bind_double(stmt, 8, last_imu_data_->angular_velocity.z);

        double temp = last_temperature_data_ ? last_temperature_data_->temperature : 0.0;
        sqlite3_bind_double(stmt, 9, temp);

        // Execute
        rc = sqlite3_step(stmt);
        if (rc != SQLITE_DONE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to insert data: %s", sqlite3_errmsg(db_));
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Data stored to database");
            // Clear stored messages to avoid duplicate inserts
            last_imu_data_.reset();
            last_temperature_data_.reset();
        }

        sqlite3_finalize(stmt);
    }

    // Thread-safe wrapper
    void store_to_database()
    {
        std::lock_guard<std::mutex> lock(db_mutex_);
        store_to_database_unlocked();
    }

    sqlite3* db_;
    sensor_msgs::msg::Imu::SharedPtr last_imu_data_;
    sensor_msgs::msg::Temperature::SharedPtr last_temperature_data_;
    std::mutex db_mutex_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUDatabaseSubscriber>());
    rclcpp::shutdown();
    return 0;
}
