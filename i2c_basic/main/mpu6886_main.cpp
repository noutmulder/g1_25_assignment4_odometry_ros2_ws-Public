/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/* MPU6886 IMU Sensor Driver
   
   This example demonstrates how to initialize and read from the MPU6886
   inertial measurement unit over I2C, including:
   - Accelerometer data in m/s^2
   - Temperature data in °C with calibration support
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mpu6886_main.h"
#include "wifi_mqtt_config.h"

static const char *TAG = "mpu6886";

/* Runtime temperature conversion parameters (initialized from datasheet typicals) */
static float g_mpu6886_temp_sens = 326.87f;    /* LSB per degC */
static float g_mpu6886_temp_offset = 25.0f;    /* degC at raw=0 (datasheet typical) */

/* ========== Low-Level I2C Functions ========== */

esp_err_t mpu6886_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, 
                                       I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6886_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), 
                               I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/* ========== I2C Initialization ========== */

void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_MASTER_NUM;
    bus_config.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    bus_config.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = MPU6886_SENSOR_ADDR;
    dev_config.scl_speed_hz = I2C_MASTER_FREQ_HZ;
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

/* ========== MPU6886 Initialization ========== */

esp_err_t mpu6886_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    esp_err_t ret;
    uint8_t who = 0;

    /* Try reading WHO_AM_I at the current device handle/address */
    ret = mpu6886_register_read(*dev_handle, MPU6886_WHO_AM_I_REG_ADDR, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WHO_AM_I read failed at address 0x%02X, trying 0x69...", MPU6886_SENSOR_ADDR);

        /* Remove current device and try alternate address 0x69 */
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(*dev_handle));
        
        i2c_device_config_t alt_dev = {};
        alt_dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        alt_dev.device_address = 0x69;
        alt_dev.scl_speed_hz = I2C_MASTER_FREQ_HZ;
        
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &alt_dev, dev_handle));

        ret = mpu6886_register_read(*dev_handle, MPU6886_WHO_AM_I_REG_ADDR, &who, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "WHO_AM_I read failed at both addresses");
            return ret;
        }
    }

    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who);

    /* Wake the sensor by clearing sleep bit in PWR_MGMT_1 (write 0x00) */
    ret = mpu6886_register_write_byte(*dev_handle, MPU6886_PWR_MGMT_1_REG_ADDR, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write PWR_MGMT_1 to wake sensor");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* Optional: re-read WHO_AM_I to ensure sensor responds after wake */
    who = 0;
    ret = mpu6886_register_read(*dev_handle, MPU6886_WHO_AM_I_REG_ADDR, &who, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WHO_AM_I after wake = 0x%02X", who);
    } else {
        ESP_LOGW(TAG, "WHO_AM_I read failed after wake");
    }

    return ESP_OK;
}

/* ========== Accelerometer Functions ========== */

float mpu6886_get_accel_sensitivity(i2c_master_dev_handle_t dev_handle)
{
    uint8_t reg = 0;
    if (mpu6886_register_read(dev_handle, MPU6886_ACCEL_CONFIG_REG, &reg, 1) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read ACCEL_CONFIG, assuming +/-2g");
        return 16384.0f;
    }

    uint8_t afs = (reg >> 3) & 0x03; /* AFS_SEL bits are 4:3 */
    switch (afs) {
        case 0: return 16384.0f; /* +/-2g */
        case 1: return 8192.0f;  /* +/-4g */
        case 2: return 4096.0f;  /* +/-8g */
        case 3: return 2048.0f;  /* +/-16g */
        default: return 16384.0f;
    }
}

/* ========== Temperature Functions ========== */

esp_err_t mpu6886_read_temperature_avg(i2c_master_dev_handle_t dev_handle, int samples, 
                                       int32_t *raw_avg, float *temp_c)
{
    if (samples <= 0) samples = 1;
    int32_t sum = 0;
    
    for (int i = 0; i < samples; i++) {
        uint8_t buf[2];
        esp_err_t r = mpu6886_register_read(dev_handle, MPU6886_TEMP_OUT_H, buf, 2);
        if (r != ESP_OK) return r;
        
        int16_t val = (int16_t)((buf[0] << 8) | buf[1]);
        sum += val;

        if (samples > 1) {
            vTaskDelay(1);
        }
    }
    
    int32_t avg = sum / samples;
    if (raw_avg) *raw_avg = avg;
    if (temp_c) *temp_c = ((float)avg) / g_mpu6886_temp_sens + g_mpu6886_temp_offset;
    
    return ESP_OK;
}

esp_err_t mpu6886_calibrate_temp_offset(i2c_master_dev_handle_t dev_handle, int samples, float ref_temp)
{
    int32_t raw_avg = 0;
    float temp_now = 0.0f;
    esp_err_t r = mpu6886_read_temperature_avg(dev_handle, samples, &raw_avg, &temp_now);
    if (r != ESP_OK) return r;

    float suggested_offset = ref_temp - ((float)raw_avg / g_mpu6886_temp_sens);
    ESP_LOGI(TAG, "Calibration: raw_avg=%d, current_conv=%.2f C, ref=%.2f C", 
             raw_avg, temp_now, ref_temp);
    ESP_LOGI(TAG, "Suggested offset = %.4f (set g_mpu6886_temp_offset to this for permanent use)", 
             suggested_offset);

    /* Apply at runtime */
    g_mpu6886_temp_offset = suggested_offset;
    ESP_LOGI(TAG, "Applied runtime offset -> new temp = %.2f C", 
             ((float)raw_avg / g_mpu6886_temp_sens) + g_mpu6886_temp_offset);
    
    return ESP_OK;
}

/* ========== Main Application ========== */

extern "C" void app_main(void)
{
    /* Initialize NVS for WiFi */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "MPU6886 IMU Sensor with WiFi/MQTT");
    ESP_LOGI(TAG, "========================================");

    /* Initialize WiFi */
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();
    
    if (!wifi_is_connected()) {
        ESP_LOGW(TAG, "WiFi not connected! Falling back to serial-only mode.");
    } else {
        ESP_LOGI(TAG, "WiFi connected! Starting MQTT...");
        mqtt_app_start();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Give MQTT time to connect
    }

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    
    /* Initialize I2C */
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Initialize MPU6886 sensor */
    if (mpu6886_init(bus_handle, &dev_handle) != ESP_OK) {
        ESP_LOGE(TAG, "MPU init failed - check wiring and device address");
        return;
    }
    ESP_LOGI(TAG, "MPU6886 initialized successfully");

    /* Get accelerometer sensitivity */
    float accel_sens = mpu6886_get_accel_sensitivity(dev_handle);
    ESP_LOGI(TAG, "Accel sensitivity = %.1f LSB/g", accel_sens);

    ESP_LOGI(TAG, "Skipping interactive temperature calibration for fast startup");

    printf("\n=== Starting sensor readout ===\n\n");

    /* Main sensor reading loop */
    uint8_t imu_buffer[14];
    const float G_TO_MS2 = 9.80665f;
    
    /* For timestamp */
    uint32_t start_time = esp_log_timestamp();
    
    /* Buffer for CSV data */
    char csv_buffer[256];
    
    while (1) {
        uint32_t timestamp_ms = esp_log_timestamp() - start_time;

        if (mpu6886_register_read(dev_handle, MPU6886_ACCEL_XOUT_H, imu_buffer, sizeof(imu_buffer)) != ESP_OK) {
            ESP_LOGW(TAG, "IMU read failed");
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        int16_t accel_x = (int16_t)(imu_buffer[0] << 8 | imu_buffer[1]);
        int16_t accel_y = (int16_t)(imu_buffer[2] << 8 | imu_buffer[3]);
        int16_t accel_z = (int16_t)(imu_buffer[4] << 8 | imu_buffer[5]);

        int16_t temp_raw = (int16_t)(imu_buffer[6] << 8 | imu_buffer[7]);

        int16_t gyro_x = (int16_t)(imu_buffer[8] << 8 | imu_buffer[9]);
        int16_t gyro_y = (int16_t)(imu_buffer[10] << 8 | imu_buffer[11]);
        int16_t gyro_z = (int16_t)(imu_buffer[12] << 8 | imu_buffer[13]);
        float temperature = ((float)temp_raw / g_mpu6886_temp_sens) + g_mpu6886_temp_offset;
        
        /* Convert accelerometer to m/s^2 */
        float ax_g = (float)accel_x / accel_sens;
        float ay_g = (float)accel_y / accel_sens;
        float az_g = (float)accel_z / accel_sens;
        
        float ax_ms2 = ax_g * G_TO_MS2;
        float ay_ms2 = ay_g * G_TO_MS2;
        float az_ms2 = az_g * G_TO_MS2;
        
        /* Convert gyroscope to rad/s (assuming FS_SEL=0, 250 deg/s -> 131 LSB/deg/s) */
        const float GYRO_SENS = 131.0f;  // LSB per deg/s for FS_SEL=0
        const float DEG_TO_RAD = 0.017453292f;
        float gx_rads = ((float)gyro_x / GYRO_SENS) * DEG_TO_RAD;
        float gy_rads = ((float)gyro_y / GYRO_SENS) * DEG_TO_RAD;
        float gz_rads = ((float)gyro_z / GYRO_SENS) * DEG_TO_RAD;
        
        /* Create CSV format: IMU,timestamp,ax,ay,az,gx,gy,gz,temp */
        snprintf(csv_buffer, sizeof(csv_buffer), 
                "IMU,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f",
                timestamp_ms, ax_ms2, ay_ms2, az_ms2, gx_rads, gy_rads, gz_rads, temperature);
        
        /* Output to serial (for wired ROS2 bridge) */
        printf("%s\n", csv_buffer);
        
        /* Publish to MQTT (for wireless ROS2 bridge) */
        if (mqtt_is_connected()) {
            mqtt_publish_imu_data(csv_buffer);
        }
        
        ESP_LOGD(TAG, "Accel: X=%.3f Y=%.3f Z=%.3f m/s² | Gyro: X=%.3f Y=%.3f Z=%.3f rad/s | Temp: %.2f°C", 
                 ax_ms2, ay_ms2, az_ms2, gx_rads, gy_rads, gz_rads, temperature);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
