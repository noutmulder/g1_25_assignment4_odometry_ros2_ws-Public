/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef MPU6886_MAIN_H
#define MPU6886_MAIN_H

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C Configuration */
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY
#define I2C_MASTER_TIMEOUT_MS       1000

/* MPU6886 Register Addresses */
#define MPU6886_SENSOR_ADDR         0x68
#define MPU6886_WHO_AM_I_REG_ADDR   0x75
#define MPU6886_PWR_MGMT_1_REG_ADDR 0x6B
#define MPU6886_ACCEL_CONFIG_REG    0x1C
#define MPU6886_ACCEL_XOUT_H        0x3B
#define MPU6886_TEMP_OUT_H          0x41
#define MPU6886_RESET_BIT           7

/* Temperature Calibration */
#define MPU6886_TEMP_SAMPLES        8

/**
 * @brief Read a sequence of bytes from MPU6886 sensor registers
 */
esp_err_t mpu6886_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Write a byte to a MPU6886 sensor register
 */
esp_err_t mpu6886_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);

/**
 * @brief Initialize I2C master and add MPU6886 device
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Initialize MPU6886 sensor: check WHO_AM_I and wake from sleep
 */
esp_err_t mpu6886_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Read accelerometer full-scale setting and return sensitivity in LSB/g
 */
float mpu6886_get_accel_sensitivity(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Read temperature multiple times and return averaged raw and converted °C
 */
esp_err_t mpu6886_read_temperature_avg(i2c_master_dev_handle_t dev_handle, int samples, int32_t *raw_avg, float *temp_c);

/**
 * @brief Calibrate temperature offset using a reference thermometer
 */
esp_err_t mpu6886_calibrate_temp_offset(i2c_master_dev_handle_t dev_handle, int samples, float ref_temp);

#ifdef __cplusplus
}
#endif

#endif /* MPU6886_MAIN_H */
