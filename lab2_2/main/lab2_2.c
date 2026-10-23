/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
// #include <stdio.h>
// #include "sdkconfig.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/i2c_master.h"
#include "lab2_2.h"

static const char *TAG = "example";

// #define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
// #define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
// #define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
// #define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
// #define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_TIMEOUT_MS       1000

// #define MPU9250_SENSOR_ADDR         0x68        /*!< Address of the MPU9250 sensor */
// #define MPU9250_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
// #define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
// #define MPU9250_RESET_BIT           7

// #define SHTC3_ADDR       0x70        /*!< Address of the SHTC3 sensor */
// #define SHTC3_WAKEUP     0x3517      /*!< Wake-up command for the SHTC3 sensor */
// #define SHTC3_SLEEP      0xB098      /*!< Sleep command for the SHTC3 sensor */
// #define SHTC3_TEMP_CMD   0x7866      /*!< Temperature measurement command for the SHTC3 sensor */
// #define SHTC3_HUM_CMD    0x58E0      /*!< Humidity measurement command for the SHTC3 sensor */



/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
// static esp_err_t mpu9250_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
// }

static esp_err_t shtc3_read(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t len)
{
    return i2c_master_receive(dev_handle, data, len, I2C_MASTER_TIMEOUT_MS);
}

static uint8_t shtc3_crc(uint8_t *data)
{
    uint8_t crc = 0xFF;

    for (int i = 0; i < 2; i++) {
        crc ^= data[i];

        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static esp_err_t read_temperature(
    i2c_master_dev_handle_t dev_handle,
    float *temperature)
{
    esp_err_t err = shtc3_write(
        dev_handle, SHTC3_TEMP_CMD);

    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t data[3];

    err = shtc3_read(dev_handle, data, 3);

    if (err != ESP_OK) {
        return err;
    }

    if (shtc3_crc(data) != data[2]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw =
        ((uint16_t)data[0] << 8) | data[1];

    *temperature =
        -45.0f + 175.0f * raw / 65536.0f;

    return ESP_OK;
}

static esp_err_t read_humidity(
    i2c_master_dev_handle_t dev_handle,
    float *humidity)
{
    esp_err_t err = shtc3_write(
        dev_handle, SHTC3_HUM_CMD);

    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t data[3];

    err = shtc3_read(dev_handle, data, 3);

    if (err != ESP_OK) {
        return err;
    }

    if (shtc3_crc(data) != data[2]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw =
        ((uint16_t)data[0] << 8) | data[1];

    *humidity =
        100.0f * raw / 65536.0f;

    return ESP_OK;
}
/**
 * @brief Write a byte to a MPU9250 sensor register
 */
// static esp_err_t mpu9250_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
// {
//     uint8_t write_buf[2] = {reg_addr, data};
//     return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
// }

static esp_err_t shtc3_write(i2c_master_dev_handle_t dev_handle, uint16_t command)
{
    uint8_t write_buf[2] = {(uint8_t)(command >> 8), (uint8_t)(command & 0xFF)};

    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    // i2c_device_config_t dev_config = {
    //     .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    //     .device_address = MPU9250_SENSOR_ADDR,
    //     .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    // };

    esp_err_t err =i2c_master_probe(*bus_handle, SHTC3_ADDR, 1000);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SHTC3 detected at 0x70");
    } else {
        ESP_LOGE(TAG, "SHTC3 not detected: %s",
                esp_err_to_name(err));
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHTC3_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

// void app_main(void)
// {
//     uint8_t data[2];
//     i2c_master_bus_handle_t bus_handle;
//     i2c_master_dev_handle_t dev_handle;
//     i2c_master_init(&bus_handle, &dev_handle);
//     ESP_LOGI(TAG, "I2C initialized successfully");

//     /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
//     ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_WHO_AM_I_REG_ADDR, data, 1));
//     ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

//     /* Demonstrate writing by resetting the MPU9250 */
//     ESP_ERROR_CHECK(mpu9250_register_write_byte(dev_handle, MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

//     ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
//     ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
//     ESP_LOGI(TAG, "I2C de-initialized successfully");
// }



void app_main(void)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    i2c_master_init(&bus_handle, &dev_handle);

    ESP_LOGI(TAG, "SHTC3 initialized");

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        float temperature = 0;
        float humidity = 0;

        //Wake up
        esp_err_t err = shtc3_write(dev_handle, SHTC3_WAKEUP);
        ESP_LOGI(TAG, "Wakeup: %s", esp_err_to_name(err));

        if (err == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(2));

            //Read temperature
            err = read_temperature(dev_handle, &temperature);
            ESP_LOGI(TAG, "Temperature: %s", esp_err_to_name(err));

            if (err == ESP_OK) {
                //Read humidity
                err = read_humidity(dev_handle, &humidity);
                ESP_LOGI(TAG, "Humidity: %s", esp_err_to_name(err));
            }
        }

        //Sleep
        esp_err_t sleep_err = shtc3_write(dev_handle, SHTC3_SLEEP);
        ESP_LOGI(TAG, "Sleep: %s", esp_err_to_name(sleep_err));

        if (err == ESP_OK) {
            err = sleep_err;
        }

        if (err == ESP_OK) {
            float fahrenheit = temperature * 9.0f / 5.0f + 32.0f;

            printf("Temperature is %.0fC (or %.0fF) "
                "with a %.0f%% humidity\n",
                temperature, fahrenheit, humidity);
        } else {
            ESP_LOGE(TAG, "SHTC3 error: %s", esp_err_to_name(err));
        }

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2000));
    }
}
