/*
 * I2C Scanner for MPU6050 Detection
 * Board: ESP32-S3-N8R8 (YD-ESP32-23)
 * Wiring:
 *   SDA → GPIO 8
 *   SCL → GPIO 9
 * Expected: MPU6050 detected at 0x68
 * Author: Ahamed Raafiq
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define I2C_SDA_PIN         GPIO_NUM_8
#define I2C_SCL_PIN         GPIO_NUM_9
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_PORT            I2C_NUM_0

static const char *TAG = "I2C_SCANNER";

void i2c_scan(i2c_master_bus_handle_t bus_handle)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "===== I2C Bus Scan Started =====");
    ESP_LOGI(TAG, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");

    int device_count = 0;

    for (uint8_t row = 0; row < 8; row++) {
        printf("%02x: ", row * 16);
        for (uint8_t col = 0; col < 16; col++) {
            uint8_t address = row * 16 + col;
            esp_err_t ret = i2c_master_probe(bus_handle, address, 100);
            if (ret == ESP_OK) {
                printf("%02x ", address);
                device_count++;
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }

    ESP_LOGI(TAG, "===== Scan Complete =====");
    ESP_LOGI(TAG, "Total devices found: %d", device_count);

    if (device_count == 0) {
        ESP_LOGE(TAG, "No I2C devices found!");
        ESP_LOGE(TAG, "Check wiring: SDA→GPIO8, SCL→GPIO9, VCC→3V3, GND→GND");
    } else {
        ESP_LOGI(TAG, "MPU6050 expected at address 0x68");
    }
    ESP_LOGI(TAG, "");
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting I2C Scanner on ESP32-S3");

    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    ESP_LOGI(TAG, "I2C initialized on SDA=%d, SCL=%d",
             I2C_SDA_PIN, I2C_SCL_PIN);

    // Continuously scan every 3 seconds
    while (1) {
        i2c_scan(bus_handle);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
