/*
 * ═══════════════════════════════════════════════════════════════════
 * IMU PUBLISHER FIRMWARE
 * ═══════════════════════════════════════════════════════════════════
 * Board:    ESP32-S3-N8R8 (YD-ESP32-23)
 * Sensor:   MPU6050 / MPU6500 / MPU9250 (I2C address 0x68)
 * Function: Read IMU, send raw data over UART as CSV with timestamp
 * Output:   "timestamp_ms,ax,ay,az,gx,gy,gz,temp\n"
 * Author:   Ahamed Raafiq
 * ═══════════════════════════════════════════════════════════════════
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

// ═══════════════════════════════════════════════════════════════════
// TUNABLE PARAMETERS
// ═══════════════════════════════════════════════════════════════════

// --- I2C Pins ---
#define I2C_SDA_PIN             GPIO_NUM_8
#define I2C_SCL_PIN             GPIO_NUM_9

// --- I2C Speed ---
// Option 1: 100000   → Standard mode
// Option 2: 400000   → Fast mode (recommended)
// Option 3: 1000000  → Fast mode plus
// Selected: Option 2
#define I2C_SPEED_HZ            400000

// --- Sensor I2C Address ---
// 0x68 → AD0 grounded (our wiring)
// 0x69 → AD0 to VCC
#define SENSOR_ADDR             0x68

// --- Sensor Type Detection ---
// The WHO_AM_I register returns different values for different chips:
// Option 1: 0x68 → Original MPU6050 (InvenSense)
// Option 2: 0x70 → MPU6500 (common clone/upgrade)
// Option 3: 0x71 → MPU9250 (9-axis version)
// Option 4: 0x73 → MPU9255
// Option 5: 0x00 → Accept ANY device (unsafe but useful for debug)
// Selected: We accept all known valid IDs (see code below)
#define ACCEPT_MPU6050          1  // 0x68
#define ACCEPT_MPU6500          1  // 0x70
#define ACCEPT_MPU9250          1  // 0x71
#define ACCEPT_MPU9255          1  // 0x73
#define ACCEPT_UNKNOWN          0  // Accept any device (debug only)

// --- UART Baud Rate ---
// Option 1: 115200   → Standard, most stable
// Option 2: 230400   → 2x speed
// Option 3: 460800   → 4x speed
// Option 4: 921600   → 8x speed (recommended for 100Hz IMU)
// Selected: Option 4
#define UART_BAUD_RATE          921600
#define UART_PORT               UART_NUM_0

// --- Sample Rate (Hz) ---
// Option 1: 50   → Low load, sluggish
// Option 2: 100  → Balanced (recommended)
// Option 3: 200  → Fast response
// Option 4: 500  → Very fast, may saturate serial
// Selected: Option 2
#define SAMPLE_RATE_HZ          100

// --- Accelerometer Range ---
// Option 1: 0x00 → ±2g   (best sensitivity)
// Option 2: 0x08 → ±4g   (balanced, recommended)
// Option 3: 0x10 → ±8g
// Option 4: 0x18 → ±16g  (no saturation)
// Selected: Option 2
#define ACCEL_RANGE_CONFIG      0x08
#define ACCEL_SCALE_FACTOR      8192.0f  // LSB/g for ±4g

// --- Gyroscope Range ---
// Option 1: 0x00 → ±250°/s
// Option 2: 0x08 → ±500°/s   (balanced, recommended)
// Option 3: 0x10 → ±1000°/s
// Option 4: 0x18 → ±2000°/s
// Selected: Option 2
#define GYRO_RANGE_CONFIG       0x08
#define GYRO_SCALE_FACTOR       65.5f    // LSB/(°/s) for ±500°/s

// --- Digital Low-Pass Filter (DLPF) ---
// Option 1: 0x00 → 260Hz BW (most noise)
// Option 2: 0x03 → 44Hz BW  (balanced, recommended)
// Option 3: 0x05 → 10Hz BW  (smooth but slow)
// Option 4: 0x06 → 5Hz BW   (very smooth)
// Selected: Option 2
#define DLPF_CONFIG             0x03

// --- Status Log ---
// Option 1: 0 → Silent (no periodic log)
// Option 2: 1 → Log every 1 second
// Selected: Option 2
#define ENABLE_STATUS_LOG       1

// --- MPU Register Addresses (do NOT change) ---
#define REG_PWR_MGMT_1          0x6B
#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_ACCEL_XOUT_H        0x3B
#define REG_WHO_AM_I            0x75

// --- Physical Constants (do NOT change) ---
#define GRAVITY_MS2             9.80665f
#define DEG_TO_RAD              0.01745329252f

// ═══════════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════

static const char *TAG = "IMU_PUB";
static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t sensor_handle;

// ═══════════════════════════════════════════════════════════════════
// SENSOR TYPE HELPER
// ═══════════════════════════════════════════════════════════════════

static const char* get_sensor_name(uint8_t who_am_i)
{
    switch (who_am_i) {
        case 0x68: return "MPU6050";
        case 0x70: return "MPU6500";
        case 0x71: return "MPU9250";
        case 0x73: return "MPU9255";
        case 0x98: return "ICM-20689";
        default:   return "Unknown";
    }
}

static bool is_sensor_supported(uint8_t who_am_i)
{
#if ACCEPT_UNKNOWN
    return true;
#endif
    if (ACCEPT_MPU6050 && who_am_i == 0x68) return true;
    if (ACCEPT_MPU6500 && who_am_i == 0x70) return true;
    if (ACCEPT_MPU9250 && who_am_i == 0x71) return true;
    if (ACCEPT_MPU9255 && who_am_i == 0x73) return true;
    return false;
}

// ═══════════════════════════════════════════════════════════════════
// I2C HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

static esp_err_t sensor_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(sensor_handle, buf, 2, 100);
}

static esp_err_t sensor_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(sensor_handle, &reg, 1, data, len, 100);
}

// ═══════════════════════════════════════════════════════════════════
// SENSOR INITIALIZATION
// ═══════════════════════════════════════════════════════════════════

static esp_err_t sensor_init(void)
{
    esp_err_t ret;
    uint8_t who_am_i = 0;

    // 1. Read device ID
    ret = sensor_read_bytes(REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WHO_AM_I = 0x%02x → %s",
             who_am_i, get_sensor_name(who_am_i));

    if (!is_sensor_supported(who_am_i)) {
        ESP_LOGE(TAG, "Unsupported sensor detected!");
        ESP_LOGE(TAG, "Update ACCEPT_* flags in code if you want to force use.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor supported ✓");

    // 2. Wake up (clear sleep bit + set clock source to internal)
    ret = sensor_write_byte(REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3. Set sample rate divider
    uint8_t smplrt_div = (1000 / SAMPLE_RATE_HZ) - 1;
    ret = sensor_write_byte(REG_SMPLRT_DIV, smplrt_div);
    if (ret != ESP_OK) return ret;

    // 4. Configure DLPF
    ret = sensor_write_byte(REG_CONFIG, DLPF_CONFIG);
    if (ret != ESP_OK) return ret;

    // 5. Gyro range
    ret = sensor_write_byte(REG_GYRO_CONFIG, GYRO_RANGE_CONFIG);
    if (ret != ESP_OK) return ret;

    // 6. Accel range
    ret = sensor_write_byte(REG_ACCEL_CONFIG, ACCEL_RANGE_CONFIG);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Sensor configured:");
    ESP_LOGI(TAG, "  Sample Rate: %d Hz", SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "  Accel Range: ±4g");
    ESP_LOGI(TAG, "  Gyro Range:  ±500°/s");
    ESP_LOGI(TAG, "  DLPF BW:     44Hz");

    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════
// READ SENSOR DATA
// ═══════════════════════════════════════════════════════════════════

static esp_err_t sensor_read_all(float *ax, float *ay, float *az,
                                   float *gx, float *gy, float *gz,
                                   float *temp)
{
    uint8_t buf[14];
    esp_err_t ret = sensor_read_bytes(REG_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) return ret;

    int16_t raw_ax = (buf[0]  << 8) | buf[1];
    int16_t raw_ay = (buf[2]  << 8) | buf[3];
    int16_t raw_az = (buf[4]  << 8) | buf[5];
    int16_t raw_t  = (buf[6]  << 8) | buf[7];
    int16_t raw_gx = (buf[8]  << 8) | buf[9];
    int16_t raw_gy = (buf[10] << 8) | buf[11];
    int16_t raw_gz = (buf[12] << 8) | buf[13];

    *ax = (raw_ax / ACCEL_SCALE_FACTOR) * GRAVITY_MS2;
    *ay = (raw_ay / ACCEL_SCALE_FACTOR) * GRAVITY_MS2;
    *az = (raw_az / ACCEL_SCALE_FACTOR) * GRAVITY_MS2;

    *gx = (raw_gx / GYRO_SCALE_FACTOR) * DEG_TO_RAD;
    *gy = (raw_gy / GYRO_SCALE_FACTOR) * DEG_TO_RAD;
    *gz = (raw_gz / GYRO_SCALE_FACTOR) * DEG_TO_RAD;

    // Note: temperature formula slightly differs between chips
    // MPU6050:  (raw / 340) + 36.53
    // MPU6500:  (raw / 333.87) + 21.00
    // We use MPU6050 formula (close enough for both)
    *temp = (raw_t / 340.0f) + 36.53f;

    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════
// MAIN APPLICATION
// ═══════════════════════════════════════════════════════════════════

void app_main(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, "IMU Publisher Firmware Starting");
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    // Configure I2C bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    // Add sensor as I2C device
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SENSOR_ADDR,
        .scl_speed_hz = I2C_SPEED_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle,
                                               &dev_config,
                                               &sensor_handle));

    // Init sensor
    if (sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init FAILED. Halting.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Starting data stream in 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // CSV header
    printf("# IMU Data Stream\n");
    printf("# timestamp_ms,ax,ay,az,gx,gy,gz,temp\n");
    printf("# units: ms,m/s²,m/s²,m/s²,rad/s,rad/s,rad/s,°C\n");

    // Sampling loop
    float ax, ay, az, gx, gy, gz, temp;
    const TickType_t sample_period = pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t sample_count = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, sample_period);

        if (sensor_read_all(&ax, &ay, &az, &gx, &gy, &gz, &temp) == ESP_OK) {
            int64_t timestamp_ms = esp_timer_get_time() / 1000;

            printf("%lld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f\n",
                   timestamp_ms, ax, ay, az, gx, gy, gz, temp);

            sample_count++;

#if ENABLE_STATUS_LOG
            if (sample_count % SAMPLE_RATE_HZ == 0) {
                ESP_LOGI(TAG, "Samples sent: %lu | temp: %.2f°C",
                         sample_count, temp);
            }
#endif
        } else {
            ESP_LOGW(TAG, "Sensor read failed");
        }
    }
}
