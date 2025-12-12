#include "double-angles-reader.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "AS5600";

// AS5600 magnetic encoder I2C register addresses
#define AS5600_ADDRESS      0x36  // Fixed I2C address for AS5600 sensors
#define RAW_ANGLE_HIGH      0x0C  // Register address for high byte of raw angle
#define RAW_ANGLE_LOW       0x0D  // Register address for low byte of raw angle
#define STATUS_REG          0x0B  // Register address for magnet detection status

// I2C Configuration - Single bus, software multiplexed
// IMPORTANT: Both encoders must share the same SCL line physically!
// Wire the encoders as follows:
//   Encoder 1: SDA -> GPIO 20, SCL -> GPIO 19
//   Encoder 2: SDA -> GPIO 21, SCL -> GPIO 19 (shared with Encoder 1)
#define I2C_MASTER_SCL_IO      19        // GPIO for SCL (shared between both encoders)
#define I2C_MASTER_SDA_IO_0    20        // GPIO for SDA (Encoder 1)
#define I2C_MASTER_SDA_IO_1    21        // GPIO for SDA (Encoder 2)

#define I2C_MASTER_NUM         I2C_NUM_0 // I2C port number
#define I2C_MASTER_FREQ_HZ     400000    // I2C master clock frequency (400kHz)
#define I2C_MASTER_TX_BUF_DISABLE 0      // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0      // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS  1000

typedef enum {
    ENCODER_1 = 0,
    ENCODER_2 = 1
} encoder_select_t;

static encoder_select_t current_encoder = ENCODER_1;

// Reconfigure I2C SDA pin for the selected encoder
static esp_err_t select_encoder(encoder_select_t encoder)
{
    if (encoder == current_encoder) {
        return ESP_OK;  // Already selected
    }


    // Configure for the selected encoder
    int sda_pin = (encoder == ENCODER_1) ? I2C_MASTER_SDA_IO_0 : I2C_MASTER_SDA_IO_1;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reconfigure I2C for encoder %d (error 0x%x)", encoder + 1, err);
        return err;
    }

    current_encoder = encoder;

    // Small delay after switching to let the line settle (still good practice)
    vTaskDelay(pdMS_TO_TICKS(1)); // TODO figure out delay

    return ESP_OK;
}

// Initialize I2C
static esp_err_t i2c_master_init(void)
{
    // Initialize with Encoder 1 first
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C");
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver");
        return err;
    }

    ESP_LOGI(TAG, "I2C initialized on SCL=%d, multiplexed SDA (Enc1=%d, Enc2=%d)",
             I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO_0, I2C_MASTER_SDA_IO_1);

    current_encoder = ENCODER_1;

    return ESP_OK;
}

// Read raw angle from AS5600 on selected encoder
static esp_err_t as5600_read_raw_angle(encoder_select_t encoder, uint16_t *angle)
{
    // Switch to the correct encoder
    esp_err_t err = select_encoder(encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select encoder %d", encoder + 1);
        return err;
    }

    uint8_t data[2];

    // Write register address
    err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        AS5600_ADDRESS,
        &(uint8_t){RAW_ANGLE_HIGH}, 1,
        data, 2,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed for encoder %d (error 0x%x: %s)",
                 encoder + 1, err, esp_err_to_name(err));
        return err;
    }

    // Combine high and low bytes
    *angle = ((uint16_t)data[0] << 8) | data[1];

    return ESP_OK;
}

// Check magnet status on selected encoder
static esp_err_t as5600_check_magnet_status(encoder_select_t encoder, const char *encoder_name)
{
    // Switch to the correct encoder
    esp_err_t err = select_encoder(encoder);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t status;

    err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        AS5600_ADDRESS,
        &(uint8_t){STATUS_REG}, 1,
        &status, 1,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] Failed to read status register", encoder_name);
        return err;
    }

    bool magnet_detected = status & 0x20;
    bool magnet_too_strong = status & 0x08;
    bool magnet_too_weak = status & 0x10;

    if (magnet_too_strong) {
        ESP_LOGW(TAG, "[%s] Magnet too strong - move it further away", encoder_name);
    } else if (magnet_too_weak) {
        ESP_LOGW(TAG, "[%s] Magnet too weak - move it closer", encoder_name);
    } else if (magnet_detected) {
        ESP_LOGI(TAG, "[%s] Magnet detected - good position", encoder_name);
    } else {
        ESP_LOGW(TAG, "[%s] No magnet detected", encoder_name);
    }

    return ESP_OK;
}

// Public API Implementation

esp_err_t double_angles_reader_init(void)
{
    ESP_LOGI(TAG, "Initializing 2x AS5600 Magnetic Encoders (Software Multiplexed)");

    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return err;
    }

    ESP_LOGI(TAG, "I2C initialized successfully");

    // Wait a bit for sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check magnet status for both encoders
    as5600_check_magnet_status(ENCODER_1, "Encoder 1");
    as5600_check_magnet_status(ENCODER_2, "Encoder 2");

    return ESP_OK;
}

esp_err_t double_angles_reader_read_encoder1_degrees(float *degrees)
{
    if (degrees == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_angle;
    esp_err_t err = as5600_read_raw_angle(ENCODER_1, &raw_angle);
    if (err != ESP_OK) {
        return err;
    }

    *degrees = raw_angle * (360.0f / 4096.0f);
    return ESP_OK;
}

esp_err_t double_angles_reader_read_encoder2_degrees(float *degrees)
{
    if (degrees == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_angle;
    esp_err_t err = as5600_read_raw_angle(ENCODER_2, &raw_angle);
    if (err != ESP_OK) {
        return err;
    }

    *degrees = raw_angle * (360.0f / 4096.0f);
    return ESP_OK;
}

esp_err_t double_angles_reader_read_encoder1_radians(float *radians)
{
    if (radians == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_angle;
    esp_err_t err = as5600_read_raw_angle(ENCODER_1, &raw_angle);
    if (err != ESP_OK) {
        return err;
    }

    *radians = raw_angle * (2.0f * M_PI / 4096.0f);
    return ESP_OK;
}

esp_err_t double_angles_reader_read_encoder2_radians(float *radians)
{
    if (radians == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_angle;
    esp_err_t err = as5600_read_raw_angle(ENCODER_2, &raw_angle);
    if (err != ESP_OK) {
        return err;
    }

    *radians = raw_angle * (2.0f * M_PI / 4096.0f);
    return ESP_OK;
}

esp_err_t double_angles_reader_read_encoder1_raw(uint16_t *raw_angle)
{
    if (raw_angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return as5600_read_raw_angle(ENCODER_1, raw_angle);
}

esp_err_t double_angles_reader_read_encoder2_raw(uint16_t *raw_angle)
{
    if (raw_angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return as5600_read_raw_angle(ENCODER_2, raw_angle);
}
