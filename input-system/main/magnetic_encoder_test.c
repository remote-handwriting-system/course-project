#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "AS5600";

// AS5600 Configuration
#define AS5600_ADDRESS      0x36
#define RAW_ANGLE_HIGH      0x0C
#define RAW_ANGLE_LOW       0x0D
#define STATUS_REG          0x0B

// I2C Configuration
#define I2C_MASTER_SCL_IO   20        // GPIO for SCL
#define I2C_MASTER_SDA_IO   19        // GPIO for SDA
#define I2C_MASTER_NUM      I2C_NUM_0 // I2C port number
#define I2C_MASTER_FREQ_HZ  400000    // I2C master clock frequency (400kHz)
#define I2C_MASTER_TX_BUF_DISABLE 0   // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0   // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS 1000

// Initialize I2C
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                             I2C_MASTER_RX_BUF_DISABLE, 
                             I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Read raw angle from AS5600
static esp_err_t as5600_read_raw_angle(uint16_t *angle)
{
    uint8_t data[2];
    
    // Write register address
    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        AS5600_ADDRESS,
        &(uint8_t){RAW_ANGLE_HIGH}, 1,
        data, 2,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );
    
    if (err != ESP_OK) {
        return err;
    }
    
    // Combine high and low bytes
    *angle = ((uint16_t)data[0] << 8) | data[1];
    
    return ESP_OK;
}

// Check magnet status
static esp_err_t as5600_check_magnet_status(void)
{
    uint8_t status;
    
    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        AS5600_ADDRESS,
        &(uint8_t){STATUS_REG}, 1,
        &status, 1,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status register");
        return err;
    }
    
    bool magnet_detected = status & 0x20;
    bool magnet_too_strong = status & 0x08;
    bool magnet_too_weak = status & 0x10;
    
    if (magnet_too_strong) {
        ESP_LOGW(TAG, "Magnet too strong - move it further away");
    } else if (magnet_too_weak) {
        ESP_LOGW(TAG, "Magnet too weak - move it closer");
    } else if (magnet_detected) {
        ESP_LOGI(TAG, "Magnet detected - good position");
    } else {
        ESP_LOGW(TAG, "No magnet detected");
    }
    
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing AS5600 Magnetic Encoder");
    
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    // Wait a bit for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check magnet status
    as5600_check_magnet_status();
    
    // Main loop
    while (1) {
        uint16_t raw_angle;
        
        if (as5600_read_raw_angle(&raw_angle) == ESP_OK) {
            // Convert to degrees and radians
            float degrees = raw_angle * (360.0f / 4096.0f);
            float radians = raw_angle * (2.0f * 3.14159265f / 4096.0f);
            
            ESP_LOGI(TAG, "Raw: %4d | Angle: %6.2f° | Radians: %.3f", 
                     raw_angle, degrees, radians);
        } else {
            ESP_LOGE(TAG, "Failed to read angle");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}