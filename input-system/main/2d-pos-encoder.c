#include "2d-pos-encoder.h"
#include <stdbool.h>
#include "double-angles-reader.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "2D_POS";

// System configuration and state variables
static bool initialized = false;
static float arm1_len = 0.0f;
static float arm2_len = 0.0f;

// Angular bias offsets for zero-position calibration (stored in radians)
static float bias1_rad = 0.0f;
static float bias2_rad = 0.0f;

esp_err_t pos_2d_init(float arm1_length, float arm2_length)
{
    if (arm1_length <= 0.0f || arm2_length <= 0.0f) {
        ESP_LOGE(TAG, "Invalid arm lengths: arm1=%.2f, arm2=%.2f", arm1_length, arm2_length);
        return ESP_ERR_INVALID_ARG;
    }

    arm1_len = arm1_length;
    arm2_len = arm2_length;
    initialized = true;

    ESP_LOGI(TAG, "2D position encoder initialized: arm1=%.2f, arm2=%.2f", arm1_len, arm2_len);

    return ESP_OK;
}

esp_err_t pos_2d_get_position(float *pos_x, float *pos_y)
{
    if (!initialized) {
        ESP_LOGE(TAG, "2D position encoder not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pos_x == NULL || pos_y == NULL) {
        ESP_LOGE(TAG, "NULL pointer provided");
        return ESP_ERR_INVALID_ARG;
    }

    // Read current encoder angles in radians (before bias correction)
    float theta1_rad_uncorrected, theta2_rad_uncorrected;
    esp_err_t err;

    err = double_angles_reader_read_encoder1_radians(&theta1_rad_uncorrected);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read encoder 1");
        return err;
    }

    err = double_angles_reader_read_encoder2_radians(&theta2_rad_uncorrected);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read encoder 2");
        return err;
    }

    // Apply bias correction to obtain calibrated joint angles
    float theta1_rad = theta1_rad_uncorrected - bias1_rad;
    float theta2_rad = theta2_rad_uncorrected - bias2_rad;

    // Forward kinematics for 2-link planar arm
    // x = L1*cos(θ1) + L2*cos(θ1 + θ2)
    // y = L1*sin(θ1) + L2*sin(θ1 + θ2)
    *pos_x = arm1_len * cosf(theta1_rad) + arm2_len * cosf(theta1_rad + theta2_rad);
    *pos_y = arm1_len * sinf(theta1_rad) + arm2_len * sinf(theta1_rad + theta2_rad);

    return ESP_OK;
}

esp_err_t pos_2d_get_arm_lengths(float *arm1_length, float *arm2_length)
{
    if (!initialized) {
        ESP_LOGE(TAG, "2D position encoder not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (arm1_length == NULL || arm2_length == NULL) {
        ESP_LOGE(TAG, "NULL pointer provided");
        return ESP_ERR_INVALID_ARG;
    }

    *arm1_length = arm1_len;
    *arm2_length = arm2_len;

    return ESP_OK;
}

esp_err_t pos_2d_set_arm_lengths(float arm1_length, float arm2_length)
{
    if (arm1_length <= 0.0f || arm2_length <= 0.0f) {
        ESP_LOGE(TAG, "Invalid arm lengths: arm1=%.2f, arm2=%.2f", arm1_length, arm2_length);
        return ESP_ERR_INVALID_ARG;
    }

    arm1_len = arm1_length;
    arm2_len = arm2_length;

    ESP_LOGI(TAG, "Arm lengths updated: arm1=%.2f, arm2=%.2f", arm1_len, arm2_len);

    return ESP_OK;
}

esp_err_t pos_2d_calibrate_zero_position(float *theta2_bias_deg)
{
    if (!initialized) {
        ESP_LOGE(TAG, "2D position encoder not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read current encoder angles in radians
    float theta1_rad_current, theta2_rad_current;
    esp_err_t err;

    err = double_angles_reader_read_encoder1_radians(&theta1_rad_current);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read encoder 1 during calibration");
        return err;
    }

    err = double_angles_reader_read_encoder2_radians(&theta2_rad_current);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read encoder 2 during calibration");
        return err;
    }

    // Store current angles as the bias values
    bias1_rad = theta1_rad_current;
    bias2_rad = theta2_rad_current;
    *theta2_bias_deg = theta2_rad_current * (360/(2*M_PI));

    ESP_LOGI(TAG, "Calibrated zero position: bias1=%.3f rad (%.1f°), bias2=%.3f rad (%.1f°)",
             bias1_rad, bias1_rad * 180.0f / M_PI,
             bias2_rad, bias2_rad * 180.0f / M_PI);

    return ESP_OK;
}

esp_err_t pos_2d_get_bias(float *bias1_radians, float *bias2_radians)
{
    if (!initialized) {
        ESP_LOGE(TAG, "2D position encoder not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (bias1_radians == NULL || bias2_radians == NULL) {
        ESP_LOGE(TAG, "NULL pointer provided");
        return ESP_ERR_INVALID_ARG;
    }

    *bias1_radians = bias1_rad;
    *bias2_radians = bias2_rad;

    return ESP_OK;
}

esp_err_t pos_2d_set_bias(float bias1_radians, float bias2_radians)
{
    if (!initialized) {
        ESP_LOGE(TAG, "2D position encoder not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    bias1_rad = bias1_radians;
    bias2_rad = bias2_radians;

    ESP_LOGI(TAG, "Bias set manually: bias1=%.3f rad (%.1f°), bias2=%.3f rad (%.1f°)",
             bias1_rad, bias1_rad * 180.0f / M_PI,
             bias2_rad, bias2_rad * 180.0f / M_PI);

    return ESP_OK;
}

esp_err_t pos_2d_reset_bias(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "2D position encoder not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    bias1_rad = 0.0f;
    bias2_rad = 0.0f;

    ESP_LOGI(TAG, "Bias reset to zero");

    return ESP_OK;
}
