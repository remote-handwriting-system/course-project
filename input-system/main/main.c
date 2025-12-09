/**
 * @file main.c
 * @brief Main application demonstrating dual encoder 2D position tracking
 *
 * This application reads two AS5600 magnetic encoders and calculates the
 * end-effector position of a 2-link planar arm using forward kinematics.
 * It includes automatic zero-position calibration on startup.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "double-angles-reader.h"
#include "2d-pos-encoder.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    /* Initialize the dual AS5600 magnetic encoder system
     * Configures I2C communication and checks magnet detection status */
    ESP_ERROR_CHECK(double_angles_reader_init());

    /* Initialize 2D position encoder with physical arm segment lengths
     * Units can be mm, cm, inches, etc. - just keep them consistent
     * Example configuration: first arm = 100mm, second arm = 80mm */
    ESP_ERROR_CHECK(pos_2d_init(100.0f, 80.0f));

    /* Allow mechanical system to settle before capturing calibration reference
     * This ensures stable encoder readings for accurate zero-position setup */
    ESP_LOGI(TAG, "Waiting 3 seconds before calibration...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* Capture current arm configuration as the zero/home reference position
     * All subsequent position calculations will be relative to this pose */
    ESP_LOGI(TAG, "Calibrating zero position (set current arm position as straight)...");
    ESP_ERROR_CHECK(pos_2d_calibrate_zero_position());

    /* Main control loop - reads encoders and calculates position at 10Hz */
    while (1) {
        float theta1_deg, theta2_deg;  // Joint angles in degrees
        float pos_x, pos_y;             // End-effector Cartesian coordinates
        bool read_success = false;
        bool pos_success = false;

        /* Read angular positions from both magnetic encoders */
        if (double_angles_reader_read_encoder1_degrees(&theta1_deg) == ESP_OK &&
            double_angles_reader_read_encoder2_degrees(&theta2_deg) == ESP_OK) {
            read_success = true;
        }

        /* Compute end-effector position using forward kinematics
         * Applies bias correction and converts joint angles to (x,y) coordinates */
        if (pos_2d_get_position(&pos_x, &pos_y) == ESP_OK) {
            pos_success = true;
        }

        /* Display joint angles and calculated position on single line for easy monitoring */
        if (read_success && pos_success) {
            ESP_LOGI(TAG, "θ1=%6.2f° θ2=%6.2f° | Position: x=%7.2f y=%7.2f",
                     theta1_deg, theta2_deg, pos_x, pos_y);
        } else {
            ESP_LOGE(TAG, "Failed to read encoders or calculate position");
        }

        /* 100ms update interval (10Hz control frequency) */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
