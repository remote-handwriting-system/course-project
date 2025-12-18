/**
 * @file input-main.c
 * @brief Robotic Arm Input System - Position Sensing and Transmission
 *
 * This system reads position and force data from a 2-DOF robotic arm using
 * encoders and force sensors, then transmits the data over WiFi/TCP to the
 * output system for replication.
 *
 * System Architecture:
 * - WiFi STA connects to output system access point (192.168.4.1)
 * - Encoder reading task samples arm position at configurable frequency
 * - TCP client queues and transmits position packets
 * - Optional test mode transmits square pattern instead of real data
 *
 * Hardware:
 * - ESP32 microcontroller
 * - 2x Encoders for joint angle measurement (I2C)
 * - Force sensor for end-effector feedback
 * - Boot button for calibration trigger
 *
 * Key Features:
 * - Configurable sampling rate (default 10ms/100Hz)
 * - Automatic calibration on button press
 * - Test mode for transmitting predefined square pattern
 * - Elbow configuration detection (up/down)
 * - Real-time position logging
 *
 * @author Generated with Claude Code
 * @date 2025
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_wifi.h"

#include "double-angles-reader.h"
#include "force-reader.h"
#include "2d-pos-encoder.h"
#include "wifi_sta.h"
#include "tcp_client.h"
#include "packet.h"


// =============================================================================
// CONFIGURATION MACROS
// =============================================================================

// Operation Mode Configuration
// Set to true to transmit square test pattern, false to use real encoder data
#define USE_TEST_SQUARE_MODE        false

// Transmission Frequency
#define INPUT_READING_FREQUENCY_MS  100   // Encoder sampling interval (default: 100ms = 10Hz)
#define SQUARE_TRANSMIT_DELAY_MS    500   // Delay between square waypoints in test mode (default: 500ms)

// GPIO Configuration
#define BOOT_BUTTON_GPIO            GPIO_NUM_9

// Button Configuration
#define BUTTON_DEBOUNCE_MS          50    // Debounce time for button presses

// Task Stack Sizes
#define ENCODER_READING_STACK_SIZE  4096
#define TRANSMITTER_STACK_SIZE      4096

// Communication Configuration
#define PACKET_BUFFER_SIZE          5     // Queue size for packet buffering

// Arm Geometry
#define ARM1_LEN                    72.50f  // Length of first arm segment in mm
#define ARM2_LEN                    100.20f  // Length of second arm segment in mm

// Encoder Configuration
#define ELBOW_SIGN_SWITCH_THRESHOLD_DEG 1  // Threshold for elbow up/down detection

// Logging Tags
#define TAG_MAIN                    "MAIN"
#define TAG_ENC                     "ENC"
#define TAG_BTN                     "BUTTON"


// types
typedef enum {
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_TRANSMITTING
} system_state_t;


// global vars
QueueHandle_t packet_queue;
volatile system_state_t system_state = STATE_IDLE;
volatile bool encoders_initialized = false;
float theta2_bias_deg;


/**
 * @brief Initialize encoder hardware and position calculation
 *
 * Configures I2C communication for dual encoder reading and initializes
 * the 2D position calculator with physical arm geometry.
 */
void init_encoders() {
    // configures I2C communication
    ESP_ERROR_CHECK(double_angles_reader_init());

    // initialize 2D position with physical arm segment lengths
    ESP_ERROR_CHECK(pos_2d_init(ARM1_LEN, ARM2_LEN));
}

/**
 * @brief Calibrate encoders to set zero/home position
 *
 * Waits 2 seconds for the user to position the arm straight, then captures
 * the current encoder readings as the zero reference. This calibration
 * determines the theta2 bias for elbow configuration detection.
 */
void calibrate_encoders() {
    ESP_LOGI(TAG_ENC, "Waiting 2 seconds before calibration...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Capture current arm configuration as the zero/home reference position
    ESP_LOGI(TAG_ENC, "Calibrating zero position (set current arm position as straight)...");
    ESP_ERROR_CHECK(pos_2d_calibrate_zero_position(&theta2_bias_deg));

    ESP_LOGI(TAG_ENC, "Calibration complete");
}

/**
 * @brief Initialize boot button GPIO
 *
 * Configures the boot button as an input with internal pull-up resistor.
 * The button is read via polling (no interrupts). Button press triggers
 * encoder calibration and starts data transmission.
 */
void init_button() {
    ESP_LOGI(TAG_BTN, "Initializing button on GPIO %d", BOOT_BUTTON_GPIO);

    // Configure button GPIO (no interrupt, polling only)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,      // No interrupt needed for polling
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE     // Enable internal pull-up
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BTN, "Failed to configure GPIO: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG_BTN, "GPIO configured successfully");

        // Read initial button state for debugging
        int level = gpio_get_level(BOOT_BUTTON_GPIO);
        ESP_LOGI(TAG_BTN, "Initial button state: %d (0=pressed, 1=released)", level);
    }
}

/**
 * @brief Button polling task for calibration control
 *
 * Continuously polls the boot button and manages system state transitions:
 * - IDLE -> CALIBRATING: Initializes encoders and performs calibration
 * - CALIBRATING -> TRANSMITTING: Starts encoder reading after calibration
 * - TRANSMITTING -> IDLE: Stops data transmission
 *
 * Includes debouncing (BUTTON_DEBOUNCE_MS) to prevent multiple triggers.
 *
 * @param[in] pvParameters Unused FreeRTOS task parameter
 */
void button_task(void *pvParameters) {
    ESP_LOGI(TAG_BTN, "Button task started, waiting for button presses...");

    int last_button_level = 1;  // Start assuming button is released (high due to pull-up)
    TickType_t last_debounce_time = 0;

    while (1) {
        // Poll button state directly (fallback if ISR doesn't work)
        int current_level = gpio_get_level(BOOT_BUTTON_GPIO);

        // Detect falling edge (button press: 1 -> 0)
        if (last_button_level == 1 && current_level == 0) {
            // Debounce
            TickType_t current_time = xTaskGetTickCount();
            if ((current_time - last_debounce_time) > pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
                last_debounce_time = current_time;

                ESP_LOGI(TAG_BTN, "=== BUTTON PRESSED! Current state: %d ===", system_state);

                switch (system_state) {
                    case STATE_IDLE:
                        ESP_LOGI(TAG_BTN, "Starting calibration...");
                        system_state = STATE_CALIBRATING;

                        // Initialize encoders if not already done
                        if (!encoders_initialized) {
                            ESP_LOGI(TAG_BTN, "Initializing encoders...");
                            init_encoders();
                            encoders_initialized = true;
                        }

                        // Perform calibration
                        calibrate_encoders();
                        ESP_LOGI("bias", "%f", theta2_bias_deg);
                        

                        // Transition to transmitting
                        system_state = STATE_TRANSMITTING;
                        ESP_LOGI(TAG_BTN, "Calibration complete. Starting data transmission...");
                        break;

                    case STATE_CALIBRATING:
                        // Ignore button presses during calibration
                        ESP_LOGI(TAG_BTN, "Calibration in progress, ignoring button press");
                        break;

                    case STATE_TRANSMITTING:
                        // Stop transmission
                        ESP_LOGI(TAG_BTN, "Stopping data transmission");
                        system_state = STATE_IDLE;

                        // Clear the queue to stop sending old data
                        xQueueReset(packet_queue);
                        ESP_LOGI(TAG_BTN, "Transmission stopped. System IDLE.");
                        break;
                }
            }
        }

        last_button_level = current_level;

        // Small delay for polling
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Encoder reading and packet transmission task
 *
 * Continuously reads encoder angles and force sensor data, converts to
 * Cartesian coordinates, and transmits position packets over TCP.
 * Runs at frequency defined by INPUT_READING_FREQUENCY_MS.
 *
 * @param[in] pvParameters Unused FreeRTOS task parameter
 */
void input_reading_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(INPUT_READING_FREQUENCY_MS);

    int8_t elbow_sign = 1;
    uint16_t current_force = 0;

    while (1) {
        if (system_state == STATE_TRANSMITTING && encoders_initialized) {
            float theta1_deg, theta2_deg;
            float pos_x, pos_y;

            // read encoder
            bool encoder_read_success = (double_angles_reader_read_encoder1_degrees(&theta1_deg) == ESP_OK &&
                                 double_angles_reader_read_encoder2_degrees(&theta2_deg) == ESP_OK);

            // read force sensor
            bool force_read_success = (force_reader_read_raw(&current_force) == ESP_OK);

            // prepare packet
            if (encoder_read_success && force_read_success) {
                // convert angles to position
                pos_2d_get_position(&pos_x, &pos_y);
                
                // calculate elbow position
                float angle_diff = theta2_deg - theta2_bias_deg;
                if (angle_diff > 180.0f) {
                    angle_diff -= 360.0f;
                }
                if (angle_diff < -180.0f) {
                    angle_diff += 360.0f;
                }
                if (angle_diff > ELBOW_SIGN_SWITCH_THRESHOLD_DEG) {
                    elbow_sign = 1;
                } else if (angle_diff < -ELBOW_SIGN_SWITCH_THRESHOLD_DEG) {
                    elbow_sign = -1;
                }

                packet_t packet;
                packet.pos_x = pos_x;
                packet.pos_y = pos_y;
                packet.elbow_sign = elbow_sign;
                packet.force = current_force;
                packet.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

                // send to queue, overwrite if full
                xQueueSend(packet_queue, &packet, 0);

                // logging
                ESP_LOGI(TAG_ENC, "Pos X: %.2f, Pos Y: %.2f, force: %d", pos_x, pos_y, current_force);
            }
            if (!encoder_read_success){
                ESP_LOGE(TAG_ENC, "Encoder read fail");
            }
            if (!force_read_success){
                // reset force
                current_force = 0.0f; 

                ESP_LOGE(TAG_ENC, "Force read fail");
            }
        }

        // wait
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

}

/**
 * @brief Test mode square pattern transmission task
 *
 * Transmits a predefined square pattern (4 corner waypoints) in a continuous
 * loop for testing the output system without requiring physical encoders.
 * Delay between waypoints is defined by SQUARE_TRANSMIT_DELAY_MS.
 *
 * Square coordinates: (140,-10) -> (160,-10) -> (160,10) -> (140,10) -> repeat
 *
 * @param[in] pvParameters Unused FreeRTOS task parameter
 */
void transmit_square(void *pvParameters) {
    const float square_positions[][2] = {
        {140.0f, -10.0f},
        {160.0f, -10.0f},
        {160.0f, 10.0f},
        {140.0f, 10.0f}
    };

    const TickType_t xDelay = pdMS_TO_TICKS(SQUARE_TRANSMIT_DELAY_MS);

    // Start transmission
    system_state = STATE_TRANSMITTING;
    ESP_LOGI(TAG_MAIN, "Starting square transmission...");

    while (1) {
        for (int i = 0; i < 4; i++) {
            packet_t packet;
            packet.pos_x = square_positions[i][0];
            packet.pos_y = square_positions[i][1];
            packet.elbow_sign = 1;
            packet.force = 0;
            packet.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

            xQueueSend(packet_queue, &packet, portMAX_DELAY);

            ESP_LOGI(TAG_MAIN, "Sent square position: X=%.2f, Y=%.2f", packet.pos_x, packet.pos_y);

            vTaskDelay(xDelay);
        }
    }
}

/**
 * @brief Application entry point
 *
 * Initializes the robotic arm input system in the following order:
 * 1. Initialize NVS (non-volatile storage)
 * 2. Connect to WiFi access point (192.168.4.1)
 * 3. Initialize force sensor
 * 4. Initialize boot button for calibration
 * 5. Create packet queue for TCP transmission
 * 6. Launch appropriate task based on USE_TEST_SQUARE_MODE:
 *    - Test mode: transmit_square task
 *    - Normal mode: input_reading_task
 * 7. Launch TCP client task for network transmission
 *
 * The system connects to the output system's WiFi AP and sends position
 * packets for arm replication.
 */
void app_main(void) {
    // initialize nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG_MAIN, "Initializing system...");

    // initialize WiFi and connect (blocking)
    wifi_init_sta();
    esp_wifi_set_ps(WIFI_PS_NONE);

    // initialize force sensor
    ESP_ERROR_CHECK(force_reader_init());

    // initialize button GPIO
    init_button();

    // create data queue
    packet_queue = xQueueCreate(PACKET_BUFFER_SIZE, sizeof(packet_t));

    // start threads based on configuration
    xTaskCreate(button_task, "ButtonTask", 2048, NULL, 12, NULL);

#if USE_TEST_SQUARE_MODE
    // Test mode: transmit square pattern
    xTaskCreate(transmit_square, "transmit_square", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG_MAIN, "Running in TEST SQUARE MODE");
#else
    // Normal mode: read encoders and transmit real data
    xTaskCreate(input_reading_task, "EncoderTask", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG_MAIN, "Running in ENCODER MODE (frequency: %dms)", INPUT_READING_FREQUENCY_MS);
#endif

    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG_MAIN, "========================================");
    ESP_LOGI(TAG_MAIN, "System started successfully!");
#if !USE_TEST_SQUARE_MODE
    ESP_LOGI(TAG_MAIN, "Press button on GPIO %d to calibrate and start transmission", BOOT_BUTTON_GPIO);
    ESP_LOGI(TAG_MAIN, "Current state: IDLE");
#endif
    ESP_LOGI(TAG_MAIN, "========================================");
}
