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


// macros
#define ENCODER_READING_STACK_SIZE      4096
#define TRANSMITTER_STACK_SIZE          4096
#define PACKET_BUFFER_SIZE              5
#define BOOT_BUTTON_GPIO                GPIO_NUM_9
#define BUTTON_DEBOUNCE_MS              50
#define ELBOW_SIGN_SWITCH_THRESHOLD_DEG 1
#define ARM1_LEN                        72.0f
#define ARM2_LEN                        72.0f
#define TAG_MAIN                        "MAIN"
#define TAG_ENC                         "ENC"
#define TAG_BTN                         "BUTTON"


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


void init_encoders() {
    // configures I2C communication
    ESP_ERROR_CHECK(double_angles_reader_init());

    // initialize 2D position with physical arm segment lengths
    ESP_ERROR_CHECK(pos_2d_init(ARM1_LEN, ARM2_LEN));
}

void calibrate_encoders() {
    ESP_LOGI(TAG_ENC, "Waiting 2 seconds before calibration...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Capture current arm configuration as the zero/home reference position
    ESP_LOGI(TAG_ENC, "Calibrating zero position (set current arm position as straight)...");
    ESP_ERROR_CHECK(pos_2d_calibrate_zero_position(&theta2_bias_deg));

    ESP_LOGI(TAG_ENC, "Calibration complete");
}

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

void input_reading_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

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

    // start threads
    xTaskCreate(button_task, "ButtonTask", 2048, NULL, 12, NULL);
    xTaskCreate(input_reading_task, "EncoderTask", 4096, NULL, 10, NULL);
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG_MAIN, "========================================");
    ESP_LOGI(TAG_MAIN, "System started successfully!");
    ESP_LOGI(TAG_MAIN, "Press button on GPIO %d to calibrate and start transmission", BOOT_BUTTON_GPIO);
    ESP_LOGI(TAG_MAIN, "Current state: IDLE");
    ESP_LOGI(TAG_MAIN, "========================================");
}
