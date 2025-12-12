#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "double-angles-reader.h"
#include "2d-pos-encoder.h"
#include "wifi_sta.h"
#include "tcp_client.h"


// macros
#define ENCODER_READING_STACK_SIZE (4096)
#define TRANSMITTER_STACK_SIZE (4096)
#define PACKET_BUFFER_SIZE (30)
#define BOOT_BUTTON_GPIO GPIO_NUM_9
#define BUTTON_DEBOUNCE_MS 50


// log tags
static const char *TAG_MAIN = "MAIN";
static const char *TAG_ENC = "ENC";
static const char *TAG_BTN = "BUTTON";


// system state
typedef enum {
    STATE_IDLE,           // Waiting for first button press
    STATE_CALIBRATING,    // Performing calibration
    STATE_TRANSMITTING    // Actively transmitting data
} system_state_t;


typedef struct __attribute__((packed)) {
    float pos_x;
    float pos_y;
    float force;
    uint32_t timestamp;
} packet_t;


// global vars
QueueHandle_t encoder_reading_queue;
volatile system_state_t system_state = STATE_IDLE;
volatile bool encoders_initialized = false;


void init_encoders() {
    /* Initialize the dual AS5600 magnetic encoder system
     * Configures I2C communication and checks magnet detection status */
    ESP_ERROR_CHECK(double_angles_reader_init());

    /* Initialize 2D position encoder with physical arm segment lengths
     * Units can be mm, cm, inches, etc. - just keep them consistent
     * Example configuration: first arm = 100mm, second arm = 80mm */
    ESP_ERROR_CHECK(pos_2d_init(72.0f, 72.0f));
}

void calibrate_encoders() {
    /* Allow mechanical system to settle before capturing calibration reference
     * This ensures stable encoder readings for accurate zero-position setup */
    ESP_LOGI(TAG_ENC, "Waiting 3 seconds before calibration...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* Capture current arm configuration as the zero/home reference position
     * All subsequent position calculations will be relative to this pose */
    ESP_LOGI(TAG_ENC, "Calibrating zero position (set current arm position as straight)...");
    ESP_ERROR_CHECK(pos_2d_calibrate_zero_position());
    ESP_LOGI(TAG_ENC, "Calibration complete!");
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
                        xQueueReset(encoder_reading_queue);
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

void encoder_reading_task(void *pvParameters) {
    // run exactly every 10ms
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    while (1) {
        // Only read and transmit when in TRANSMITTING state
        if (system_state == STATE_TRANSMITTING && encoders_initialized) {
            float theta1_deg, theta2_deg;
            float pos_x, pos_y;

            // read encoder
            bool read_success = (double_angles_reader_read_encoder1_degrees(&theta1_deg) == ESP_OK &&
                                 double_angles_reader_read_encoder2_degrees(&theta2_deg) == ESP_OK);

            // prepare packet
            if (read_success) {
                // convert angles to position
                pos_2d_get_position(&pos_x, &pos_y);

                packet_t packet;
                packet.pos_x = pos_x;
                packet.pos_y = pos_y;
                packet.force = 0.0f;  // Placeholder for future force sensor
                packet.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS; // Current time in ms

                // send to queue, overwrite if full
                xQueueSend(encoder_reading_queue, &packet, 0);

                // Logging output
                ESP_LOGI(TAG_ENC, "Pos X: %.2f, Pos Y: %.2f (Angle 1: %.2f°, Angle 2: %.2f°)", pos_x, pos_y, theta1_deg, theta2_deg);
            } else {
                ESP_LOGE(TAG_ENC, "Encoder Read Fail");
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

    // Initialize WiFi and connect (blocking)
    wifi_init_sta();

    // Create data queue
    encoder_reading_queue = xQueueCreate(PACKET_BUFFER_SIZE, sizeof(packet_t));
    if (encoder_reading_queue == NULL) {
        ESP_LOGE(TAG_MAIN, "Failed to create queue!");
        return;
    }

    // Initialize button GPIO
    init_button();

    // Create button task (polling-based, no ISR needed)
    BaseType_t task_created = xTaskCreate(button_task, "ButtonTask", 2048, NULL, 12, NULL);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create button task!");
        return;
    }
    ESP_LOGI(TAG_MAIN, "Button task created (polling mode)");

    // Start encoder reading and TCP client tasks
    xTaskCreate(encoder_reading_task, "EncoderTask", 4096, NULL, 10, NULL);
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG_MAIN, "========================================");
    ESP_LOGI(TAG_MAIN, "System started successfully!");
    ESP_LOGI(TAG_MAIN, "Press button on GPIO %d to calibrate and start transmission", BOOT_BUTTON_GPIO);
    ESP_LOGI(TAG_MAIN, "Current state: IDLE");
    ESP_LOGI(TAG_MAIN, "========================================");
}
