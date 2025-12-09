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
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/queue.h"
#include "double-angles-reader.h"
#include "2d-pos-encoder.h"
#include "wifi_sta.h"

// Macros
#define ENCODER_READING_STACK_SIZE (4096)
#define TRANSMITTER_STACK_SIZE (4096)
#define PACKET_BUFFER_SIZE (128)


// Tags
static const char *TAG_MAIN = "MAIN";
static const char *TAG_WIFI = "WIFI";
static const char *TAG_ENC = "ENC";


// TODO: MOVE TO ../components
// TODO: sync time between sender and receiver (just calculate offset)
typedef struct __attribute__((packed)) {
    float angle1_deg;
    float angle2_deg;
    float force;
    uint32_t timestamp_us; 
} packet_t;


// Global vars
QueueHandle_t encoder_reading_queue;


// Initialization
void init_encoders() {
    /* Initialize the dual AS5600 magnetic encoder system
     * Configures I2C communication and checks magnet detection status */
    ESP_ERROR_CHECK(double_angles_reader_init());

    /* Initialize 2D position encoder with physical arm segment lengths
     * Units can be mm, cm, inches, etc. - just keep them consistent
     * Example configuration: first arm = 100mm, second arm = 80mm */
    ESP_ERROR_CHECK(pos_2d_init(100.0f, 80.0f));

    /* Allow mechanical system to settle before capturing calibration reference
     * This ensures stable encoder readings for accurate zero-position setup */
    ESP_LOGI(TAG_ENC, "Waiting 3 seconds before calibration...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* Capture current arm configuration as the zero/home reference position
     * All subsequent position calculations will be relative to this pose */
    ESP_LOGI(TAG_ENC, "Calibrating zero position (set current arm position as straight)...");
    ESP_ERROR_CHECK(pos_2d_calibrate_zero_position());
}

void init_wifi() {

}


// Thread tasks
void encoder_reading_task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Run exactly every 10ms

    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        float theta1_deg, theta2_deg;
        float pos_x, pos_y;
        
        // 1. Read Hardware
        bool read_success = (double_angles_reader_read_encoder1_degrees(&theta1_deg) == ESP_OK &&
                             double_angles_reader_read_encoder2_degrees(&theta2_deg) == ESP_OK);
        
        // 2. Local Calculation (Optional: Only if you need X/Y locally)
        if (read_success) {
            pos_2d_get_position(&pos_x, &pos_y);
        }

        // 3. Prepare Packet
        if (read_success) {
            packet_t packet;
            packet.angle1_deg = theta1_deg;
            packet.angle2_deg = theta2_deg;
            packet.timestamp_us = xTaskGetTickCount() * portTICK_PERIOD_MS; // Current time in ms

            // 4. Send to Queue (Non-blocking: overwrite if full)
            // We pass the address of 'packet', FreeRTOS copies the data in.
            xQueueSend(encoder_reading_queue, &packet, 0);
            
            // Log occasionally (logging every 10ms spans the console)
            static int log_counter = 0;
            if (log_counter++ > 100) { 
                ESP_LOGI(TAG_ENC, "TX: T1:%.1f T2:%.1f", theta1_deg, theta2_deg);
                log_counter = 0;
            }
        } else {
            ESP_LOGE(TAG_ENC, "Encoder Read Fail");
        }

        // 5. Precise Timing
        // vTaskDelayUntil ensures exactly 10ms period, compensating for calculation time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

}



// Main program loop
void app_main(void)
{
    // 1. Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Connect to WiFi (Blocks until connected!)
    wifi_init_sta();

    // 3. Create Queue
    encoder_reading_queue = xQueueCreate(50, sizeof(packet_t));

    // 4. Initialize Hardware
    init_encoders();

    // 5. Start Encoder Task
    xTaskCreate(encoder_reading_task, "EncoderTask", 4096, NULL, 5, NULL);
    
    // 6. Start TCP Sender Task (We will write this next)
    // xTaskCreate(tcp_sender_task, "TCPTask", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG_MAIN, "System Started");
}
