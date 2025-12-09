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
#include "tcp_client.h"


// macros
#define ENCODER_READING_STACK_SIZE (4096)
#define TRANSMITTER_STACK_SIZE (4096)
#define PACKET_BUFFER_SIZE (30)


// log tags
static const char *TAG_MAIN = "MAIN";
static const char *TAG_ENC = "ENC";


// TODO: MOVE TO ../components
// TODO: sync time between sender and receiver (just calculate offset)
typedef struct __attribute__((packed)) {
    float angle1_deg;
    float angle2_deg;
    float force;
    uint32_t timestamp_us; 
} packet_t;


// global vars
QueueHandle_t encoder_reading_queue;


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

void encoder_reading_task(void *pvParameters) {
    // run exactly every 10ms
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    while (1) {
        float theta1_deg, theta2_deg;
        float pos_x, pos_y;
        
        // read encoder
        bool read_success = (double_angles_reader_read_encoder1_degrees(&theta1_deg) == ESP_OK &&
                             double_angles_reader_read_encoder2_degrees(&theta2_deg) == ESP_OK);
        
        // convert angles to position
        if (read_success) {
            pos_2d_get_position(&pos_x, &pos_y);
        }

        // prepare packet
        if (read_success) {
            packet_t packet;
            packet.angle1_deg = theta1_deg;
            packet.angle2_deg = theta2_deg;
            packet.timestamp_us = xTaskGetTickCount() * portTICK_PERIOD_MS; // Current time in ms

            // send to queue, overwrite if full
            xQueueSend(encoder_reading_queue, &packet, 0);
            
            ESP_LOGI(TAG_ENC, "TX: T1:%.1f T2:%.1f", theta1_deg, theta2_deg);
        } else {
            ESP_LOGE(TAG_ENC, "Encoder Read Fail");
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

    // connect to wifi (blocking)
    wifi_init_sta();

    // create queue
    encoder_reading_queue = xQueueCreate(PACKET_BUFFER_SIZE, sizeof(packet_t));

    // initialize encoders
    init_encoders();

    // start encoder thread
    xTaskCreate(encoder_reading_task, "EncoderTask", 4096, NULL, 10, NULL);
    
    // start tcp client thread
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG_MAIN, "system started");
}
