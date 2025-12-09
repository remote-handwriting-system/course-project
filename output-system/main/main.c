#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_ap.h"
#include "led_strip.h"

#include "tcp_server.h"


// macros
#define LED_PIN 8 
#define STRIP_GPIO (8)
#define PACKET_BUFFER_SIZE (30)


// log tags
static const char *TAG_MAIN = "MAIN";
static const char *TAG_SERVO = "SERVO";


// TODO: MOVE TO ../components
// TODO: sync time between sender and receiver (just calculate offset)
typedef struct __attribute__((packed)) {
    float angle1_deg;
    float angle2_deg;
    float force;
    uint32_t timestamp_us; 
} packet_t;


// global vars
led_strip_handle_t led_strip;
QueueHandle_t servo_queue;


void configure_led(void) {
    static const led_strip_config_t strip_config = {
        .strip_gpio_num = STRIP_GPIO,
        .max_leds = 1,
    };
    static const led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
}


// This task waits for angle data and updates the LED color
void servo_task(void *pvParameters)
{
    led_strip_handle_t current_led_strip;
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = 1, 
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, 
        .flags.with_dma = false,
    };
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &current_led_strip));
    led_strip_clear(current_led_strip);

    ESP_LOGI(TAG_SERVO, "Color LED Ready on GPIO %d. Waiting for angles...", LED_PIN);

    packet_t rx_packet = {0};
    uint8_t r = 0, g = 0, b = 0;
    const uint8_t brightness = 50;
    BaseType_t result;
    bool packet_received = false;

    while (1) {
        
        // drain queue
        packet_received = false;
        do {
            // Check for a packet immediately (0 delay)
            result = xQueueReceive(servo_queue, &rx_packet, 0); 
            
            if (result == pdPASS) {
                packet_received = true;
            }
        } 
        while (result == pdPASS); // loop until the queue is empty

        // process data
        if (packet_received) { 
            float angle = rx_packet.angle1_deg;
            while(angle >= 360.0f) angle -= 360.0f;
            while(angle < 0.0f) angle += 360.0f;
            r = (uint8_t)(brightness * (angle)/ 360.0f);
            g = (uint8_t)(brightness * (angle) / 360.0f);
            b = 0;
            ESP_ERROR_CHECK(led_strip_set_pixel(current_led_strip, 0, r, g, b));
            ESP_ERROR_CHECK(led_strip_refresh(current_led_strip));
            
            ESP_LOGI(TAG_SERVO, "Angle: %.1f -> RGB(%d, %d, %d)", angle, r, g, b);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

void app_main(void)
{
    configure_led();

    servo_queue = xQueueCreate(PACKET_BUFFER_SIZE, sizeof(packet_t));

    // initialize nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // start access point
    wifi_init_softap();

    // start servo thread
    xTaskCreate(servo_task, "servo_task", 4096, NULL, 10, NULL);

    // start tcp server thread
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG_MAIN, "System Initialization Complete.");
}