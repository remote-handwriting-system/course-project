/**
 * @file main.c
 * @brief Output System (Receiver) Main
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_ap.h" // Your custom header

static const char *TAG_MAIN = "MAIN";

// Task to write to the terminal (Placeholder for future data display)
void terminal_task(void *pvParameters) {
    while (1) {
        // Right now, just log that we are alive.
        // Later, this task (or the TCP task) will print the received angles.
        ESP_LOGI(TAG_MAIN, "Output System Running... [AP Active]");
        
        // Update at 1Hz
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    // 1. Initialize NVS (Crucial for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Start the Access Point
    // This starts the internal WiFi driver tasks. They run in the background.
    wifi_init_softap();

    // 3. Start your User Task
    xTaskCreate(terminal_task, "TerminalTask", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG_MAIN, "System Initialization Complete.");
}