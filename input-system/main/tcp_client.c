#include "tcp_client.h"
#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

#include "packet.h"


// macros
#define TAG "TCP_CLIENT"


// global vars
extern QueueHandle_t packet_queue; // defined in input-main.c


void tcp_client_task(void *pvParameters)
{
    char host_ip[] = "192.168.4.1";
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(3333);

        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int nodelay = 1;
        if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay)) < 0) {
            ESP_LOGE(TAG, "Failed to set TCP_NODELAY: errno %d", errno);
        }

        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, 3333);
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "Successfully connected");

        packet_t packet;
        while (1) {
            // block indefinitely until a packet arrives in the queue
            if (xQueueReceive(packet_queue, &packet, portMAX_DELAY) == pdTRUE) {
                int err = send(sock, &packet, sizeof(packet_t), 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break; // break loop to close socket and reconnect
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}