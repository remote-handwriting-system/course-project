#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "lwip/sockets.h"


// macros
#define HOST_IP_ADDR "192.168.4.1"
#define PORT 3333


// log tags
static const char *TAG = "TCP_CLIENT";


// TODO move to ../components
typedef struct __attribute__((packed)) {
    float pos_x;
    float pos_y;
    float force;
    uint32_t timestamp;
} packet_t;


// global vars
extern QueueHandle_t encoder_reading_queue; // defined in main.c


void tcp_client_task(void *pvParameters) {
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    while (1) {
        // setup target address
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);

        // create socket
        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Connecting to %s...", host_ip);

        // connect (blocking)
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(1000 / portTICK_PERIOD_MS); // wait before retry
            continue;
        }
        
        // optimize latency
        int nodelay = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        ESP_LOGI(TAG, "Successfully connected");

        // Keep connection alive and transmit when data is available
        packet_t packet_to_send;
        while (1) {
            // Try to receive data with timeout (100ms) to keep connection responsive
            // This allows the connection to stay alive even when not transmitting
            if (xQueueReceive(encoder_reading_queue, &packet_to_send, pdMS_TO_TICKS(100))) {
                // send data
                int err = send(sock, &packet_to_send, sizeof(packet_t), 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break; // reconnect
                }
            }
            // If no data, loop continues - keeps connection alive
        }

        shutdown(sock, 0);
        close(sock);
        ESP_LOGI(TAG, "Connection closed, attempting to reconnect...");
    }
}