#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "packet.h" 


// macros
#define PORT 3333
#define TAG  "TCP_SERVER"


// global vars
extern QueueHandle_t packet_queue; // defined in main.c


void tcp_server_task(void *pvParameters) {
    int addr_family = AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    ip_protocol = IPPROTO_IP;

    while (1) {

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        listen(listen_sock, 1);

        ESP_LOGI(TAG, "Socket listening on port %d", PORT);

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);

        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);

        int nodelay = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        int buf_size = 1024;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size));

        ESP_LOGI(TAG, "Client Connected!");

        packet_t rx_packet;
        while (1) {
            // read bytes
            int len = recv(sock, &rx_packet, sizeof(packet_t), 0);

            if (len < 0) {
                ESP_LOGE(TAG, "Recv failed: errno %d", errno);
                break; 
            } else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed by client");
                break;
            } else if (len == sizeof(packet_t)) {
                BaseType_t result = xQueueSend(packet_queue, &rx_packet, portMAX_DELAY);

                if (result != pdPASS) {
                    ESP_LOGW(TAG, "Queue full, dropping packet."); 
                }
            } else {
                ESP_LOGW(TAG, "Received incomplete packet size: %d bytes.", len);
            }
        }
        
        shutdown(sock, 0);
        close(sock);
        close(listen_sock);
    }
}