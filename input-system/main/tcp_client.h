#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

/**
 * @brief FreeRTOS Task that acts as a TCP Client.
 * * - Connects to 192.168.4.1 : 3333
 * - Consumes data from the global 'encoder_reading_queue'
 * - Sends packet_t structures over WiFi
 * * @param pvParameters Unused
 */
void tcp_client_task(void *pvParameters);

#endif // TCP_CLIENT_H