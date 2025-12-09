#ifndef TCP_SERVER_H
#define TCP_SERVER_H

/**
 * @brief FreeRTOS Task that acts as a TCP Server.
 * * - Listens on Port 3333
 * - Accepts incoming connections
 * - Reads packet_t structures
 * - (Future) pushes to Servo Queue
 * * @param pvParameters Unused
 */
void tcp_server_task(void *pvParameters);

#endif // TCP_SERVER_H