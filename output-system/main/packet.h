#ifndef PACKET_H
#define PACKET_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

// Data structure for the packet received over TCP
typedef struct __attribute__((packed)) {
    float angle1_deg;
    float angle2_deg;
    float force;
    uint32_t timestamp_us; 
} packet_t;

// Global declaration of the queue handle. 
// This will be DEFINED in main.c and initialized in app_main().
extern QueueHandle_t servo_queue; 

#endif // PACKET_H