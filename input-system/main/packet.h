#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct __attribute__((packed)) {
    float pos_x;
    float pos_y;
    int8_t elbow_sign;
    uint16_t force;
    uint32_t timestamp;
} packet_t;

#endif // PACKET_H