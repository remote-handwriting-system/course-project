#ifndef PACKET_H
#define PACKET_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

typedef struct __attribute__((packed)) {
    float pos_x;
    float pos_y;
    float angle_elbow;
    float angle_shoulder;
    int8_t elbow_sign;
    uint16_t force;
    uint32_t timestamp;
} packet_t;

#endif // PACKET_H