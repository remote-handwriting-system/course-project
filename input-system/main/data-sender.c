#include "data-sender.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "DATA_SENDER";

// Ring buffer structure for managing packet queue
typedef struct {
    data_packet_t *packets;    // Array of data packets
    uint32_t capacity;         // Maximum number of packets
    uint32_t head;             // Write position (next slot to fill)
    uint32_t tail;             // Read position (next slot to read)
    uint32_t count;            // Current number of packets in buffer
    uint32_t high_water_mark;  // Maximum count reached (for diagnostics)
} ring_buffer_t;

// Module state
static bool initialized = false;
static ring_buffer_t buffer = {0};

esp_err_t data_sender_init(uint32_t buffer_size)
{
    if (buffer_size == 0) {
        ESP_LOGE(TAG, "Invalid buffer size: must be greater than 0");
        return ESP_ERR_INVALID_ARG;
    }

    // Clean up any existing buffer
    if (initialized) {
        ESP_LOGW(TAG, "Re-initializing data sender, freeing old buffer");
        data_sender_deinit();
    }

    // Allocate packet buffer
    buffer.packets = (data_packet_t *)malloc(buffer_size * sizeof(data_packet_t));
    if (buffer.packets == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for %lu packets", buffer_size);
        return ESP_ERR_NO_MEM;
    }

    // Initialize ring buffer state
    buffer.capacity = buffer_size;
    buffer.head = 0;
    buffer.tail = 0;
    buffer.count = 0;
    buffer.high_water_mark = 0;

    initialized = true;

    ESP_LOGI(TAG, "Data sender initialized with buffer size: %lu packets (%lu bytes)",
             buffer_size, buffer_size * sizeof(data_packet_t));

    return ESP_OK;
}

esp_err_t data_sender_write_packet(float pos_x, float pos_y, float pressure)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Data sender not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create packet structure
    data_packet_t packet = {
        .pos_x = pos_x,
        .pos_y = pos_y,
        .pressure = pressure
    };

    // Check if buffer is full - implement overwrite policy
    if (buffer.count >= buffer.capacity) {
        ESP_LOGW(TAG, "Buffer full (%lu packets), overwriting oldest data", buffer.capacity);
        // Advance tail to discard oldest packet
        buffer.tail = (buffer.tail + 1) % buffer.capacity;
        buffer.count--;
    }

    // Write packet to buffer at head position
    memcpy(&buffer.packets[buffer.head], &packet, sizeof(data_packet_t));

    // Advance head pointer (circular)
    buffer.head = (buffer.head + 1) % buffer.capacity;
    buffer.count++;

    // Update high water mark for diagnostics
    if (buffer.count > buffer.high_water_mark) {
        buffer.high_water_mark = buffer.count;
    }

    return ESP_OK;
}

esp_err_t data_sender_write_packet_struct(const data_packet_t *packet)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Data sender not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (packet == NULL) {
        ESP_LOGE(TAG, "NULL packet pointer provided");
        return ESP_ERR_INVALID_ARG;
    }

    return data_sender_write_packet(packet->pos_x, packet->pos_y, packet->pressure);
}

esp_err_t data_sender_peek_packet(data_packet_t *packet)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Data sender not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (packet == NULL) {
        ESP_LOGE(TAG, "NULL packet pointer provided");
        return ESP_ERR_INVALID_ARG;
    }

    if (buffer.count == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    // Copy packet from tail position without removing it
    memcpy(packet, &buffer.packets[buffer.tail], sizeof(data_packet_t));

    return ESP_OK;
}

esp_err_t data_sender_read_packet(data_packet_t *packet)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Data sender not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (packet == NULL) {
        ESP_LOGE(TAG, "NULL packet pointer provided");
        return ESP_ERR_INVALID_ARG;
    }

    if (buffer.count == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    // Copy packet from tail position
    memcpy(packet, &buffer.packets[buffer.tail], sizeof(data_packet_t));

    // Advance tail pointer and decrement count
    buffer.tail = (buffer.tail + 1) % buffer.capacity;
    buffer.count--;

    return ESP_OK;
}

esp_err_t data_sender_get_packet_count(uint32_t *count)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Data sender not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (count == NULL) {
        ESP_LOGE(TAG, "NULL count pointer provided");
        return ESP_ERR_INVALID_ARG;
    }

    *count = buffer.count;
    return ESP_OK;
}

bool data_sender_is_empty(void)
{
    if (!initialized) {
        return true;
    }
    return (buffer.count == 0);
}

bool data_sender_is_full(void)
{
    if (!initialized) {
        return false;
    }
    return (buffer.count >= buffer.capacity);
}

esp_err_t data_sender_clear_buffer(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Data sender not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Reset ring buffer pointers and count
    buffer.head = 0;
    buffer.tail = 0;
    buffer.count = 0;

    ESP_LOGI(TAG, "Buffer cleared");

    return ESP_OK;
}

esp_err_t data_sender_get_stats(uint32_t *capacity, uint32_t *count, uint32_t *high_water)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Data sender not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (capacity == NULL || count == NULL) {
        ESP_LOGE(TAG, "NULL pointer provided for required parameters");
        return ESP_ERR_INVALID_ARG;
    }

    *capacity = buffer.capacity;
    *count = buffer.count;

    // High water mark is optional
    if (high_water != NULL) {
        *high_water = buffer.high_water_mark;
    }

    return ESP_OK;
}

esp_err_t data_sender_deinit(void)
{
    if (!initialized) {
        return ESP_OK;  // Already deinitialized
    }

    // Free allocated buffer memory
    if (buffer.packets != NULL) {
        free(buffer.packets);
        buffer.packets = NULL;
    }

    // Reset buffer state
    memset(&buffer, 0, sizeof(ring_buffer_t));
    initialized = false;

    ESP_LOGI(TAG, "Data sender deinitialized");

    return ESP_OK;
}
