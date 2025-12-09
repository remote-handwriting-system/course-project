#ifndef DATA_SENDER_H
#define DATA_SENDER_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Data packet structure for position and pressure measurements
 *
 * This structure represents a single measurement sample containing the
 * end-effector position (x, y) and force/pressure reading. All values
 * are stored as floats for precision and ease of transmission.
 */
typedef struct {
    float pos_x;      // X-coordinate of position
    float pos_y;      // Y-coordinate of position
    float pressure;   // Pressure or force measurement value
} data_packet_t;

/**
 * @brief Initialize the data sender buffer system
 *
 * Allocates and configures the internal ring buffer for storing data packets
 * before network transmission. The buffer size determines how many packets
 * can be queued if network transmission is slower than data acquisition.
 *
 * @param[in] buffer_size Maximum number of data packets the buffer can hold
 *
 * @return ESP_OK on successful initialization
 * @return ESP_ERR_INVALID_ARG if buffer_size is zero
 * @return ESP_ERR_NO_MEM if buffer allocation fails
 */
esp_err_t data_sender_init(uint32_t buffer_size);

/**
 * @brief Add a data packet to the transmission buffer
 *
 * Writes a new measurement sample to the buffer. This should be called at
 * each data acquisition tick. If the buffer is full, behavior depends on
 * the configured overflow policy (currently overwrites oldest data).
 *
 * @param[in] pos_x X-coordinate of current position
 * @param[in] pos_y Y-coordinate of current position
 * @param[in] pressure Current pressure/force reading
 *
 * @return ESP_OK if packet successfully buffered
 * @return ESP_ERR_INVALID_STATE if data sender not initialized
 */
esp_err_t data_sender_write_packet(float pos_x, float pos_y, float pressure);

/**
 * @brief Add a pre-constructed data packet to the buffer
 *
 * Alternative interface accepting a pointer to a complete data_packet_t
 * structure. Useful when the packet is constructed elsewhere.
 *
 * @param[in] packet Pointer to data packet structure to buffer
 *
 * @return ESP_OK if packet successfully buffered
 * @return ESP_ERR_INVALID_STATE if data sender not initialized
 * @return ESP_ERR_INVALID_ARG if packet pointer is NULL
 */
esp_err_t data_sender_write_packet_struct(const data_packet_t *packet);

/**
 * @brief Retrieve the next packet from the buffer without removing it
 *
 * Peeks at the oldest buffered packet without consuming it. Useful for
 * checking data before committing to a network send operation.
 *
 * @param[out] packet Pointer to structure receiving the peeked packet data
 *
 * @return ESP_OK if packet retrieved successfully
 * @return ESP_ERR_INVALID_STATE if data sender not initialized
 * @return ESP_ERR_INVALID_ARG if packet pointer is NULL
 * @return ESP_ERR_NOT_FOUND if buffer is empty
 */
esp_err_t data_sender_peek_packet(data_packet_t *packet);

/**
 * @brief Read and remove the next packet from the buffer
 *
 * Retrieves the oldest buffered packet and removes it from the queue.
 * This should be called by the network transmission layer after
 * successfully sending the data.
 *
 * @param[out] packet Pointer to structure receiving the packet data
 *
 * @return ESP_OK if packet retrieved and removed successfully
 * @return ESP_ERR_INVALID_STATE if data sender not initialized
 * @return ESP_ERR_INVALID_ARG if packet pointer is NULL
 * @return ESP_ERR_NOT_FOUND if buffer is empty
 */
esp_err_t data_sender_read_packet(data_packet_t *packet);

/**
 * @brief Get number of packets currently in the buffer
 *
 * Returns the count of buffered packets waiting to be transmitted.
 * Useful for monitoring buffer utilization and detecting transmission
 * bottlenecks.
 *
 * @param[out] count Pointer to variable receiving packet count
 *
 * @return ESP_OK if count retrieved successfully
 * @return ESP_ERR_INVALID_STATE if data sender not initialized
 * @return ESP_ERR_INVALID_ARG if count pointer is NULL
 */
esp_err_t data_sender_get_packet_count(uint32_t *count);

/**
 * @brief Check if the buffer is empty
 *
 * @return true if buffer contains no packets
 * @return false if buffer contains one or more packets
 */
bool data_sender_is_empty(void);

/**
 * @brief Check if the buffer is full
 *
 * @return true if buffer is at maximum capacity
 * @return false if buffer has space for more packets
 */
bool data_sender_is_full(void);

/**
 * @brief Clear all packets from the buffer
 *
 * Removes all buffered packets, resetting the buffer to empty state.
 * Useful for recovering from error conditions or resetting the system.
 *
 * @return ESP_OK if buffer cleared successfully
 * @return ESP_ERR_INVALID_STATE if data sender not initialized
 */
esp_err_t data_sender_clear_buffer(void);

/**
 * @brief Get buffer statistics
 *
 * Retrieves current buffer utilization metrics including total capacity,
 * current packet count, and high water mark (maximum usage).
 *
 * @param[out] capacity Pointer to variable receiving total buffer capacity
 * @param[out] count Pointer to variable receiving current packet count
 * @param[out] high_water Pointer to variable receiving maximum usage (optional, can be NULL)
 *
 * @return ESP_OK if statistics retrieved successfully
 * @return ESP_ERR_INVALID_STATE if data sender not initialized
 * @return ESP_ERR_INVALID_ARG if required pointers are NULL
 */
esp_err_t data_sender_get_stats(uint32_t *capacity, uint32_t *count, uint32_t *high_water);

/**
 * @brief Deinitialize the data sender and free resources
 *
 * Releases all allocated memory and resets the module to uninitialized state.
 * Should be called during system shutdown or before re-initialization.
 *
 * @return ESP_OK on successful deinitialization
 */
esp_err_t data_sender_deinit(void);

#endif // DATA_SENDER_H
