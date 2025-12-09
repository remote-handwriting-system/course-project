#ifndef DOUBLE_ANGLES_READER_H
#define DOUBLE_ANGLES_READER_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Initialize the dual AS5600 magnetic encoder system
 *
 * Configures I2C communication with two AS5600 magnetic rotary encoders using
 * software multiplexing. Both encoders share SCL (GPIO 19) but use separate
 * SDA lines (GPIO 20 for encoder 1, GPIO 21 for encoder 2).
 *
 * The I2C bus operates at 400kHz and automatically switches between encoders
 * by reconfiguring the SDA pin. A 100ms stabilization delay is included after
 * initialization, and magnet detection status is checked for both encoders.
 *
 * @note This function must be called before any encoder read operations.
 * @note Both encoders must be physically wired with shared SCL on GPIO 19.
 *
 * @return ESP_OK on successful initialization, error code otherwise
 */
esp_err_t double_angles_reader_init(void);

/**
 * @brief Read angular position from encoder 1 in degrees
 *
 * Retrieves the current angular position from the first encoder and converts
 * the 12-bit raw value (0-4095) to degrees (0-360°). The encoder is selected
 * via I2C multiplexing before reading.
 *
 * @param[out] degrees Pointer to variable receiving angle in degrees (0-360°)
 *
 * @return ESP_OK if read successful
 * @return ESP_ERR_INVALID_ARG if degrees pointer is NULL
 * @return I2C communication error code if read fails
 */
esp_err_t double_angles_reader_read_encoder1_degrees(float *degrees);

/**
 * @brief Read angular position from encoder 2 in degrees
 *
 * Retrieves the current angular position from the second encoder and converts
 * the 12-bit raw value (0-4095) to degrees (0-360°). The encoder is selected
 * via I2C multiplexing before reading.
 *
 * @param[out] degrees Pointer to variable receiving angle in degrees (0-360°)
 *
 * @return ESP_OK if read successful
 * @return ESP_ERR_INVALID_ARG if degrees pointer is NULL
 * @return I2C communication error code if read fails
 */
esp_err_t double_angles_reader_read_encoder2_degrees(float *degrees);

/**
 * @brief Read angular position from encoder 1 in radians
 *
 * Retrieves the current angular position from the first encoder and converts
 * the 12-bit raw value (0-4095) to radians (0-2π). Useful for trigonometric
 * calculations and kinematic computations.
 *
 * @param[out] radians Pointer to variable receiving angle in radians (0-2π)
 *
 * @return ESP_OK if read successful
 * @return ESP_ERR_INVALID_ARG if radians pointer is NULL
 * @return I2C communication error code if read fails
 */
esp_err_t double_angles_reader_read_encoder1_radians(float *radians);

/**
 * @brief Read angular position from encoder 2 in radians
 *
 * Retrieves the current angular position from the second encoder and converts
 * the 12-bit raw value (0-4095) to radians (0-2π). Useful for trigonometric
 * calculations and kinematic computations.
 *
 * @param[out] radians Pointer to variable receiving angle in radians (0-2π)
 *
 * @return ESP_OK if read successful
 * @return ESP_ERR_INVALID_ARG if radians pointer is NULL
 * @return I2C communication error code if read fails
 */
esp_err_t double_angles_reader_read_encoder2_radians(float *radians);

/**
 * @brief Read raw 12-bit angle value from encoder 1
 *
 * Retrieves the unprocessed 12-bit angle measurement (0-4095) directly from
 * the AS5600 sensor. This provides maximum resolution without floating-point
 * conversion overhead.
 *
 * @param[out] raw_angle Pointer to variable receiving raw angle value (0-4095)
 *
 * @return ESP_OK if read successful
 * @return ESP_ERR_INVALID_ARG if raw_angle pointer is NULL
 * @return I2C communication error code if read fails
 */
esp_err_t double_angles_reader_read_encoder1_raw(uint16_t *raw_angle);

/**
 * @brief Read raw 12-bit angle value from encoder 2
 *
 * Retrieves the unprocessed 12-bit angle measurement (0-4095) directly from
 * the AS5600 sensor. This provides maximum resolution without floating-point
 * conversion overhead.
 *
 * @param[out] raw_angle Pointer to variable receiving raw angle value (0-4095)
 *
 * @return ESP_OK if read successful
 * @return ESP_ERR_INVALID_ARG if raw_angle pointer is NULL
 * @return I2C communication error code if read fails
 */
esp_err_t double_angles_reader_read_encoder2_raw(uint16_t *raw_angle);

#endif // DOUBLE_ANGLES_READER_H
