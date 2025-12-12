#ifndef FORCE_READER_H
#define FORCE_READER_H

#include "esp_err.h"

/**
 * @brief Initialize the Force Sensitive Resistor (FSR) ADC reader
 *
 * Configures ADC1 Channel 0 for reading FSR sensor values.
 * Uses 12-bit resolution (0-4095) with 12dB attenuation for 0-3.3V range.
 *
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t force_reader_init(void);

/**
 * @brief Read raw ADC value from the force sensor
 *
 * Returns the raw 12-bit ADC reading (0-4095) from the FSR sensor.
 * Higher values indicate greater applied force.
 *
 * @param[out] raw_value Pointer to store the raw ADC reading
 * @return ESP_OK on successful read, ESP_FAIL on error
 */
esp_err_t force_reader_read_raw(int *raw_value);

/**
 * @brief Read normalized force value (0.0 to 1.0)
 *
 * Returns a normalized force reading where:
 * - 0.0 = no force detected
 * - 1.0 = maximum force (full ADC range)
 *
 * @param[out] force Pointer to store normalized force value (0.0-1.0)
 * @return ESP_OK on successful read, ESP_FAIL on error
 */
esp_err_t force_reader_read_normalized(float *force);

/**
 * @brief Deinitialize the force reader and free resources
 *
 * Cleans up ADC unit resources. Call this during shutdown.
 *
 * @return ESP_OK on success
 */
esp_err_t force_reader_deinit(void);

#endif // FORCE_READER_H
