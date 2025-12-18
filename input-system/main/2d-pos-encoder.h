#ifndef TWO_D_POS_ENCODER_H
#define TWO_D_POS_ENCODER_H

#include "esp_err.h"

/**
 * @brief Initialize the 2D position encoder system for a two-link planar arm
 *
 * Configures forward kinematics parameters for calculating end-effector position
 * from joint angles. The system models a 2-link planar manipulator where:
 *   - Encoder 1 controls the base joint (shoulder) angle
 *   - Encoder 2 controls the second joint (elbow) angle
 *   - Both angles are measured relative to the horizontal reference axis
 *
 * The arm lengths can be specified in any consistent unit (mm, cm, inches, etc.)
 * and will determine the unit of calculated positions.
 *
 * @note Must be called before any position calculations or bias operations.
 * @note Requires double_angles_reader_init() to be called first.
 *
 * @param[in] arm1_length Physical length of first arm segment (base to joint 2)
 * @param[in] arm2_length Physical length of second arm segment (joint 2 to end-effector)
 *
 * @return ESP_OK on successful initialization
 * @return ESP_ERR_INVALID_ARG if either length is zero or negative
 */
esp_err_t pos_2d_init(float arm1_length, float arm2_length);

/**
 * @brief Calculate end-effector position using forward kinematics
 *
 * Reads current angles from both encoders, applies any configured bias offsets,
 * and computes the 2D Cartesian position (x, y) of the arm's end-effector using
 * forward kinematics equations:
 *   x = L1·cos(θ1) + L2·cos(θ1 + θ2)
 *   y = L1·sin(θ1) + L2·sin(θ1 + θ2)
 *
 * Position is calculated relative to the base joint origin, with positive x
 * extending right and positive y extending upward.
 *
 * @param[out] pos_x Pointer to variable receiving x-coordinate of end-effector
 * @param[out] pos_y Pointer to variable receiving y-coordinate of end-effector
 *
 * @return ESP_OK if position calculated successfully
 * @return ESP_ERR_INVALID_STATE if system not initialized
 * @return ESP_ERR_INVALID_ARG if either pointer is NULL
 * @return Encoder error code if angle reading fails
 */
esp_err_t pos_2d_get_position(float *pos_x, float *pos_y);

/**
 * @brief Retrieve currently configured arm segment lengths
 *
 * Returns the arm lengths that were set during initialization or by the most
 * recent call to pos_2d_set_arm_lengths().
 *
 * @param[out] arm1_length Pointer to variable receiving first arm length
 * @param[out] arm2_length Pointer to variable receiving second arm length
 *
 * @return ESP_OK if lengths retrieved successfully
 * @return ESP_ERR_INVALID_STATE if system not initialized
 * @return ESP_ERR_INVALID_ARG if either pointer is NULL
 */
esp_err_t pos_2d_get_arm_lengths(float *arm1_length, float *arm2_length);

/**
 * @brief Update arm segment lengths for kinematic calculations
 *
 * Allows dynamic reconfiguration of arm dimensions without reinitializing the
 * entire system. Useful for systems with adjustable or interchangeable arm
 * segments. Changes take effect immediately for subsequent position calculations.
 *
 * @param[in] arm1_length New length for first arm segment
 * @param[in] arm2_length New length for second arm segment
 *
 * @return ESP_OK if lengths updated successfully
 * @return ESP_ERR_INVALID_ARG if either length is zero or negative
 */
esp_err_t pos_2d_set_arm_lengths(float arm1_length, float arm2_length);

/**
 * @brief Calibrate current arm configuration as the zero/home position
 *
 * Captures the current encoder readings and stores them as angular offsets (bias).
 * After calibration, this physical arm configuration is treated as the reference
 * zero position (both joint angles at 0°), and all subsequent angle measurements
 * are relative to this pose.
 *
 * Typical workflow:
 *   1. Physically position the arm in a known straight or home configuration
 *   2. Call this function to capture that position as the zero reference
 *   3. All future position calculations will be relative to this reference
 *
 * The bias values persist until changed by pos_2d_set_bias() or cleared by
 * pos_2d_reset_bias().
 *
 * @note Ensure the arm is stationary when calling this function.
 *
 * @return ESP_OK if calibration successful
 * @return ESP_ERR_INVALID_STATE if system not initialized
 * @return Encoder error code if angle reading fails during calibration
 */
esp_err_t pos_2d_calibrate_zero_position(float *theta1_bias_deg, float *theta2_bias_deg);

/**
 * @brief Retrieve current angular bias offsets
 *
 * Returns the angle offsets (in radians) that are automatically subtracted from
 * raw encoder readings before position calculations. These values are set by
 * pos_2d_calibrate_zero_position() or pos_2d_set_bias().
 *
 * @param[out] bias1_radians Pointer to variable receiving encoder 1 bias in radians
 * @param[out] bias2_radians Pointer to variable receiving encoder 2 bias in radians
 *
 * @return ESP_OK if bias values retrieved successfully
 * @return ESP_ERR_INVALID_STATE if system not initialized
 * @return ESP_ERR_INVALID_ARG if either pointer is NULL
 */
esp_err_t pos_2d_get_bias(float *bias1_radians, float *bias2_radians);

/**
 * @brief Manually configure angular bias offsets
 *
 * Sets the angle offsets (in radians) that will be subtracted from raw encoder
 * readings. Provides an alternative to automatic calibration via
 * pos_2d_calibrate_zero_position() when bias values are known in advance
 * (e.g., from previous calibration or mechanical design specifications).
 *
 * @param[in] bias1_radians Angular offset for encoder 1 in radians
 * @param[in] bias2_radians Angular offset for encoder 2 in radians
 *
 * @return ESP_OK if bias set successfully
 * @return ESP_ERR_INVALID_STATE if system not initialized
 */
esp_err_t pos_2d_set_bias(float bias1_radians, float bias2_radians);

/**
 * @brief Clear all angular bias offsets
 *
 * Resets both encoder bias values to zero, causing position calculations to use
 * raw encoder readings without any offset compensation. Useful for returning to
 * absolute encoder measurements or clearing an unwanted calibration.
 *
 * @return ESP_OK if bias reset successfully
 * @return ESP_ERR_INVALID_STATE if system not initialized
 */
esp_err_t pos_2d_reset_bias(void);

#endif // TWO_D_POS_ENCODER_H
