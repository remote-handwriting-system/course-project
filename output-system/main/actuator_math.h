#ifndef ACTUATOR_MATH_H
#define ACTUATOR_MATH_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Structure to hold inverse kinematics results
 */
typedef struct {
    int servo_val_shoulder;  // Calculated shoulder servo position (0-1023)
    int servo_val_elbow;     // Calculated elbow servo position (0-1023)
    int servo_speed_shoulder; // Calculated shoulder servo speed (0-1023)
    int servo_speed_elbow;    // Calculated elbow servo speed (0-1023)
} ik_result_t;

/**
 * @brief Calculate inverse kinematics for 2-DOF arm with coordinated motion
 *
 * Computes the joint angles and servo positions for a 2-link planar arm
 * to reach the target (x, y) position. Uses geometric IK solution.
 * Also calculates coordinated servo speeds to ensure straight-line motion.
 *
 * @param[in] target_x Target X coordinate in mm
 * @param[in] target_y Target Y coordinate in mm
 * @param[in] elbow_sign Elbow configuration (+1 for elbow-up, -1 for elbow-down)
 * @param[in] arm1_len Length of first arm segment in mm
 * @param[in] arm2_len Length of second arm segment in mm
 * @param[in] servo_midpoint Servo center position value (typically 512)
 * @param[in] prev_shoulder Previous shoulder servo position (for speed calculation)
 * @param[in] prev_elbow Previous elbow servo position (for speed calculation)
 * @param[in] max_speed Maximum servo speed (0-1023)
 * @param[out] result Pointer to store computed servo positions and speeds
 */
void inverse_kinematics(float target_x,
                       float target_y,
                       int8_t elbow_sign,
                       float arm1_len,
                       float arm2_len,
                       uint16_t servo_midpoint,
                       int prev_shoulder,
                       int prev_elbow,
                       int max_speed,
                       ik_result_t *result);

/**
 * @brief Low-pass filter for position data
 *
 * Applies exponential smoothing to filter noisy position data.
 * Output = alpha * new_data + (1 - alpha) * old_filtered
 *
 * @param[in] old_filtered Previously filtered value
 * @param[in] new_data New raw data point
 * @param[in] alpha Filter coefficient (0.0 to 1.0)
 *                  - Higher values (closer to 1.0) = less filtering, faster response
 *                  - Lower values (closer to 0.0) = more filtering, slower response
 * @return float The filtered output value
 */
float low_pass_pos_filter(float old_filtered, float new_data, float alpha);

typedef struct {
    float prev_x;
    float prev_y;
    float filtered_x;
    float filtered_y;
    uint32_t prev_time_ms;
    bool initialized;
    float max_speed;  // mm/s - tune this to your drawing speed
} VelocityFilter;

void velocity_filter_init(VelocityFilter* filter, float max_speed);
void velocity_filter_apply(VelocityFilter* filter, float raw_x, float raw_y, 
                          float* filtered_x, float* filtered_y);

/**
 * @brief Calculate force feedback control for end effector
 *
 * Implements proportional control to adjust end effector position based on
 * force error. Updates position only if error exceeds deadband threshold.
 *
 * @param[in] target_force Desired force setpoint (ADC counts)
 * @param[in] current_force Measured force from sensor (ADC counts)
 * @param[in,out] end_effector_pos Current position, updated with new position
 * @param[in] servo_midpoint Minimum servo position limit
 * @param[in] servo_limit Maximum servo position limit
 * @param[in] prop_gain Proportional gain constant
 * @param[in] force_deadband Force error deadband in ADC counts
 * @return int16_t The force error value
 */
int16_t force_effect(uint16_t target_force,
                    uint16_t current_force,
                    uint16_t *end_effector_pos,
                    uint16_t servo_midpoint,
                    uint16_t servo_limit,
                    float prop_gain,
                    int force_deadband);

#endif // ACTUATOR_MATH_H
