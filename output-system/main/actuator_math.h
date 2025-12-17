#ifndef ACTUATOR_MATH_H
#define ACTUATOR_MATH_H

#include <stdint.h>

/**
 * @brief Structure to hold inverse kinematics results
 */
typedef struct {
    int servo_val_shoulder;  // Calculated shoulder servo position (0-1023)
    int servo_val_elbow;     // Calculated elbow servo position (0-1023)
} ik_result_t;

/**
 * @brief Calculate inverse kinematics for 2-DOF arm
 *
 * Computes the joint angles and servo positions for a 2-link planar arm
 * to reach the target (x, y) position. Uses geometric IK solution.
 *
 * @param[in] target_x Target X coordinate in mm
 * @param[in] target_y Target Y coordinate in mm
 * @param[in] elbow_sign Elbow configuration (+1 for elbow-up, -1 for elbow-down)
 * @param[in] arm1_len Length of first arm segment in mm
 * @param[in] arm2_len Length of second arm segment in mm
 * @param[in] servo_midpoint Servo center position value (typically 512)
 * @param[out] result Pointer to store computed servo positions
 */
void inverse_kinematics(float target_x,
                       float target_y,
                       int8_t elbow_sign,
                       float arm1_len,
                       float arm2_len,
                       uint16_t servo_midpoint,
                       ik_result_t *result);

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
