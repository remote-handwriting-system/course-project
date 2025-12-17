#include "actuator_math.h"
#include <math.h>
#include <stdlib.h>

void inverse_kinematics(float target_x,
                       float target_y,
                       int8_t elbow_sign,
                       float arm1_len,
                       float arm2_len,
                       uint16_t servo_midpoint,
                       ik_result_t *result) {

    float cos_theta2 = (powf(target_x, 2.0f) + powf(target_y, 2.0f) - powf(arm1_len, 2.0f) - powf(arm2_len, 2.0f)) / (2.0f * arm1_len * arm2_len);

    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

    float theta2_rad = (elbow_sign > 0) ? acosf(cos_theta2) : -acosf(cos_theta2);

    int servo_val_elbow = servo_midpoint + (int)(theta2_rad * 195.57f);

    float k1 = arm1_len + arm2_len * cosf(theta2_rad);
    float k2 = arm2_len * sinf(theta2_rad);
    float theta1_rad = atan2f(target_y, target_x) - atan2f(k2, k1);
    int servo_val_shoulder = servo_midpoint + (int)(theta1_rad * 195.57f);

    if (servo_val_elbow < 0) servo_val_elbow = 0;
    if (servo_val_elbow > 1023) servo_val_elbow = 1023;
    if (servo_val_shoulder < 0) servo_val_shoulder = 0;
    if (servo_val_shoulder > 1023) servo_val_shoulder = 1023;

    result->servo_val_shoulder = servo_val_shoulder;
    result->servo_val_elbow = servo_val_elbow;
}

float low_pass_pos_filter(float old_filtered, float new_data, float alpha) {
    return alpha * new_data + (1.0f - alpha) * old_filtered;
}

int16_t force_effect(uint16_t target_force,
                    uint16_t current_force,
                    uint16_t *end_effector_pos,
                    uint16_t servo_midpoint,
                    uint16_t servo_limit,
                    float prop_gain,
                    int force_deadband) {

    int16_t error = target_force - current_force;

    if (abs(error) > force_deadband) {
        *end_effector_pos += (int16_t)((float)error * prop_gain);
    }

    if (*end_effector_pos > servo_limit) {
        *end_effector_pos = servo_limit;
    }
    if (*end_effector_pos < servo_midpoint) {
        *end_effector_pos = servo_midpoint;
    }

    return error;
}
