#include "actuator_math.h"
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void inverse_kinematics(float target_x,
                       float target_y,
                       int8_t elbow_sign,
                       float arm1_len,
                       float arm2_len,
                       uint16_t servo_midpoint,
                       int prev_shoulder,
                       int prev_elbow,
                       int max_speed,
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

    // Calculate coordinated speeds for straight-line motion
    int delta_shoulder = abs(servo_val_shoulder - prev_shoulder);
    int delta_elbow = abs(servo_val_elbow - prev_elbow);

    // Find the joint that needs to move the most
    int max_delta = (delta_shoulder > delta_elbow) ? delta_shoulder : delta_elbow;

    if (max_delta == 0) {
        // No movement needed
        result->servo_speed_shoulder = max_speed;
        result->servo_speed_elbow = max_speed;
    } else {
        // Scale speeds proportionally so both joints finish at the same time
        result->servo_speed_shoulder = (delta_shoulder * max_speed) / max_delta;
        result->servo_speed_elbow = (delta_elbow * max_speed) / max_delta;

        // Ensure minimum speed of 1 if there's any movement
        if (delta_shoulder > 0 && result->servo_speed_shoulder == 0) {
            result->servo_speed_shoulder = 1;
        }
        if (delta_elbow > 0 && result->servo_speed_elbow == 0) {
            result->servo_speed_elbow = 1;
        }
    }
}

float low_pass_pos_filter(float old_filtered, float new_data, float alpha) {
    return alpha * new_data + (1.0f - alpha) * old_filtered;
}

void velocity_filter_init(VelocityFilter* filter, float max_speed) {
    filter->prev_x = 0.0f;
    filter->prev_y = 0.0f;
    filter->filtered_x = 0.0f;
    filter->filtered_y = 0.0f;
    filter->prev_time_ms = 0;
    filter->initialized = false;
    filter->max_speed = max_speed;  // e.g., 100.0f for 100 mm/s
}

void velocity_filter_apply(VelocityFilter* filter, float raw_x, float raw_y,
                          float* filtered_x, float* filtered_y) {
    uint32_t current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Initialize on first call
    if (!filter->initialized) {
        filter->prev_x = raw_x;
        filter->prev_y = raw_y;
        filter->filtered_x = raw_x;
        filter->filtered_y = raw_y;
        filter->prev_time_ms = current_time_ms;
        filter->initialized = true;
        *filtered_x = raw_x;
        *filtered_y = raw_y;
        return;
    }
    
    // Calculate time delta (in seconds)
    float dt = (current_time_ms - filter->prev_time_ms) / 1000.0f;
    if (dt <= 0.0f) dt = 0.02f;  // Fallback to 20ms if time didn't advance
    
    // Calculate velocity (mm/s)
    float velocity_x = (raw_x - filter->prev_x) / dt;
    float velocity_y = (raw_y - filter->prev_y) / dt;
    float speed = sqrtf(velocity_x * velocity_x + velocity_y * velocity_y);
    
    // Adaptive alpha based on speed
    // Slow movement (0 mm/s) -> alpha = 0.1 (heavy filtering)
    // Fast movement (max_speed) -> alpha = 0.7 (light filtering)
    float speed_ratio = speed / filter->max_speed;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;  // Clamp to max
    float alpha = 0.1f + 0.6f * speed_ratio;
    
    // Apply low-pass filter
    filter->filtered_x = alpha * raw_x + (1.0f - alpha) * filter->filtered_x;
    filter->filtered_y = alpha * raw_y + (1.0f - alpha) * filter->filtered_y;
    
    // Update outputs
    *filtered_x = filter->filtered_x;
    *filtered_y = filter->filtered_y;
    
    // Store for next iteration
    filter->prev_x = raw_x;
    filter->prev_y = raw_y;
    filter->prev_time_ms = current_time_ms;
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
