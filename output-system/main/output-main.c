/**
 * @file output-main.c
 * @brief Robotic Arm Output System - Main Control Module
 *
 * This system controls a 2-DOF robotic arm with force feedback using Dynamixel
 * RX-24F servos. It receives position commands over WiFi/TCP and translates them
 * into coordinated servo movements that follow straight-line paths in Cartesian space.
 *
 * System Architecture:
 * - WiFi AP (192.168.4.1) receives position packets from input system
 * - TCP server queues incoming position commands
 * - Servo task (50Hz) processes commands and controls servos via RS485
 * - Inverse kinematics converts (x,y) positions to joint angles
 * - Coordinated speed control ensures straight-line motion
 * - Optional interpolation and filtering for smooth motion
 *
 * Hardware:
 * - ESP32 microcontroller
 * - 3x Dynamixel RX-24F servos (shoulder, elbow, end-effector)
 * - RS485 half-duplex communication (1Mbaud)
 * - Force sensor for end-effector feedback
 *
 * Key Features:
 * - Coordinated joint motion for straight-line Cartesian paths
 * - Configurable interpolation for discrete waypoints or continuous tracking
 * - Optional low-pass filtering for noise reduction
 * - Force feedback control for end-effector
 * - Real-time logging at 5Hz
 *
 * @author Generated with Claude Code
 * @date 2025
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_ap.h"
#include "led_strip.h"

#include "tcp_server.h"
#include "force-reader.h"
#include "packet.h"
#include "actuator_math.h"


// =============================================================================
// CONFIGURATION MACROS
// =============================================================================

// GPIO Configuration
#define LED_PIN                   8
#define STRIP_GPIO                8
#define TX_PIN                    2
#define RX_PIN                    3
#define RTS_PIN                   18

// Communication Configuration
#define UART_PORT                 UART_NUM_1
#define BAUD_RATE                 1000000
#define BUF_SIZE                  256
#define PACKET_BUFFER_SIZE        5     // Queue size for packet buffering (increased for smoother motion)

// Dynamixel Protocol
#define INST_WRITE                0x03
#define ADDR_TORQUE_ENABLE        0x18
#define ADDR_GOAL_POSITION        0x1E
#define ADDR_MOVING_SPEED         0x20
#define ADDR_CW_COMPLIANCE_SLOPE  0x1C
#define ADDR_CCW_COMPLIANCE_SLOPE 0x1D

// Servo Configuration
#define COMPLIANCE_SLOPE          200
#define SERVO_MIDPOINT            512
#define END_SERVO_LIMIT           576 // physical max is 592

// Arm Geometry
#define ARM1_LEN                  72.0f
#define ARM2_LEN                  101.4f

// Motion Control Parameters
// Tuning Guide:
//   - For discrete waypoints (e.g., 4 corners): USE_INTERPOLATION=true, INTERPOLATION_STEP=5.0
//   - For continuous data stream (100Hz):      USE_INTERPOLATION=false or INTERPOLATION_STEP=1.0-3.0
//   - If motion is laggy:                      Increase INTERPOLATION_STEP, increase MAX_SERVO_SPEED
//   - If motion is jittery:                    Enable USE_LOWPASS_FILTER, decrease INTERPOLATION_STEP
#define MAX_SERVO_SPEED           800    // Maximum servo speed (0-1023, tune for responsiveness)
#define USE_INTERPOLATION         false   // Enable interpolation for smooth path following
#define INTERPOLATION_STEP        8.0f   // mm per step - smaller = smoother but slower
#define USE_LOWPASS_FILTER        true  // Enable low-pass filter to reduce noise (adds lag)
#define LOW_PASS_X_ALPHA          0.3f   // Low-pass filter coefficient for X (0-1, higher = less filtering)
#define LOW_PASS_Y_ALPHA          0.3f   // Low-pass filter coefficient for Y (0-1, higher = less filtering)

// Force Control Parameters
#define FORCE_PROP_GAIN           0.05f  // Proportional gain for force control (higher = more jitter)
#define FORCE_DEADBAND            5      // Force error deadband to prevent oscillation

// Logging Tags
#define TAG_MAIN                  "MAIN"
#define TAG_SERVO                 "SERVO"


// global vars
led_strip_handle_t led_strip;
QueueHandle_t packet_queue;
uart_config_t uart_config = {
    .baud_rate = BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

/**
 * @brief Configure the on-board LED strip
 *
 * Initializes a single WS2812 LED using the RMT peripheral.
 * The LED is cleared (turned off) after initialization.
 */
void configure_led(void) {
    static const led_strip_config_t strip_config = {
        .strip_gpio_num = STRIP_GPIO,
        .max_leds = 1,
    };
    static const led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
}

/**
 * @brief Calculate Dynamixel protocol checksum
 *
 * Computes the checksum for a Dynamixel protocol packet according to
 * the specification: ~(ID + LENGTH + INSTRUCTION + PARAM1 + ... + PARAMN)
 *
 * @param[in] packet Pointer to the packet buffer
 * @return Calculated checksum byte
 */
uint8_t calculate_checksum(uint8_t *packet) {
    uint8_t checksum = 0;
    int length_val = packet[3];
    int limit = 3 + length_val - 1;
    for (int i = 2; i <= limit; i++) {
        checksum += packet[i];
    }
    return ~checksum;
}

/**
 * @brief Send a Dynamixel packet and check for errors
 *
 * Transmits a packet to the Dynamixel servos via RS485 half-duplex UART,
 * then reads the response to check for errors. Logs a warning if the servo
 * reports an error condition.
 *
 * @param[in] packet Pointer to the packet buffer to send
 * @param[in] packet_len Length of the packet in bytes
 */
void send_packet_and_check(uint8_t *packet, int packet_len) {
    uart_flush_input(UART_PORT);
    uart_write_bytes(UART_PORT, (const char *)packet, packet_len);
    uart_wait_tx_done(UART_PORT, 10);

    // read response to clear buffer and check for errors
    uint8_t response[64];
    int len = uart_read_bytes(UART_PORT, response, 64, pdMS_TO_TICKS(20)); // Short timeout

    if (len >= 6 && response[0] == 0xFF && response[1] == 0xFF) {
        if (response[4] != 0) {
            ESP_LOGW(TAG_SERVO, "Servo ID %d reported Error: 0x%02X", response[2], response[4]);
        }
    }
}

/**
 * @brief Set the compliance slope for a Dynamixel servo
 *
 * Configures both clockwise (CW) and counter-clockwise (CCW) compliance slopes.
 * Compliance slope controls the flexibility/stiffness of the servo's position control.
 * Higher values = more compliant (softer), lower values = stiffer.
 *
 * @param[in] id Dynamixel servo ID (1-254)
 * @param[in] slope Compliance slope value (0-254, typically 32-200)
 */
void dynamixel_set_compliance_slope(uint8_t id, uint8_t slope) {
    // set CW Slope
    uint8_t packet1[8];
    packet1[0] = 0xFF;
    packet1[1] = 0xFF;
    packet1[2] = id;
    packet1[3] = 0x04;
    packet1[4] = INST_WRITE;
    packet1[5] = ADDR_CW_COMPLIANCE_SLOPE;
    packet1[6] = slope;
    packet1[7] = calculate_checksum(packet1);
    send_packet_and_check(packet1, 8);

    // set CCW Slope
    uint8_t packet2[8];
    packet2[0] = 0xFF;
    packet2[1] = 0xFF;
    packet2[2] = id;
    packet2[3] = 0x04;
    packet2[4] = INST_WRITE;
    packet2[5] = ADDR_CCW_COMPLIANCE_SLOPE;
    packet2[6] = slope;
    packet2[7] = calculate_checksum(packet2);
    send_packet_and_check(packet2, 8);
}

/**
 * @brief Enable or disable torque on a Dynamixel servo
 *
 * Controls whether the servo is actively holding position (torque on) or
 * can be freely moved by hand (torque off). Must be enabled before the
 * servo will respond to position or speed commands.
 *
 * @param[in] id Dynamixel servo ID (1-254)
 * @param[in] enable 1 to enable torque, 0 to disable
 */
void dynamixel_set_torque(uint8_t id, uint8_t enable) {
    uint8_t packet[8];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 0x04;
    packet[4] = INST_WRITE;
    packet[5] = ADDR_TORQUE_ENABLE;
    packet[6] = enable;
    packet[7] = calculate_checksum(packet);
    send_packet_and_check(packet, 8);
}

/**
 * @brief Set the goal position of a Dynamixel servo
 *
 * Commands the servo to move to the specified position. The servo will
 * move at the currently configured speed. Position is automatically clamped
 * to the valid range [0, 1023].
 *
 * @param[in] id Dynamixel servo ID (1-254)
 * @param[in] position Target position (0-1023, where 512 is typically center)
 */
void dynamixel_set_position(uint8_t id, uint16_t position) {
    if (position > 1023) position = 1023;
    uint8_t pL = position & 0xFF;
    uint8_t pH = (position >> 8) & 0xFF;

    uint8_t packet[9];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 0x05;
    packet[4] = INST_WRITE;
    packet[5] = ADDR_GOAL_POSITION;
    packet[6] = pL;
    packet[7] = pH;
    packet[8] = calculate_checksum(packet);
    send_packet_and_check(packet, 9);
}

/**
 * @brief Set the moving speed of a Dynamixel servo
 *
 * Configures how fast the servo moves to goal positions. Speed is automatically
 * clamped to the valid range [0, 1023]. A value of 0 means maximum speed with
 * no speed control. Higher values = faster movement.
 *
 * @param[in] id Dynamixel servo ID (1-254)
 * @param[in] speed Moving speed (0-1023, where 0 = max speed, ~0.111 RPM per unit)
 */
void dynamixel_set_speed(uint8_t id, uint16_t speed) {
    if (speed > 1023) speed = 1023;
    uint8_t sL = speed & 0xFF;
    uint8_t sH = (speed >> 8) & 0xFF;

    uint8_t packet[9];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 0x05;
    packet[4] = INST_WRITE;
    packet[5] = ADDR_MOVING_SPEED;
    packet[6] = sL;
    packet[7] = sH;
    packet[8] = calculate_checksum(packet);
    send_packet_and_check(packet, 9);
}

/**
 * @brief Initialize all Dynamixel servos with default settings
 *
 * Configures the three servos (shoulder, elbow, end-effector) with:
 * - Torque enabled
 * - Initial speed settings for arm servos (400)
 * - Compliance slope for smooth motion
 *
 * Small delays between commands ensure proper servo communication.
 */
void dynamixel_init_servos() {
    dynamixel_set_torque(1, 1); vTaskDelay(pdMS_TO_TICKS(10));
    dynamixel_set_torque(2, 1); vTaskDelay(pdMS_TO_TICKS(10));
    dynamixel_set_torque(3, 1); vTaskDelay(pdMS_TO_TICKS(10));
    dynamixel_set_speed(1, 400); vTaskDelay(pdMS_TO_TICKS(10));
    dynamixel_set_speed(2, 400); vTaskDelay(pdMS_TO_TICKS(10));
    dynamixel_set_position(3, SERVO_MIDPOINT), vTaskDelay(pdMS_TO_TICKS(10));
    dynamixel_set_compliance_slope(1, COMPLIANCE_SLOPE); vTaskDelay(pdMS_TO_TICKS(10));
    dynamixel_set_compliance_slope(2, COMPLIANCE_SLOPE); vTaskDelay(pdMS_TO_TICKS(10));
    dynamixel_set_compliance_slope(3, COMPLIANCE_SLOPE); // TODO determine this

}

/**
 * @brief Main servo control task
 *
 * This FreeRTOS task implements the complete servo control pipeline:
 * 1. Receives position packets from TCP queue
 * 2. Optionally applies low-pass filtering to reduce noise
 * 3. Applies linear interpolation for smooth Cartesian motion
 * 4. Computes inverse kinematics to get joint angles
 * 5. Calculates coordinated servo speeds for straight-line motion
 * 6. Implements force feedback control for end-effector
 * 7. Commands servos with positions and speeds
 *
 * The task runs at 50Hz (20ms) and supports two filtering modes:
 * - Discrete waypoints: USE_INTERPOLATION=true for path generation
 * - Continuous stream: USE_INTERPOLATION=false for direct tracking
 *
 * Configuration parameters are defined at the top of the file as macros.
 *
 * @param[in] pvParameters Unused FreeRTOS task parameter
 */
void servo_task(void *pvParameters) {
    float target_x = ARM1_LEN+ARM2_LEN;
    float target_y = 0.0f;
    float actual_x = ARM1_LEN+ARM2_LEN;
    float actual_y = 0.0f;
    int8_t  current_elbow_sign = 1;
    uint16_t target_force = 0;
    uint16_t current_force = 0;
    uint16_t end_effector_pos = SERVO_MIDPOINT;

    packet_t rx_packet = {0};

    force_reader_init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Run at 50Hz (20ms)

    //placeholder
    uint64_t cnt = 0;

    bool is_first_iteration = true;

    // Track previous servo positions for coordinated motion
    int prev_shoulder = SERVO_MIDPOINT;
    int prev_elbow = SERVO_MIDPOINT;

    // Track previous servo speeds to avoid redundant commands
    int last_speed_shoulder = -1;
    int last_speed_elbow = -1;

    while (1) {
        // 1. Receive Packets (Update the "Target")
        if (xQueueReceive(packet_queue, &rx_packet, 0) == pdPASS) {
            // ESP_LOGI(TAG_SERVO, "packet received");
            float new_target_x = rx_packet.pos_x;
            float new_target_y = rx_packet.pos_y;

            // Optional low-pass filter on incoming target positions (removes noise before interpolation)
            if (USE_LOWPASS_FILTER) {
                if (is_first_iteration) {
                    target_x = new_target_x;
                    target_y = new_target_y;
                    is_first_iteration = false;
                } else {
                    target_x = low_pass_pos_filter(target_x, new_target_x, LOW_PASS_X_ALPHA);
                    target_y = low_pass_pos_filter(target_y, new_target_y, LOW_PASS_Y_ALPHA);
                }
            } else {
                target_x = new_target_x;
                target_y = new_target_y;
            }

            current_elbow_sign = rx_packet.elbow_sign;
            target_force = rx_packet.force;
        }

        // Linear interpolation towards target in Cartesian space (if enabled)
        if (USE_INTERPOLATION) {
            float delta_x = target_x - actual_x;
            float delta_y = target_y - actual_y;
            float distance = sqrtf(delta_x * delta_x + delta_y * delta_y);

            if (distance > INTERPOLATION_STEP) {
                // Move one step towards target
                float ratio = INTERPOLATION_STEP / distance;
                actual_x += delta_x * ratio;
                actual_y += delta_y * ratio;
            } else {
                // Close enough, just use target
                actual_x = target_x;
                actual_y = target_y;
            }
        } else {
            // No interpolation - directly use target (for continuous data streams)
            actual_x = target_x;
            actual_y = target_y;
        }

        // inverse kinematics with coordinated motion
        ik_result_t ik_result;
        inverse_kinematics(actual_x, actual_y, current_elbow_sign, ARM1_LEN, ARM2_LEN,
                          SERVO_MIDPOINT, prev_shoulder, prev_elbow, MAX_SERVO_SPEED, &ik_result);

        // force feedback
        force_reader_read_raw(&current_force);
        int16_t error = force_effect(target_force, current_force, &end_effector_pos, SERVO_MIDPOINT, END_SERVO_LIMIT, FORCE_PROP_GAIN, FORCE_DEADBAND);

        // Printing
        if (cnt >= 1) {
            cnt = 0;
            // UBaseType_t queue_items = uxQueueMessagesWaiting(packet_queue);
            // ESP_LOGI(TAG_SERVO, "Queue: %d/%d packets | target: (%.2f, %.2f) actual: (%.2f, %.2f)",
            //          queue_items, PACKET_BUFFER_SIZE, target_x, target_y, actual_x, actual_y);
            // ESP_LOGI(TAG_SERVO, "servo speeds: shoulder=%d elbow=%d", ik_result.servo_speed_shoulder, ik_result.servo_speed_elbow);
            ESP_LOGI(TAG_SERVO, "target force: %d sensed: %d error: %d", target_force, current_force, error);

            // float servo1_angle = ((float) ik_result.servo_val_elbow-(float)SERVO_MIDPOINT)*(360.0f/1023.0f);
            // float servo2_angle = ((float) ik_result.servo_val_shoulder-(float)SERVO_MIDPOINT)*(360.0f/1023.0f);
            // ESP_LOGI(TAG_SERVO, "%d and %d", ik_result.servo_val_elbow, ik_result.servo_val_shoulder);
            // ESP_LOGI(TAG_SERVO, "%f and %f", servo1_angle, servo2_angle);
            // ESP_LOGI(TAG_SERVO, "recv angles: %f and %f", rx_packet.angle_shoulder, rx_packet.angle_elbow);
        } else {
            cnt++;
        }

        // drive servos with coordinated speeds (only update speed if changed)
        // if (ik_result.servo_speed_shoulder != last_speed_shoulder) {
        //     dynamixel_set_speed(1, ik_result.servo_speed_shoulder);
        //     last_speed_shoulder = ik_result.servo_speed_shoulder;
        // }
        // if (ik_result.servo_speed_elbow != last_speed_elbow) {
        //     dynamixel_set_speed(2, ik_result.servo_speed_elbow);
        //     last_speed_elbow = ik_result.servo_speed_elbow;
        // }

        // Always update positions
        // dynamixel_set_position(1, ik_result.servo_val_shoulder);
        // dynamixel_set_position(2, ik_result.servo_val_elbow);
        dynamixel_set_position(3, end_effector_pos);

        // angle pos
        int elbow_pos = (int) (rx_packet.angle_elbow * (1023.0f/360.0f))+SERVO_MIDPOINT;
        int shoulder_pos = (int) (rx_packet.angle_shoulder * (1023.0f/360.0f))+SERVO_MIDPOINT;
        dynamixel_set_position(1, shoulder_pos);
        dynamixel_set_position(2, elbow_pos);
        ESP_LOGI(TAG_SERVO, "%d and %d", shoulder_pos, elbow_pos);

        // Update previous positions for next iteration
        // prev_shoulder = ik_result.servo_val_shoulder;
        // prev_elbow = ik_result.servo_val_elbow;

        // Wait for next control cycle (50Hz timing)
        // vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief Application entry point
 *
 * Initializes the robotic arm output system in the following order:
 * 1. Configure RS485 half-duplex UART for Dynamixel communication
 * 2. Initialize and enable torque on all servos
 * 3. Create packet queue for receiving position data
 * 4. Initialize NVS (non-volatile storage)
 * 5. Start WiFi access point (192.168.4.1)
 * 6. Launch servo control task (50Hz)
 * 7. Launch TCP server task for receiving position commands
 *
 * The system acts as a WiFi AP that receives position packets from the
 * input system and translates them into coordinated servo movements.
 */
void app_main(void)
{
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_PIN, RX_PIN, RTS_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG_SERVO, "System Started. Enabling Torque...");

    dynamixel_init_servos();

    // configure_led();

    packet_queue = xQueueCreate(PACKET_BUFFER_SIZE, sizeof(packet_t));

    // initialize nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // start access point
    wifi_init_softap();

    // start servo thread
    xTaskCreate(servo_task, "servo_task", 4096, NULL, 10, NULL);

    // start tcp server thread
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG_MAIN, "System Initialization Complete.");
}