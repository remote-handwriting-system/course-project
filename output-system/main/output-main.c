#include <stdio.h>
#include <math.h>
#include <string.h>

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


// macros
#define LED_PIN                   8 
#define STRIP_GPIO                8
#define PACKET_BUFFER_SIZE        5

#define TX_PIN                    2
#define RX_PIN                    3
#define RTS_PIN                   18

#define UART_PORT                 UART_NUM_1
#define BAUD_RATE                 57600
#define BUF_SIZE                  256

#define INST_WRITE                0x03  
#define ADDR_TORQUE_ENABLE        0x18  
#define ADDR_GOAL_POSITION        0x1E  
#define ADDR_MOVING_SPEED         0x20
#define ADDR_CW_COMPLIANCE_SLOPE  0x1C
#define ADDR_CCW_COMPLIANCE_SLOPE 0x1D
#define COMPLIANCE_SLOPE          200
#define SERVO_MIDPOINT            512
#define END_SERVO_LIMIT           576 // physical max is 592
#define ARM1_LEN                  72.0f
#define ARM2_LEN                  101.4f
#define LOW_PASS_X_ALPHA          0.3f // The larger, the more influence the new datapoint has on the x-axis movement
#define LOW_PASS_Y_ALPHA          0.3f // The larger, the more influence the new datapoint has on the y-axis movement

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

uint8_t calculate_checksum(uint8_t *packet) {
    uint8_t checksum = 0;
    int length_val = packet[3];
    int limit = 3 + length_val - 1; 
    for (int i = 2; i <= limit; i++) {
        checksum += packet[i];
    }
    return ~checksum;
}

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

void set_compliance_slope(uint8_t id, uint8_t slope) {
    // set CW Slope
    uint8_t packet1[8];
    packet1[0] = 0xFF; packet1[1] = 0xFF; packet1[2] = id;
    packet1[3] = 0x04; packet1[4] = INST_WRITE;
    packet1[5] = ADDR_CW_COMPLIANCE_SLOPE;
    packet1[6] = slope;
    packet1[7] = calculate_checksum(packet1);
    send_packet_and_check(packet1, 8);

    // set CCW Slope
    uint8_t packet2[8];
    packet2[0] = 0xFF; packet2[1] = 0xFF; packet2[2] = id;
    packet2[3] = 0x04; packet2[4] = INST_WRITE;
    packet2[5] = ADDR_CCW_COMPLIANCE_SLOPE;
    packet2[6] = slope;
    packet2[7] = calculate_checksum(packet2);
    send_packet_and_check(packet2, 8);
}

void set_torque(uint8_t id, uint8_t enable) {
    uint8_t packet[8];
    packet[0] = 0xFF; packet[1] = 0xFF; packet[2] = id;
    packet[3] = 0x04;
    packet[4] = INST_WRITE;
    packet[5] = ADDR_TORQUE_ENABLE; 
    packet[6] = enable; 
    packet[7] = calculate_checksum(packet);
    send_packet_and_check(packet, 8);
}

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

void init_servos() {
    set_torque(1, 1); vTaskDelay(pdMS_TO_TICKS(10)); 
    set_torque(2, 1); vTaskDelay(pdMS_TO_TICKS(10)); 
    set_torque(3, 1); vTaskDelay(pdMS_TO_TICKS(10));
    dynamixel_set_speed(1, 400), vTaskDelay(pdMS_TO_TICKS(10));
    dynamixel_set_speed(2, 400), vTaskDelay(pdMS_TO_TICKS(10));
    // dynamixel_set_position(3, 400), vTaskDelay(pdMS_TO_TICKS(10));
    set_compliance_slope(1, COMPLIANCE_SLOPE); vTaskDelay(pdMS_TO_TICKS(10)); 
    set_compliance_slope(2, COMPLIANCE_SLOPE); vTaskDelay(pdMS_TO_TICKS(10)); 
    set_compliance_slope(3, COMPLIANCE_SLOPE); // TODO determine this

}

void servo_task(void *pvParameters) {
    float current_x = ARM1_LEN+ARM2_LEN;
    float current_y = 0.0f;
    int8_t  current_elbow_sign = 1;
    uint16_t target_force = 0;
    uint16_t current_force = 0;
    uint16_t end_effector_pos = SERVO_MIDPOINT;
    
    // control parameters
    const float PROP_GAIN = 0.05f; // higher -> more jitter
    const int FORCE_DEADBAND = 5;

    packet_t rx_packet = {0};

    force_reader_init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // Run at 50Hz (20ms)
    //placeholder
    uint64_t cnt = 0;

    // Initialize velocity-based filter
    VelocityFilter vel_filter;
    velocity_filter_init(&vel_filter, 100.0f);  // Max speed: 100 mm/s - tune this!

    float filtered_pos_x = 0.0f;
    float filtered_pos_y = 0.0f;
    bool is_first_iteration = true;
    while (1) {
        // check for network packet
        if (xQueueReceive(packet_queue, &rx_packet, 0) == pdPASS) {
            // ESP_LOGI(TAG_SERVO, "packet received");
            current_x = rx_packet.pos_x;
            current_y = rx_packet.pos_y;
            current_elbow_sign = rx_packet.elbow_sign;
            target_force = rx_packet.force;
        }

        // // low pass filter on position
        // if (is_first_iteration) {
        //     filtered_pos_x = current_x;
        //     filtered_pos_y = current_y;
        //     is_first_iteration = false;
        // }
        // filtered_pos_x = low_pass_pos_filter(filtered_pos_x, current_x, LOW_PASS_X_ALPHA);
        // filtered_pos_y = low_pass_pos_filter(filtered_pos_y, current_y, LOW_PASS_Y_ALPHA);

        // // inverse kinematics
        // ik_result_t ik_result;
        // inverse_kinematics(filtered_pos_x, filtered_pos_y, current_elbow_sign, ARM1_LEN, ARM2_LEN, SERVO_MIDPOINT, &ik_result);

        // Apply velocity-based filtering
        velocity_filter_apply(&vel_filter, current_x, current_y, &filtered_pos_x, &filtered_pos_y);

        // inverse kinematics
        ik_result_t ik_result;
        inverse_kinematics(filtered_pos_x, filtered_pos_y, current_elbow_sign, ARM1_LEN, ARM2_LEN, SERVO_MIDPOINT, &ik_result);

        // force feedback
        force_reader_read_raw(&current_force);
        int16_t error = force_effect(target_force, current_force, &end_effector_pos, SERVO_MIDPOINT, END_SERVO_LIMIT, PROP_GAIN, FORCE_DEADBAND);

        // Printing
        if (cnt >= 10) {
            cnt = 0;
            ESP_LOGI(TAG_SERVO, "target force:   %d", target_force);
            ESP_LOGI(TAG_SERVO, "sensed force:   %d", current_force);
            ESP_LOGI(TAG_SERVO, "error:          %d", error);
            ESP_LOGI(TAG_SERVO, "end_eff_pos:    %d", end_effector_pos);
        } else {
            cnt++;
        }

        // drive servos
        dynamixel_set_position(1, ik_result.servo_val_shoulder);
        dynamixel_set_position(2, ik_result.servo_val_elbow);
        dynamixel_set_position(3, end_effector_pos);

        // dynamixel_set_position(3, 512);
        // ESP_LOGI(TAG_SERVO, "512");
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // dynamixel_set_position(3, 576);
        // ESP_LOGI(TAG_SERVO, "540");
        // vTaskDelay(pdMS_TO_TICKS(1000));

        // vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_PIN, RX_PIN, RTS_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG_SERVO, "System Started. Enabling Torque...");

    init_servos();

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