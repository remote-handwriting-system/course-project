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


// macros
#define LED_PIN 8 
#define STRIP_GPIO (8)
#define PACKET_BUFFER_SIZE (5)

#define TX_PIN       (2)         
#define RX_PIN       (3)         
#define RTS_PIN      (18)        

#define UART_PORT    (UART_NUM_1) 
#define BAUD_RATE    (57600)      
#define BUF_SIZE     (256)

#define INST_WRITE          (0x03)  
#define ADDR_TORQUE_ENABLE  (0x18)  
#define ADDR_GOAL_POSITION  (0x1E)  
#define ADDR_MOVING_SPEED   (0x20)
#define ADDR_CW_COMPLIANCE_SLOPE  (0x1C)
#define ADDR_CCW_COMPLIANCE_SLOPE (0x1D)
#define COMPLIANCE_SLOPE (200)
#define SERVO_END_LIMIT 30

// log tags
static const char *TAG_MAIN = "MAIN";
static const char *TAG_SERVO = "SERVO";


// TODO: MOVE TO ../components
// TODO: sync time between sender and receiver (just calculate offset)
typedef struct __attribute__((packed)) {
    float x_pos;
    float y_pos;
    int8_t elbow_sign;
    uint16_t force;
    uint32_t timestamp; 
} packet_t;


// global vars
led_strip_handle_t led_strip;
QueueHandle_t servo_queue;
// TODO MOVE
static float arm1_len = 72.0f;
static float arm2_len = 72.0f;

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

    // Read response to clear buffer and check for errors
    uint8_t response[64];
    int len = uart_read_bytes(UART_PORT, response, 64, pdMS_TO_TICKS(20)); // Short timeout

    if (len >= 6 && response[0] == 0xFF && response[1] == 0xFF) {
        if (response[4] != 0) {
            ESP_LOGW(TAG_SERVO, "Servo ID %d reported Error: 0x%02X", response[2], response[4]);
        }
    }
}

void set_compliance_slope(uint8_t id, uint8_t slope) {
    // Slope range: 0 (Stiff) to 254 (Very Soft)
    // Default is usually 32. Try 64 or 128 for smoother stops.
    
    // Set CW Slope
    uint8_t packet1[8];
    packet1[0] = 0xFF; packet1[1] = 0xFF; packet1[2] = id;
    packet1[3] = 0x04; packet1[4] = INST_WRITE;
    packet1[5] = ADDR_CW_COMPLIANCE_SLOPE;
    packet1[6] = slope;
    packet1[7] = calculate_checksum(packet1);
    send_packet_and_check(packet1, 8);

    // Set CCW Slope
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
    packet[3] = 0x04; // Length
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
    packet[3] = 0x05; // Length
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
    packet[3] = 0x05; // Length
    packet[4] = INST_WRITE;
    packet[5] = ADDR_MOVING_SPEED; // Address 32 (0x20)
    packet[6] = sL; 
    packet[7] = sH;
    packet[8] = calculate_checksum(packet);
    send_packet_and_check(packet, 9);
}

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


void servo_task(void *pvParameters) {

    // Persistent State Variables (These keep their value between loop cycles)
    double current_x = arm1_len+arm2_len;
    double current_y = 0.0f;
    int8_t  current_elbow_sign = 1;
    
    // Gripper State
    uint16_t target_force = 0;     // From network
    uint16_t current_force = 0;    // CHANGED: float -> uint32_t
    float gripper_pos = 512.0f;    // Current servo position (float for smooth math)
    
    // Control Parameters
    const float PROP_GAIN = 0.5f;         // Proportional Gain (Tune this! High = fast/jittery, Low = slow/smooth)
    const int FORCE_DEADBAND = 50; // Ignore small force differences
    const int GRIPPER_MIN = 512 - SERVO_END_LIMIT;
    const int GRIPPER_MAX = 512 + SERVO_END_LIMIT;

    packet_t rx_packet = {0};

    force_reader_init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // Run at 50Hz (20ms)
    while (1) {
        // check for network packet
        if (xQueueReceive(servo_queue, &rx_packet, 0) == pdPASS) {
            ESP_LOGI(TAG_SERVO, "packet received");
            current_x = rx_packet.x_pos;
            current_y = rx_packet.y_pos;
            current_elbow_sign = rx_packet.elbow_sign;
            target_force = rx_packet.force;
        }

        // inverse kinematics
        float cos_theta2 = (pow(current_x, 2.0f) + pow(current_y, 2.0f) - 
                           pow((double) arm1_len, 2.0f) -
                           pow((double) arm2_len, 2.0f)) /
                           (2.0f * arm1_len * arm2_len);

        if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
        if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

        float theta2_rad = (current_elbow_sign > 0) ? acosf(cos_theta2) : -acosf(cos_theta2);
        int servo_val_elbow = 512 + (int)(theta2_rad * 195.57f);

        float k1 = arm1_len + arm2_len * cosf(theta2_rad);
        float k2 = arm2_len * sinf(theta2_rad);
        float theta1_rad = atan2f(current_y, current_x) - atan2f(k2, k1);
        int servo_val_shoulder = 512 + (int)(theta1_rad * 195.57f);

        if (servo_val_elbow < 0) servo_val_elbow = 0;
        if (servo_val_elbow > 1023) servo_val_elbow = 1023;
        if (servo_val_shoulder < 0) servo_val_shoulder = 0;
        if (servo_val_shoulder > 1023) servo_val_shoulder = 1023;

        // force feedback loop
        force_reader_read_raw(&current_force); 
        
        ESP_LOGI(TAG_SERVO, "received force: %d", target_force);
        ESP_LOGI(TAG_SERVO, "sensed force:   %d", current_force);

        float error = target_force - (float)current_force;

        if (fabs(error) > FORCE_DEADBAND) {
            gripper_pos += (error * PROP_GAIN);
        }
        if (gripper_pos > GRIPPER_MAX) gripper_pos = GRIPPER_MAX;
        if (gripper_pos < GRIPPER_MIN) gripper_pos = GRIPPER_MIN;

        // drive servos
        dynamixel_set_position(1, servo_val_shoulder);
        dynamixel_set_position(2, servo_val_elbow);
        dynamixel_set_position(3, (int)gripper_pos);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main(void)
{
    // 1. UART Init
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_PIN, RX_PIN, RTS_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG_SERVO, "System Started. Enabling Torque...");

    // init servos2
    set_compliance_slope(1, COMPLIANCE_SLOPE);
    vTaskDelay(pdMS_TO_TICKS(50)); 
    set_compliance_slope(2, COMPLIANCE_SLOPE);
    vTaskDelay(pdMS_TO_TICKS(50)); 
    set_compliance_slope(2, COMPLIANCE_SLOPE);


    configure_led();

    servo_queue = xQueueCreate(PACKET_BUFFER_SIZE, sizeof(packet_t));

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