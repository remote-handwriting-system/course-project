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


// macros
#define LED_PIN 8 
#define STRIP_GPIO (8)
#define PACKET_BUFFER_SIZE (1)

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
#define ADDR_CW_COMPLIANCE_SLOPE  (0x1C) // 28
#define ADDR_CCW_COMPLIANCE_SLOPE (0x1D) // 29

// log tags
static const char *TAG_MAIN = "MAIN";
static const char *TAG_SERVO = "SERVO";


// TODO: MOVE TO ../components
// TODO: sync time between sender and receiver (just calculate offset)
typedef struct __attribute__((packed)) {
    float x_pos;
    float y_pos;
    int8_t elbow_sign;
    float force;
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

// This task waits for angle data and updates the LED color
void servo_task(void *pvParameters) {
    led_strip_handle_t current_led_strip;
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = 1, 
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, 
        .flags.with_dma = false,
    };
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &current_led_strip));
    led_strip_clear(current_led_strip);

    ESP_LOGI(TAG_SERVO, "Color LED Ready on GPIO %d. Waiting for angles...", LED_PIN);

    packet_t rx_packet = {0};
    uint8_t r = 0, g = 0, b = 0;
    const uint8_t brightness = 50;
    BaseType_t result;
    bool packet_received = false;

    while (1) {
        
        // drain queue
        packet_received = false;
        do {
            // Check for a packet immediately (0 delay)
            result = xQueueReceive(servo_queue, &rx_packet, 0); 
            
            if (result == pdPASS) {
                packet_received = true;
            }
        } while (result == pdPASS); // loop until the queue is empty

        // process data
        if (packet_received) { 
            float x_pos = rx_packet.x_pos;
            float y_pos = rx_packet.y_pos;

            // 1. Math Setup
            float r_sq = x_pos * x_pos + y_pos * y_pos;

            // 2. Inverse Kinematics (Law of Cosines)
            float cos_theta2 = (r_sq - arm1_len * arm1_len - arm2_len * arm2_len) / (2.0f * arm1_len * arm2_len);

            // Safety: Clamp value to [-1, 1] to prevent NaN errors if target is slightly out of reach
            if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
            if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

            // Calculate Angles in Radians
            // Note: This assumes "Elbow Down" configuration. For Elbow Up, make theta2 negative.
            if (rx_packet.elbow_sign > 0) {
                float theta2_rad = acosf(cos_theta2);
            } else {
                float theta2_rad = -acosf(cos_theta2);
            }
            
            float k1 = arm1_len + arm2_len * cosf(theta2_rad);
            float k2 = arm2_len * sinf(theta2_rad);
            float theta1_rad = atan2f(y_pos, x_pos) - atan2f(k2, k1);

            // 3. Convert Radians to Dynamixel Units (0-1023)
            // RX-24F Range: 0 to 300 degrees. Center (512) is 150 degrees.
            // Factor: 1023 units / 300 degrees = ~3.41 units per degree
            // Radians to Degrees: * 57.2957795
            
            // Convert to degrees
            float theta1_deg = theta1_rad * (180.0f / M_PI);
            float theta2_deg = theta2_rad * (180.0f / M_PI);

            // Map to Servo Values (assuming 0 rad = Center/512)
            // Note: You might need to change the +/- depending on your motor mounting direction!
            int servo_val_1 = 512 + (int)(theta1_deg * 3.41f);
            int servo_val_2 = 512 + (int)(theta2_deg * 3.41f);

            // 4. Send to Servos (Constraints check included)
            if (servo_val_1 < 0) servo_val_1 = 0;
            if (servo_val_1 > 1023) servo_val_1 = 1023;
            if (servo_val_2 < 0) servo_val_2 = 0;
            if (servo_val_2 > 1023) servo_val_2 = 1023;

            // Send the commands
            dynamixel_set_position(1, servo_val_1); // ID 1 = Shoulder
            dynamixel_set_position(2, servo_val_2); // ID 2 = Elbow
        }

        vTaskDelay(pdMS_TO_TICKS(20)); 
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

    set_compliance_slope(1, 180);
    vTaskDelay(pdMS_TO_TICKS(50)); 
    set_compliance_slope(2, 180);
    vTaskDelay(pdMS_TO_TICKS(50)); 
    set_compliance_slope(2, 180);


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











// WIGGLE ARM
// --- Wiggle Helper ---
// Moves a servo to Center (512) -> +30 -> -30 -> Center
// void wiggle_servo(uint8_t id) {
//     ESP_LOGI(TAG, "Wiggling Servo ID: %d", id);
    
//     // 1. Move slightly Right (550)
//     set_position(id, 550);
//     vTaskDelay(pdMS_TO_TICKS(250)); // Fast wiggle

//     // 2. Move slightly Left (474)
//     set_position(id, 474);
//     vTaskDelay(pdMS_TO_TICKS(250));

//     // 3. Return to Center (512)
//     set_position(id, 512);
//     vTaskDelay(pdMS_TO_TICKS(250));
// }

// void app_main(void)
// {
//     // 1. UART Init
//     uart_config_t uart_config = {
//         .baud_rate = BAUD_RATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT,
//     };
//     ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
//     ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
//     ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_PIN, RX_PIN, RTS_PIN, UART_PIN_NO_CHANGE));
//     ESP_ERROR_CHECK(uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

//     ESP_LOGI(TAG, "System Started. Enabling Torque...");
//     vTaskDelay(pdMS_TO_TICKS(1000));

//     // 2. Enable Torque for all 3 servos
//     // (We do this once at the start)
//     set_torque(1, 1);
//     vTaskDelay(pdMS_TO_TICKS(50));
//     set_torque(2, 1);
//     vTaskDelay(pdMS_TO_TICKS(50));
//     set_torque(3, 1);
//     vTaskDelay(pdMS_TO_TICKS(50));

//     // 3. Main Sequence Loop
//     while (1) {
//         wiggle_servo(1);
//         vTaskDelay(pdMS_TO_TICKS(500));

//         wiggle_servo(2);
//         vTaskDelay(pdMS_TO_TICKS(500)); 

//         wiggle_servo(3);
//         vTaskDelay(pdMS_TO_TICKS(1000)); // Longer pause before restarting sequence
        
//         ESP_LOGI(TAG, "Sequence Complete. Repeating...");
//     }
// }