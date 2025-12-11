#include <stdio.h>
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
#define PACKET_BUFFER_SIZE (30)


// log tags
static const char *TAG_MAIN = "MAIN";
static const char *TAG_SERVO = "SERVO";


// TODO: MOVE TO ../components
// TODO: sync time between sender and receiver (just calculate offset)
typedef struct __attribute__((packed)) {
    float angle1_deg;
    float angle2_deg;
    float force;
    uint32_t timestamp_us; 
} packet_t;


// global vars
led_strip_handle_t led_strip;
QueueHandle_t servo_queue;


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
void servo_task(void *pvParameters)
{
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
            float
            // float angle = rx_packet.angle1_deg;
            // while(angle >= 360.0f) angle -= 360.0f;
            // while(angle < 0.0f) angle += 360.0f;
            // r = (uint8_t)(brightness * (angle)/ 360.0f);
            // g = (uint8_t)(brightness * (angle) / 360.0f);
            // b = 0;
            // ESP_ERROR_CHECK(led_strip_set_pixel(current_led_strip, 0, r, g, b));
            // ESP_ERROR_CHECK(led_strip_refresh(current_led_strip));
            // ESP_LOGI(TAG_SERVO, "Angle: %.1f -> RGB(%d, %d, %d)", angle, r, g, b);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

void app_main(void)
{
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






GET ID
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

// --- Hardware Pin Configuration ---
// CHANGED: Moved to GPIO 2/3 to avoid conflict with Console (GPIO 16/17)
#define TX_PIN       (2)         // Connect to SparkFun RX-I
#define RX_PIN       (3)         // Connect to SparkFun TX-O
#define RTS_PIN      (18)        // Connect to SparkFun RTS
#define CTS_PIN      (UART_PIN_NO_CHANGE) 

// --- UART Configuration ---
#define UART_PORT    (UART_NUM_1) // Use UART1 (UART0 is for console)
#define BAUD_RATE    (57600)      // Default RX-24F baud rate
#define BUF_SIZE     (256)

static const char *TAG = "DYNAMIXEL";

// Checksum for Protocol 1.0: ~(ID + Length + Instruction + Params...)
uint8_t calculate_checksum(uint8_t *packet) {
    uint8_t checksum = 0;
    int length_val = packet[3];
    int limit = 3 + length_val - 1; 
    
    for (int i = 2; i <= limit; i++) {
        checksum += packet[i];
    }
    return ~checksum;
}

void app_main(void)
{
    // 1. Configure UART
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 2. Install Driver on UART_NUM_1
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    
    // 3. Set Pins (using new GPIOs)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_PIN, RX_PIN, RTS_PIN, CTS_PIN));

    // 4. Set RS485 Half Duplex Mode
    ESP_ERROR_CHECK(uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG, "UART Initialized on GPIO 2(TX)/3(RX). Starting Ping...");

    while (1) {
        // --- Packet Construction ---
        // PING (0x01) to Broadcast ID (0xFE)
        uint8_t packet[6];
        packet[0] = 0xFF; 
        packet[1] = 0xFF; 
        packet[2] = 0xFE; 
        packet[3] = 0x02; // Length 
        packet[4] = 0x01; // Instruction: PING
        packet[5] = calculate_checksum(packet);

        // Clear RX buffer
        uart_flush_input(UART_PORT);

        // Send Packet
        uart_write_bytes(UART_PORT, (const char *)packet, 6);
        uart_wait_tx_done(UART_PORT, 10); // Wait for send to complete

        // Read Response
        uint8_t data[64];
        int len = uart_read_bytes(UART_PORT, data, 64, pdMS_TO_TICKS(100));

        if (len > 0) {
            // Check headers
            if (len >= 6 && data[0] == 0xFF && data[1] == 0xFF) {
                uint8_t found_id = data[2];
                uint8_t error_code = data[4];
                
                ESP_LOGI(TAG, "Response Received!");
                ESP_LOGI(TAG, "  > SERVO ID: %d (0x%02X)", found_id, found_id);
                
                if (error_code == 0) {
                    ESP_LOGI(TAG, "  > Status: OK");
                } else {
                    ESP_LOGW(TAG, "  > Status Error: 0x%02X", error_code);
                }
            } else {
                ESP_LOGW(TAG, "Garbage data received (Len: %d)", len);
                ESP_LOG_BUFFER_HEX(TAG, data, len);
            }
        } else {
            ESP_LOGE(TAG, "No response. Check wiring, power (12V), and baud rate.");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}






// WIGGLE SINGLE
// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/uart.h"
// #include "driver/gpio.h"
// #include "esp_log.h"

// // --- Hardware Pins ---
// #define TX_PIN       (2)         
// #define RX_PIN       (3)         
// #define RTS_PIN      (18)        

// // --- UART Config ---
// #define UART_PORT    (UART_NUM_1) 
// #define BAUD_RATE    (57600)      
// #define BUF_SIZE     (256)

// // --- Dynamixel Protocol 1.0 ---
// #define SERVO_ID            (1)     
// #define INST_WRITE          (0x03)  
// #define ADDR_TORQUE_ENABLE  (0x18)  
// #define ADDR_GOAL_POSITION  (0x1E)  

// static const char *TAG = "DXL_DEBUG";

// // Calculate Checksum
// uint8_t calculate_checksum(uint8_t *packet) {
//     uint8_t checksum = 0;
//     int length_val = packet[3];
//     int limit = 3 + length_val - 1; 
//     for (int i = 2; i <= limit; i++) {
//         checksum += packet[i];
//     }
//     return ~checksum;
// }

// // --- NEW: Send AND Read Response ---
// void send_packet_and_check_error(uint8_t *packet, int packet_len, const char *action_name) {
//     // 1. Clear buffer to ensure we only read fresh data
//     uart_flush_input(UART_PORT);

//     // 2. Send Command
//     uart_write_bytes(UART_PORT, (const char *)packet, packet_len);
//     uart_wait_tx_done(UART_PORT, 10); // Wait for physical send

//     // 3. Read Response (Dynamixel replies immediately)
//     uint8_t response[64];
//     // Give it 50ms to reply
//     int len = uart_read_bytes(UART_PORT, response, 64, pdMS_TO_TICKS(50));

//     if (len > 0) {
//         // Validate Header [0xFF, 0xFF]
//         if (len >= 6 && response[0] == 0xFF && response[1] == 0xFF) {
//             uint8_t error_byte = response[4];
            
//             if (error_byte == 0) {
//                 ESP_LOGI(TAG, "[%s] Success! (No Error)", action_name);
//             } else {
//                 ESP_LOGE(TAG, "[%s] FAILED! Error Byte: 0x%02X", action_name, error_byte);
//                 // Decode the error
//                 if (error_byte & 0x01) ESP_LOGE(TAG, "  -> Input Voltage Error (Check PSU Voltage!)");
//                 if (error_byte & 0x02) ESP_LOGE(TAG, "  -> Angle Limit Error");
//                 if (error_byte & 0x04) ESP_LOGE(TAG, "  -> Overheating Error");
//                 if (error_byte & 0x08) ESP_LOGE(TAG, "  -> Range Error");
//                 if (error_byte & 0x10) ESP_LOGE(TAG, "  -> Checksum Error");
//                 if (error_byte & 0x20) ESP_LOGE(TAG, "  -> Overload Error (Stall)");
//             }
//         } else {
//             ESP_LOGW(TAG, "[%s] Received junk data (len=%d)", action_name, len);
//             ESP_LOG_BUFFER_HEX(TAG, response, len);
//         }
//     } else {
//         ESP_LOGE(TAG, "[%s] Timeout - No Response from Servo", action_name);
//     }
// }

// void dynamixel_set_torque(uint8_t id, uint8_t enable) {
//     uint8_t packet[8];
//     packet[0] = 0xFF; packet[1] = 0xFF; packet[2] = id;
//     packet[3] = 0x04; // Length
//     packet[4] = INST_WRITE;
//     packet[5] = ADDR_TORQUE_ENABLE; 
//     packet[6] = enable; 
//     packet[7] = calculate_checksum(packet);

//     send_packet_and_check_error(packet, 8, "Set Torque");
// }

// void dynamixel_set_position(uint8_t id, uint16_t position) {
//     if (position > 1023) position = 1023;
//     uint8_t pL = position & 0xFF;
//     uint8_t pH = (position >> 8) & 0xFF;

//     uint8_t packet[9];
//     packet[0] = 0xFF; packet[1] = 0xFF; packet[2] = id;
//     packet[3] = 0x05; // Length
//     packet[4] = INST_WRITE;
//     packet[5] = ADDR_GOAL_POSITION;
//     packet[6] = pL; packet[7] = pH;
//     packet[8] = calculate_checksum(packet);

//     send_packet_and_check_error(packet, 9, "Set Position");
// }

// void app_main(void)
// {
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

//     ESP_LOGI(TAG, "System Initialized.");
//     vTaskDelay(pdMS_TO_TICKS(1000));

//     // Try to Enable Torque
//     dynamixel_set_torque(SERVO_ID, 1);
//     vTaskDelay(pdMS_TO_TICKS(500));

//     while (1) {
//         dynamixel_set_position(SERVO_ID, 400);
//         vTaskDelay(pdMS_TO_TICKS(2000)); 

//         dynamixel_set_position(SERVO_ID, 600);
//         vTaskDelay(pdMS_TO_TICKS(2000)); 
//     }
// }







// WIGGLE ARM
// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/uart.h"
// #include "driver/gpio.h"
// #include "esp_log.h"

// // --- Hardware Pins ---
// #define TX_PIN       (2)         
// #define RX_PIN       (3)         
// #define RTS_PIN      (18)        

// // --- UART Configuration ---
// #define UART_PORT    (UART_NUM_1) 
// #define BAUD_RATE    (57600)      
// #define BUF_SIZE     (256)

// // --- Protocol 1.0 Constants ---
// #define INST_WRITE          (0x03)  
// #define ADDR_TORQUE_ENABLE  (0x18)  
// #define ADDR_GOAL_POSITION  (0x1E)  

// static const char *TAG = "DXL_SEQ";

// // --- Checksum Helper ---
// uint8_t calculate_checksum(uint8_t *packet) {
//     uint8_t checksum = 0;
//     int length_val = packet[3];
//     int limit = 3 + length_val - 1; 
//     for (int i = 2; i <= limit; i++) {
//         checksum += packet[i];
//     }
//     return ~checksum;
// }

// // --- Send & Check Error Helper ---
// void send_packet_and_check(uint8_t *packet, int packet_len) {
//     uart_flush_input(UART_PORT);
//     uart_write_bytes(UART_PORT, (const char *)packet, packet_len);
//     uart_wait_tx_done(UART_PORT, 10);

//     // Read response to clear buffer and check for errors
//     uint8_t response[64];
//     int len = uart_read_bytes(UART_PORT, response, 64, pdMS_TO_TICKS(20)); // Short timeout

//     if (len >= 6 && response[0] == 0xFF && response[1] == 0xFF) {
//         if (response[4] != 0) {
//             ESP_LOGW(TAG, "Servo ID %d reported Error: 0x%02X", response[2], response[4]);
//         }
//     }
// }

// // --- Set Torque ---
// void set_torque(uint8_t id, uint8_t enable) {
//     uint8_t packet[8];
//     packet[0] = 0xFF; packet[1] = 0xFF; packet[2] = id;
//     packet[3] = 0x04; // Length
//     packet[4] = INST_WRITE;
//     packet[5] = ADDR_TORQUE_ENABLE; 
//     packet[6] = enable; 
//     packet[7] = calculate_checksum(packet);
//     send_packet_and_check(packet, 8);
// }

// // --- Set Position ---
// void set_position(uint8_t id, uint16_t position) {
//     if (position > 1023) position = 1023;
//     uint8_t pL = position & 0xFF;
//     uint8_t pH = (position >> 8) & 0xFF;

//     uint8_t packet[9];
//     packet[0] = 0xFF; packet[1] = 0xFF; packet[2] = id;
//     packet[3] = 0x05; // Length
//     packet[4] = INST_WRITE;
//     packet[5] = ADDR_GOAL_POSITION;
//     packet[6] = pL; packet[7] = pH;
//     packet[8] = calculate_checksum(packet);
//     send_packet_and_check(packet, 9);
// }

// // --- Wiggle Helper ---
// // Moves a servo to Center (512) -> +30 -> -30 -> Center
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