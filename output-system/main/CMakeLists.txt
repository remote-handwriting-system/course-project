idf_component_register(SRCS "actuator_math.c"
                            "force-reader.c"
                            "output-main.c"
                            "wifi_ap.c"
                            "tcp_server.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_adc led_strip esp_wifi esp_event nvs_flash esp_netif lwip driver esp_timer log freertos)