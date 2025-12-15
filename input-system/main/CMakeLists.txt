idf_component_register(SRCS "force-reader.c" "tcp_client.c" "input-main.c"
                            "double-angles-reader.c"
                            "2d-pos-encoder.c"
                            "wifi_sta.c"
                            "tcp_client.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_adc esp_wifi esp_event nvs_flash esp_netif lwip driver esp_timer log freertos)