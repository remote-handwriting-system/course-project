idf_component_register(SRCS "main.c"
                            "double-angles-reader.c"
                            "2d-pos-encoder.c"
                            "data-sender.c"
                            "wifi_sta.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_wifi esp_event nvs_flash esp_netif lwip driver esp_timer log freertos)
