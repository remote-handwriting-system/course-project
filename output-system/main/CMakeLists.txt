idf_component_register(SRCS "main.c"
                            "wifi_ap.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_wifi esp_event nvs_flash esp_netif lwip driver esp_timer log freertos)