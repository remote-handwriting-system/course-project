#ifndef WIFI_AP_H
#define WIFI_AP_H

/**
 * @brief Initialize the ESP32 as a SoftAP (Access Point)
 * * Creates a WiFi network that the Transmitter can connect to.
 * SSID and Password are defined in wifi_ap.c
 */
void wifi_init_softap(void);

#endif // WIFI_AP_H