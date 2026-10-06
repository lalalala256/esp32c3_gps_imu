/*
 * wifi_ap.h - WiFi SoftAP for the web file browser.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

/*
 * Bring up a WPA2 SoftAP using the SSID/password/channel from menuconfig.
 * Handles NVS, esp_netif and the default event loop itself, so it is the only
 * WiFi entry point the application needs. Idempotent.
 */
esp_err_t wifi_ap_start(void);

/* True once the AP interface is running. */
bool wifi_ap_is_up(void);

/*
 * Stop the radio. Called on the way into deep sleep: the AP is unreachable
 * while the chip is asleep, and a stopped driver cannot take a turn on the SD
 * card's bus during the teardown. The netif and the event loop are left alone
 * because deep sleep resets the chip regardless.
 */
void wifi_ap_stop(void);

/* Dotted-quad address of the AP, or a fallback string when not yet known. */
const char *wifi_ap_ip_str(void);
