/*
 * wifi_ap.c - SoftAP bring-up.
 *
 * The ESP32 hosts its own network rather than joining one, because the logger
 * is meant to be usable in the field where there is no infrastructure. Connect
 * a phone or laptop to the AP and browse to the address printed at boot.
 *
 * NVS is required even though the credentials are compiled in: the WiFi driver
 * stores its PHY calibration data there.
 */
#include "wifi_ap.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "wifi";

#define AP_SSID     APP_WIFI_SSID
#define AP_PASSWORD APP_WIFI_PASSWORD
#define AP_PASSWORD_LEN (sizeof(AP_PASSWORD) - 1u)

static esp_netif_t  *s_netif;
static volatile bool s_up;
static char          s_ip[16] = "192.168.4.1";

/* Copy with guaranteed termination (avoids relying on strlcpy availability). */
static void copy_str(char *dst, size_t n, const char *src)
{
    if (n == 0) {
        return;
    }
    strncpy(dst, src, n - 1);
    dst[n - 1] = '\0';
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "client " MACSTR " connected (aid %u)", MAC2STR(e->mac),
                 (unsigned)e->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "client " MACSTR " disconnected (aid %u)", MAC2STR(e->mac),
                 (unsigned)e->aid);
    } else if (base == IP_EVENT && id == IP_EVENT_AP_STAIPASSIGNED) {
        const ip_event_ap_staipassigned_t *e = data;
        ESP_LOGI(TAG, "client assigned " IPSTR, IP2STR(&e->ip));
    }
}

static esp_err_t nvs_ensure_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erasing, retrying");
        const esp_err_t e = nvs_flash_erase();
        if (e != ESP_OK) {
            return e;
        }
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t wifi_ap_start(void)
{
    if (s_up) {
        return ESP_OK;
    }

    esp_err_t err = nvs_ensure_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return err;
    }

    s_netif = esp_netif_create_default_wifi_ap();
    if (!s_netif) {
        ESP_LOGE(TAG, "failed to create the default AP interface");
        return ESP_FAIL;
    }

    const wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Credentials are compile-time constants, so there is nothing to persist;
     * keeping the config in RAM avoids flash wear. */
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              on_wifi_event, NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WIFI_EVENT handler registration failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED,
                                              on_wifi_event, NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IP_EVENT handler registration failed: %s", esp_err_to_name(err));
        return err;
    }

    wifi_config_t ap = { 0 };
    ap.ap.channel = APP_WIFI_CHANNEL;
    ap.ap.max_connection = APP_WIFI_MAX_STA;
    ap.ap.beacon_interval = 100;
    ap.ap.ssid_hidden = 0;
    ap.ap.pmf_cfg.required = false;
    copy_str((char *)ap.ap.ssid, sizeof(ap.ap.ssid), AP_SSID);
    ap.ap.ssid_len = (uint8_t)strlen((char *)ap.ap.ssid);

    if (AP_PASSWORD_LEN == 0) {
        ap.ap.authmode = WIFI_AUTH_OPEN;
        ESP_LOGW(TAG, "no AP password configured: the network is OPEN");
    } else if (AP_PASSWORD_LEN < 8 || AP_PASSWORD_LEN > 63) {
        ap.ap.authmode = WIFI_AUTH_OPEN;
        ESP_LOGE(TAG, "AP password must be 8..63 characters (or empty); got %u. "
                 "Falling back to an OPEN network.",
                 (unsigned)AP_PASSWORD_LEN);
    } else {
        ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
        copy_str((char *)ap.ap.password, sizeof(ap.ap.password), AP_PASSWORD);
    }

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &ap);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return err;
    }

    /*
     * Raise the TX power explicitly. The API takes 0.25 dBm units, and the
     * SuperMini's small ceramic antenna makes range the limiting factor, so a
     * conservative default is worth overriding here.
     */
    const int8_t tx_units = (int8_t)(APP_WIFI_TX_POWER_DBM * 4);
    err = esp_wifi_set_max_tx_power(tx_units);
    if (err == ESP_OK) {
        int8_t got = 0;
        if (esp_wifi_get_max_tx_power(&got) == ESP_OK) {
            ESP_LOGI(TAG, "max TX power: %.2f dBm", got / 4.0);
        }
    } else {
        ESP_LOGW(TAG, "esp_wifi_set_max_tx_power(%d) failed: %s",
                 (int)tx_units, esp_err_to_name(err));
    }

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(s_netif, &ip) == ESP_OK && ip.ip.addr != 0) {
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ip.ip));
    }

    s_up = true;
    ESP_LOGI(TAG, "SoftAP \"%s\" up (%s, channel %d, max %d clients)",
             AP_SSID,
             ap.ap.authmode == WIFI_AUTH_OPEN ? "open" : "WPA2-PSK",
             (int)APP_WIFI_CHANNEL, (int)APP_WIFI_MAX_STA);
    ESP_LOGI(TAG, "browse to http://%s/ once connected", s_ip);
    return ESP_OK;
}

bool wifi_ap_is_up(void)
{
    return s_up;
}

const char *wifi_ap_ip_str(void)
{
    return s_ip;
}
