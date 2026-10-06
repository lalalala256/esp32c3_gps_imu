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
#include <stdlib.h>
#include <string.h>

#include "app_config.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
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

/* Name a WIFI_EVENT so the log says what happened rather than just a number. */
static const char *wifi_event_name(int32_t id)
{
    switch (id) {
    case WIFI_EVENT_WIFI_READY:          return "WIFI_READY";
    case WIFI_EVENT_SCAN_DONE:           return "SCAN_DONE";
    case WIFI_EVENT_STA_START:           return "STA_START";
    case WIFI_EVENT_STA_STOP:            return "STA_STOP";
    case WIFI_EVENT_STA_CONNECTED:       return "STA_CONNECTED";
    case WIFI_EVENT_STA_DISCONNECTED:    return "STA_DISCONNECTED";
    case WIFI_EVENT_AP_START:            return "AP_START";
    case WIFI_EVENT_AP_STOP:             return "AP_STOP";
    case WIFI_EVENT_AP_STACONNECTED:     return "AP_STACONNECTED";
    case WIFI_EVENT_AP_STADISCONNECTED:  return "AP_STADISCONNECTED";
    case WIFI_EVENT_AP_PROBEREQRECVED:   return "AP_PROBEREQRECVED";
    default:                             return "UNKNOWN";
    }
}

/* Reason a station association failed. NO_AP_FOUND is the one that matters: it
 * means nothing on the air answered our probes. */
static const char *disconnect_reason_name(uint8_t r)
{
    switch (r) {
    case WIFI_REASON_NO_AP_FOUND:
        return "NO_AP_FOUND (nothing answered our transmissions)";
    case WIFI_REASON_AUTH_FAIL:
        return "AUTH_FAIL (the far end answered, so transmit works)";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        return "HANDSHAKE_TIMEOUT (answered, but keys did not match)";
    case WIFI_REASON_ASSOC_FAIL:
        return "ASSOC_FAIL";
    case WIFI_REASON_BEACON_TIMEOUT:
        return "BEACON_TIMEOUT";
    default:
        return "other";
    }
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    /*
     * Log every event by name. esp_wifi_start() returning ESP_OK only means the
     * driver accepted the request; WIFI_EVENT_AP_START is what actually proves the
     * AP is up and beaconing, and it was not being handled at all.
     */
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_AP_PROBEREQRECVED) {
            /*
             * Log every one, immediately. A phone scanning for networks sends a
             * probe request on each channel it visits, so seeing these proves the
             * phone is talking to us and we are hearing it - which is what
             * separates "our beacons are not radiating" from "the phone never
             * looked". Losing a few in a flood is fine; missing them is not, so
             * this one is deliberately not rate limited.
             */
            const wifi_event_ap_probe_req_rx_t *e = data;
            static uint32_t s_probes;
            s_probes++;
            ESP_LOGW(TAG, ">>> PROBE REQUEST #%u from " MACSTR " rssi=%d dBm",
                     (unsigned)s_probes, MAC2STR(e->mac), (int)e->rssi);
        } else {
            ESP_LOGI(TAG, "wifi event: %s (%d)", wifi_event_name(id), (int)id);
        }
    }

    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "client " MACSTR " connected (aid %u)", MAC2STR(e->mac),
                 (unsigned)e->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "client " MACSTR " disconnected (aid %u)", MAC2STR(e->mac),
                 (unsigned)e->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "STA associated: the transmitter works, so the SoftAP failure is "
                 "on the client side");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *e = data;
        ESP_LOGW(TAG, "STA association failed: reason %u = %s",
                 (unsigned)e->reason, disconnect_reason_name(e->reason));
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

#if (APP_WIFI_SCAN_AT_BOOT || APP_WIFI_STA_TEST)
    /* The STA interface exists only so the RF self-check can scan, and so the
     * optional STA join test has an interface. Both are driver operations on
     * WIFI_IF_STA, and the AP and STA share the one radio. */
    if (!esp_netif_create_default_wifi_sta()) {
        ESP_LOGW(TAG, "no STA interface; the RF self-check will not run");
    }
#endif

    const wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Credentials are compile-time constants, so there is nothing to persist;
     * keeping the config in RAM avoids flash wear. */
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    /*
     * Unmask every WiFi event.
     *
     * The driver's default event mask hides WIFI_EVENT_AP_PROBEREQRECVED, so
     * without this the "is any client talking to us?" check never fires at all -
     * which looks exactly like an AP that hears nothing, and sends you hunting a
     * radio fault instead of a missing line of code.
     */
    err = esp_wifi_set_event_mask(WIFI_EVENT_MASK_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_event_mask failed: %s", esp_err_to_name(err));
    }

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

    err = esp_wifi_set_mode((APP_WIFI_SCAN_AT_BOOT || APP_WIFI_STA_TEST)
                                ? WIFI_MODE_APSTA : WIFI_MODE_AP);
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
     * Setting the TX power goes through a PHY power-table recalibration. If that
     * recalibration goes wrong the call still returns ESP_OK and the value reads
     * back correctly, while the radio transmits at a negligible level - which is
     * indistinguishable from a broken power amplifier from the outside.
     *
     * The symptom that made this suspicious: the AP starts, hears probe requests
     * from phones, and yet no client can see its beacons. A radio link is
     * symmetric, so if a phone's probe arrives at -60 dBm then a +20 dBm beacon
     * should reach that phone at around -55 dBm, which every phone lists. If no
     * client sees it, the beacon is not actually going out.
     *
     * The driver default is the country limit anyway (20 dBm here), so leaving
     * the call out loses nothing. Flip this back to 1 to restore it.
     */
#define APP_WIFI_FORCE_TX_POWER 0

#if APP_WIFI_FORCE_TX_POWER
    const int8_t tx_units = (int8_t)(APP_WIFI_TX_POWER_DBM * 4);
    err = esp_wifi_set_max_tx_power(tx_units);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_max_tx_power(%d) failed: %s",
                 (int)tx_units, esp_err_to_name(err));
    }
#endif

    int8_t tx_now = 0;
    if (esp_wifi_get_max_tx_power(&tx_now) == ESP_OK) {
        ESP_LOGW(TAG, "TX power in use: %.2f dBm (forced=%d)",
                 tx_now / 4.0, APP_WIFI_FORCE_TX_POWER);
    }

    /*
     * NOTE: esp_wifi_set_bandwidth() is deliberately NOT called.
     *
     * Setting the AP bandwidth after esp_wifi_start() makes the driver stop and
     * restart the AP - AP_STOP and then a second AP_START are both logged - and
     * after that restart no client could see the AP, even though AP_START had
     * fired and the configuration read back correctly.
     *
     * The same board's radio is proven good (an Arduino sketch on this very
     * board associates with an AP), and this project's WiFi/PHY configuration is
     * byte-for-byte the IDF default, so every extra step is being removed to
     * match the known-good bring-up exactly: set_mode, set_config, start.
     *
     * HT40 in the 2.4 GHz band is still undesirable, so revisit this once the AP
     * is actually visible. Set to 1 to restore the call.
     */
#define APP_WIFI_SET_AP_BANDWIDTH 0

#if APP_WIFI_SET_AP_BANDWIDTH
    err = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_bandwidth(WIFI_BW_HT20) failed: %s",
                 esp_err_to_name(err));
    }
#endif

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(s_netif, &ip) == ESP_OK && ip.ip.addr != 0) {
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ip.ip));
    }

#if APP_WIFI_SCAN_AT_BOOT
    /*
     * RF self-check.
     *
     * Hosting an AP can look healthy from the inside - AP_START fires and the
     * configuration reads back exactly as requested - while nothing is actually
     * radiating, because the antenna path is broken. Scanning answers that, and
     * also shows whether this board can see its own beacon.
     */
    wifi_scan_config_t scan = { 0 };
    scan.show_hidden = true;
    scan.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan.scan_time.active.min = 100;
    scan.scan_time.active.max = 300;

    esp_err_t serr = esp_wifi_scan_start(&scan, true);
    if (serr != ESP_OK) {
        ESP_LOGW(TAG, "RF self-check scan failed: %s", esp_err_to_name(serr));
    } else {
        uint16_t found = 0;
        esp_wifi_scan_get_ap_num(&found);
        ESP_LOGW(TAG, "RF self-check: %u network(s) visible from this board",
                 (unsigned)found);

        wifi_ap_record_t *recs = calloc(found ? found : 1, sizeof(*recs));
        if (recs) {
            uint16_t n = found;
            if (n && esp_wifi_scan_get_ap_records(&n, recs) == ESP_OK) {
                bool self = false;
                for (uint16_t i = 0; i < n; i++) {
                    /* Debug level: the list itself is only interesting when the
                     * summary above looks wrong. */
                    ESP_LOGD(TAG, "  %2u: \"%s\" rssi=%d ch=%u auth=%d",
                             (unsigned)i, (const char *)recs[i].ssid,
                             (int)recs[i].rssi, (unsigned)recs[i].primary,
                             (int)recs[i].authmode);
                    if (strcmp((const char *)recs[i].ssid, AP_SSID) == 0) {
                        self = true;
                    }
                }
                ESP_LOGW(TAG, "  our own SSID \"%s\" %s in that scan", AP_SSID,
                         self ? "APPEARS" : "does NOT appear");
            }
            free(recs);
        }
    }
#endif

    /*
     * Read the configuration back out of the driver. "We asked for X" and "the
     * driver holds X" are different things, and this project has already been
     * caught twice by assuming the first meant the second.
     */
    wifi_config_t got = { 0 };
    if (esp_wifi_get_config(WIFI_IF_AP, &got) == ESP_OK) {
        ESP_LOGI(TAG, "driver holds: ssid=\"%s\" len=%u authmode=%d channel=%u "
                 "hidden=%u beacon_interval=%u",
                 (const char *)got.ap.ssid, (unsigned)got.ap.ssid_len,
                 (int)got.ap.authmode, (unsigned)got.ap.channel,
                 (unsigned)got.ap.ssid_hidden, (unsigned)got.ap.beacon_interval);
    } else {
        ESP_LOGE(TAG, "could not read the AP configuration back");
    }

    uint8_t channel = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    if (esp_wifi_get_channel(&channel, &second) == ESP_OK) {
        ESP_LOGI(TAG, "actual radio channel: %u (second %d)",
                 (unsigned)channel, (int)second);
    }

    wifi_bandwidth_t bw = 0;
    if (esp_wifi_get_bandwidth(WIFI_IF_AP, &bw) == ESP_OK) {
        ESP_LOGI(TAG, "actual bandwidth: %s",
                 bw == WIFI_BW_HT20 ? "HT20 (20 MHz)" : "HT40 (40 MHz)");
    }

    wifi_country_t country = { 0 };
    if (esp_wifi_get_country(&country) == ESP_OK) {
        ESP_LOGI(TAG, "regulatory: cc=\"%s\" channels %u-%u, max TX %d dBm",
                 country.cc, (unsigned)country.schan,
                 (unsigned)(country.schan + country.nchan - 1),
                 (int)country.max_tx_power);
    }

    s_up = true;
    ESP_LOGI(TAG, "SoftAP \"%s\" up (%s, channel %d, max %d clients)",
             AP_SSID,
             ap.ap.authmode == WIFI_AUTH_OPEN ? "open" : "WPA2-PSK",
             (int)APP_WIFI_CHANNEL, (int)APP_WIFI_MAX_STA);
    ESP_LOGI(TAG, "browse to http://%s/ once connected", s_ip);

    ESP_LOGW(TAG, "watching for probe requests: scan for \"%s\" from a phone and a "
             ">>> PROBE REQUEST line appears here for each one we hear", AP_SSID);

#if APP_WIFI_STA_TEST
    /* Optional: join an existing network, purely to prove the transmitter works.
     * Receiving proves only the receiver, and on this board receiving works
     * perfectly while the SoftAP is invisible to every client. */
    wifi_config_t sta = { 0 };
    copy_str((char *)sta.sta.ssid, sizeof(sta.sta.ssid), APP_WIFI_STA_SSID);
    copy_str((char *)sta.sta.password, sizeof(sta.sta.password),
             APP_WIFI_STA_PASSWORD);
    sta.sta.threshold.authmode = WIFI_AUTH_OPEN;

    err = esp_wifi_set_config(WIFI_IF_STA, &sta);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "STA test: esp_wifi_set_config failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGW(TAG, "STA test: joining \"%s\"", APP_WIFI_STA_SSID);
        err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "STA test: esp_wifi_connect failed: %s", esp_err_to_name(err));
        }
    }
#endif

    return ESP_OK;
}

bool wifi_ap_is_up(void)
{
    return s_up;
}

void wifi_ap_stop(void)
{
    if (!s_up) {
        return;
    }
    s_up = false;

    const esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_stop failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "SoftAP stopped for sleep");
}

const char *wifi_ap_ip_str(void)
{
    return s_ip;
}
