/*
 * status.c - prints one compact status line every APP_STATUS_PERIOD_S.
 *
 * The line is intended to be read by a human watching the console, and it is
 * deliberately the only place that reports drop counters: a non-zero drop is
 * the signal that a consumer (usually the SD card) could not keep up.
 */
#include "status.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "data_types.h"
#include "logger_sd.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "status";

static void format_utc(const gnss_sample_t *g, char *buf, size_t n)
{
    if (!g->utc_valid) {
        snprintf(buf, n, "--:--:--");
        return;
    }
    snprintf(buf, n, "%02u:%02u:%02u.%03u",
             (unsigned)g->hour, (unsigned)g->minute, (unsigned)g->second,
             (unsigned)g->millisecond);
}

static void status_task(void *arg)
{
    (void)arg;

    const uint32_t period_s = APP_STATUS_PERIOD_S;
    uint32_t prev_imu = 0, prev_mag = 0, prev_gnss = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(period_s * 1000));

        const uint32_t imu = g_stats.imu_samples;
        const uint32_t mag = g_stats.mag_samples;
        const uint32_t gnss = g_stats.gnss_samples;

        const unsigned imu_hz = (imu - prev_imu) / period_s;
        const unsigned mag_hz = (mag - prev_mag) / period_s;
        const unsigned gnss_hz = (gnss - prev_gnss) / period_s;

        prev_imu = imu;
        prev_mag = mag;
        prev_gnss = gnss;

        gnss_sample_t g;
        pps_stats_t pps;
        app_state_get_gnss(&g, &pps);

        logger_status_t ls;
        logger_get_status(&ls);

        char utc[24];
        format_utc(&g, utc, sizeof(utc));

        uint64_t fs_total = 0, fs_free = 0;
        const bool have_fs = ls.sd_mounted &&
            esp_vfs_fat_info(APP_LOG_MOUNT_POINT, &fs_total, &fs_free) == ESP_OK;

        const uint32_t pps_jitter =
            (pps.max_interval_us >= pps.min_interval_us)
                ? (pps.max_interval_us - pps.min_interval_us) : 0;

        ESP_LOGI(TAG,
                 "up=%us imu=%uHz mag=%uHz gnss=%u/s log=%s session=%s "
                 "utc=%s fix=%u/%u sats=%u hdop=%.2f "
                 "pps=%u iv=%uus jit=%uus "
                 "rows=%llu/%llu/%llu/%llu bytes=%llu err=%u rst=%u "
                 "drop=%u/%u/%u/%u sd=%s",
                 (unsigned)(esp_timer_get_time() / 1000000),
                 imu_hz, mag_hz, gnss_hz,
                 ls.logging ? "on" : "off",
                 ls.session[0] ? ls.session : "-",
                 utc, (unsigned)g.fix_type, (unsigned)g.fix_mode,
                 (unsigned)g.sats, g.hdop,
                 (unsigned)pps.seq, (unsigned)pps.last_interval_us, (unsigned)pps_jitter,
                 (unsigned long long)ls.rows[LOGGER_FILE_IMU],
                 (unsigned long long)ls.rows[LOGGER_FILE_MAG],
                 (unsigned long long)ls.rows[LOGGER_FILE_GNSS],
                 (unsigned long long)ls.rows[LOGGER_FILE_EVENT],
                 (unsigned long long)ls.bytes_written,
                 (unsigned)ls.write_errors, (unsigned)ls.remounts,
                 (unsigned)g_stats.imu_drops, (unsigned)g_stats.mag_drops,
                 (unsigned)g_stats.gnss_drops, (unsigned)g_stats.key_drops,
                 have_fs ? "ok" : "no");

        if (have_fs) {
            ESP_LOGI(TAG, "  storage: %llu/%llu KiB free, %s",
                     (unsigned long long)(fs_free / 1024),
                     (unsigned long long)(fs_total / 1024),
                     ls.logging ? "recording" : "paused");
        }
    }
}

esp_err_t status_init(void)
{
    return ESP_OK;
}

void status_start_task(void)
{
    xTaskCreate(status_task, "status", APP_STACK_STATUS, NULL, APP_PRIO_STATUS, NULL);
}
