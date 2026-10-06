/*
 * logger_sd.c - writes the four CSV streams to the TF card and mirrors a
 * subset of them to the USB serial console.
 *
 * Design notes
 * ------------
 * - One file per source, because the sources run at very different rates and
 *   keeping them separate keeps each row self-describing via `t_us`.
 * - Each power-up gets a fresh run index, discovered by scanning the card, so
 *   nothing is ever overwritten and no NVS state is required.
 * - Files are opened with a large stdio buffer and flushed on a timer; the SD
 *   card can block for tens of milliseconds, so writes are never issued from
 *   the sensor tasks.
 * - The logger never blocks the producers: all queues are drained
 *   non-blockingly, and a full queue is reported as a drop rather than
 *   stalling the ISR or the sensor task.
 * - A card that starts failing is closed, remounted once and reopened under a
 *   new run index; the event is recorded in events_*.csv.
 */
#include "logger_sd.h"

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "app_config.h"
#include "board_pins.h"
#include "data_types.h"
#include "log_format.h"

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

static const char *TAG = "logger";

/* ------------------------------------------------------- metadata strings */

#if defined(CONFIG_APP_BMI160_ACC_RANGE_2G)
#define META_ACC_G   2
#elif defined(CONFIG_APP_BMI160_ACC_RANGE_4G)
#define META_ACC_G   4
#elif defined(CONFIG_APP_BMI160_ACC_RANGE_16G)
#define META_ACC_G   16
#else
#define META_ACC_G   8
#endif

#if defined(CONFIG_APP_BMI160_GYR_RANGE_125DPS)
#define META_GYR_DPS 125
#elif defined(CONFIG_APP_BMI160_GYR_RANGE_250DPS)
#define META_GYR_DPS 250
#elif defined(CONFIG_APP_BMI160_GYR_RANGE_500DPS)
#define META_GYR_DPS 500
#elif defined(CONFIG_APP_BMI160_GYR_RANGE_2000DPS)
#define META_GYR_DPS 2000
#else
#define META_GYR_DPS 1000
#endif

#if defined(CONFIG_APP_QMC5883L_RANGE_8G)
#define META_MAG_G   8
#else
#define META_MAG_G   2
#endif

#if defined(CONFIG_APP_QMC5883L_OSR_64)
#define META_MAG_OSR 64
#elif defined(CONFIG_APP_QMC5883L_OSR_128)
#define META_MAG_OSR 128
#elif defined(CONFIG_APP_QMC5883L_OSR_512)
#define META_MAG_OSR 512
#else
#define META_MAG_OSR 256
#endif

/* ------------------------------------------------------------------ state */

static const char *const s_ext[LOGGER_FILE_COUNT] = {
    "imu", "mag", "gnss", "events"
};

static const char *const s_header[LOGGER_FILE_COUNT] = {
    LOG_HEADER_IMU, LOG_HEADER_MAG, LOG_HEADER_GNSS, LOG_HEADER_EVENT
};

static sdmmc_card_t      *s_card;
static bool               s_bus_inited;
static FILE              *s_f[LOGGER_FILE_COUNT];
static char               s_path[LOGGER_FILE_COUNT][80];

static SemaphoreHandle_t  s_status_mutex;
static logger_status_t    s_status;

static uint64_t           s_pending_bytes;
static uint32_t           s_consec_errors;
static unsigned           s_mirror_count[LOGGER_FILE_COUNT];
static bool               s_logging;
static bool               s_task_started;

/* ------------------------------------------------------------ small helpers */

static void status_lock(void)
{
    if (s_status_mutex) {
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    }
}

static void status_unlock(void)
{
    if (s_status_mutex) {
        xSemaphoreGive(s_status_mutex);
    }
}

static void status_set_logging(bool on)
{
    status_lock();
    s_status.logging = on;
    status_unlock();
}

/* ------------------------------------------------------------- file handling */

static void close_files(void)
{
    for (int i = 0; i < LOGGER_FILE_COUNT; i++) {
        if (s_f[i]) {
            fflush(s_f[i]);
            fclose(s_f[i]);
            s_f[i] = NULL;
        }
    }
    s_pending_bytes = 0;
}

static void write_metadata(int idx, uint32_t run)
{
    FILE *f = s_f[idx];
    if (!f) {
        return;
    }

    switch (idx) {
    case LOGGER_FILE_IMU:
        fprintf(f, "# imu run=%u odr_hz=%d acc_g=%d gyr_dps=%d int1=%d\n",
                (unsigned)run, APP_IMU_ODR_HZ, META_ACC_G, META_GYR_DPS,
                APP_BMI160_USE_INT1);
        break;
    case LOGGER_FILE_MAG:
        fprintf(f, "# mag run=%u odr_hz=%d osr=%d rng_g=%d\n",
                (unsigned)run, APP_MAG_RATE_HZ, META_MAG_OSR, META_MAG_G);
        break;
    case LOGGER_FILE_GNSS:
        fprintf(f, "# gnss run=%u baud=%d uart=%d epoch=RMC pps_gpio=%d\n",
                (unsigned)run, APP_GNSS_BAUD, APP_GNSS_UART_PORT, PIN_GNSS_PPS);
        break;
    default:
        fprintf(f, "# events run=%u\n", (unsigned)run);
        break;
    }

    fprintf(f, "%s\n", s_header[idx]);
}

static esp_err_t open_run_files(uint32_t run)
{
    for (int i = 0; i < LOGGER_FILE_COUNT; i++) {
        snprintf(s_path[i], sizeof(s_path[i]), "%s/%s_%04u.csv",
                 APP_LOG_MOUNT_POINT, s_ext[i], (unsigned)run);

        s_f[i] = fopen(s_path[i], "w");
        if (!s_f[i]) {
            ESP_LOGE(TAG, "cannot open %s", s_path[i]);
            close_files();
            return ESP_FAIL;
        }
        if (setvbuf(s_f[i], NULL, _IOFBF, APP_LOG_FILE_BUF_BYTES) != 0) {
            ESP_LOGW(TAG, "setvbuf failed for %s", s_path[i]);
        }
        write_metadata(i, run);
    }

    ESP_LOGI(TAG, "run %u: %s/{%s,%s,%s,%s}_%04u.csv",
             (unsigned)run, APP_LOG_MOUNT_POINT,
             s_ext[0], s_ext[1], s_ext[2], s_ext[3], (unsigned)run);
    return ESP_OK;
}

static esp_err_t sd_mount(void)
{
    esp_err_t err;

    if (!s_bus_inited) {
        const spi_bus_config_t bus = {
            .mosi_io_num     = PIN_SD_MOSI,
            .miso_io_num     = PIN_SD_MISO,
            .sclk_io_num     = PIN_SD_SCK,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = APP_SD_MAX_TRANSFER_SZ,
        };
        err = spi_bus_initialize(APP_SD_SPI_HOST, &bus, SDSPI_DEFAULT_DMA);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
            return err;
        }
        s_bus_inited = true;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = APP_SD_SPI_HOST;
    host.max_freq_khz = APP_SD_SPI_FREQ_KHZ;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.host_id = APP_SD_SPI_HOST;
    slot.gpio_cs = PIN_SD_CS;

    const esp_vfs_fat_mount_config_t mnt = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };

    err = esp_vfs_fat_sdspi_mount(APP_LOG_MOUNT_POINT, &host, &slot, &mnt, &s_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TF card mount failed: %s", esp_err_to_name(err));
        s_card = NULL;
        return err;
    }

    status_lock();
    s_status.sd_mounted = true;
    status_unlock();
    return ESP_OK;
}

static uint32_t scan_next_run_index(void)
{
    DIR *dir = opendir(APP_LOG_MOUNT_POINT);
    if (!dir) {
        return 1;
    }

    uint32_t max = 0;
    struct dirent *e;
    while ((e = readdir(dir)) != NULL) {
        unsigned idx;
        if (strncmp(e->d_name, "imu_", 4) == 0 &&
            sscanf(e->d_name, "%*[^_]_%u", &idx) == 1) {
            if (idx > max) {
                max = idx;
            }
        }
    }
    closedir(dir);

    return max + 1;
}

/* ------------------------------------------------------------- write path */

static void write_row(logger_file_t idx, const char *line)
{
    FILE *f = s_f[idx];
    if (!f) {
        return;
    }

    const int n = fprintf(f, "%s\n", line);
    if (n < 0) {
        s_consec_errors++;
        status_lock();
        s_status.write_errors++;
        status_unlock();
        return;
    }

    s_consec_errors = 0;
    s_pending_bytes += (uint64_t)n;

    status_lock();
    s_status.bytes_written += (uint64_t)n;
    s_status.rows[idx]++;
    status_unlock();

    if (APP_SERIAL_MIRROR) {
        bool mirror = true;
        if (idx == LOGGER_FILE_IMU || idx == LOGGER_FILE_MAG) {
            /* The 100 Hz streams would otherwise throttle the logger behind the
             * console; decimate them, but keep GNSS and events complete. */
            mirror = (++s_mirror_count[idx] % APP_SERIAL_MIRROR_DECIM) == 0;
        }
        if (mirror) {
            printf("%s\n", line);
        }
    }
}

static void logger_recover(void);

static void flush_files(void)
{
    for (int i = 0; i < LOGGER_FILE_COUNT; i++) {
        if (!s_f[i]) {
            continue;
        }
        if (fflush(s_f[i]) != 0) {
            s_consec_errors++;
            status_lock();
            s_status.write_errors++;
            status_unlock();
            continue;
        }
        fsync(fileno(s_f[i]));
    }
    s_pending_bytes = 0;

    if (s_consec_errors >= APP_LOG_MAX_CONSEC_ERR) {
        logger_recover();
    }
}

static void logger_recover(void)
{
    ESP_LOGE(TAG, "%u consecutive write errors, attempting remount",
             (unsigned)s_consec_errors);
    app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SD_ERROR);

    close_files();

    if (s_card) {
        const esp_err_t e = esp_vfs_fat_sdcard_unmount(APP_LOG_MOUNT_POINT, s_card);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "unmount failed: %s", esp_err_to_name(e));
        }
        s_card = NULL;
    }

    status_lock();
    s_status.sd_mounted = false;
    status_unlock();

    s_consec_errors = 0;

    if (sd_mount() != ESP_OK) {
        ESP_LOGE(TAG, "remount failed, continuing serial-only");
        return;
    }

    const uint32_t run = scan_next_run_index();
    if (open_run_files(run) != ESP_OK) {
        ESP_LOGE(TAG, "reopening files failed, continuing serial-only");
        return;
    }

    status_lock();
    s_status.run_index = run;
    s_status.remounts++;
    status_unlock();

    app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SD_REMOUNT);
    ESP_LOGW(TAG, "remounted, now logging to run %u", (unsigned)run);
}

/* ----------------------------------------------------------- queue draining */

static void drain_imu(void)
{
    imu_sample_t s;
    while (xQueueReceive(q_imu, &s, 0) == pdTRUE) {
        if (!s_logging) {
            continue;
        }
        char line[APP_LOG_LINE_BYTES];
        if (log_fmt_imu(line, sizeof(line), &s) > 0) {
            write_row(LOGGER_FILE_IMU, line);
        }
    }
}

static void drain_mag(void)
{
    mag_sample_t s;
    while (xQueueReceive(q_mag, &s, 0) == pdTRUE) {
        if (!s_logging) {
            continue;
        }
        char line[APP_LOG_LINE_BYTES];
        if (log_fmt_mag(line, sizeof(line), &s) > 0) {
            write_row(LOGGER_FILE_MAG, line);
        }
    }
}

static void drain_gnss(void)
{
    gnss_sample_t s;
    while (xQueueReceive(q_gnss, &s, 0) == pdTRUE) {
        if (!s_logging) {
            continue;
        }
        char line[APP_LOG_LINE_BYTES];
        if (log_fmt_gnss(line, sizeof(line), &s) > 0) {
            write_row(LOGGER_FILE_GNSS, line);
        }
    }
}

static void drain_key(void)
{
    key_event_t e;
    while (xQueueReceive(q_key, &e, 0) == pdTRUE) {
        uint8_t out = e.event;

        if (e.event == EVT_KEY1_PRESS) {
            s_logging = !s_logging;
            out = s_logging ? EVT_LOG_START : EVT_LOG_STOP;
            status_set_logging(s_logging);
            ESP_LOGI(TAG, "logging %s", s_logging ? "started" : "stopped");
        } else if (e.event == EVT_KEY2_PRESS) {
            out = EVT_MARK;
            ESP_LOGI(TAG, "event marker");
        }

        /* Event rows are always written, even while logging is paused, so the
         * start/stop bookkeeping itself is never lost. */
        const key_event_t row = { .t_us = e.t_us, .src = e.src, .event = out };
        char line[APP_LOG_LINE_BYTES];
        if (log_fmt_event(line, sizeof(line), &row) > 0) {
            write_row(LOGGER_FILE_EVENT, line);
        }
    }
}

/* -------------------------------------------------------------------- task */

/*
 * This task must never return: a FreeRTOS task that falls off the end of its
 * entry function triggers an abort. Draining by polling instead of with a queue
 * set keeps that guarantee simple, because there is then no setup step which
 * can fail due to startup ordering (xQueueAddToSet() rejects a queue that is
 * not empty, and boot events are queued before this task starts).
 */
static void logger_task(void *arg)
{
    (void)arg;

    const TickType_t flush_ticks = pdMS_TO_TICKS(APP_LOG_FLUSH_MS);
    const TickType_t poll_ticks = pdMS_TO_TICKS(APP_LOG_POLL_MS);
    TickType_t next_flush = xTaskGetTickCount() + flush_ticks;

    for (;;) {
        /* Drain every queue completely and non-blockingly. Producers are never
         * blocked; a full queue is reported via the drop counters instead. */
        drain_imu();
        drain_mag();
        drain_gnss();
        drain_key();

        const TickType_t now = xTaskGetTickCount();
        if ((int32_t)(now - next_flush) >= 0 ||
            s_pending_bytes >= APP_LOG_BATCH_FLUSH_BYTES) {
            flush_files();
            next_flush = now + flush_ticks;
        }

        vTaskDelay(poll_ticks ? poll_ticks : 1);
    }
}

/* -------------------------------------------------------------------- API */

esp_err_t logger_init(void)
{
    s_status_mutex = xSemaphoreCreateMutex();
    if (!s_status_mutex) {
        ESP_LOGE(TAG, "cannot create status mutex");
        return ESP_ERR_NO_MEM;
    }

    s_status.logging = false;

    if (!APP_LOG_TO_SD) {
        ESP_LOGW(TAG, "TF logging disabled in menuconfig (serial mirror only)");
    } else if (sd_mount() != ESP_OK) {
        app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SD_MOUNT_FAIL);
    } else {
        app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SD_MOUNTED);

        const uint32_t run = scan_next_run_index();
        if (open_run_files(run) != ESP_OK) {
            ESP_LOGE(TAG, "opening run files failed, serial-only");
        } else {
            s_status.run_index = run;
        }
    }

    s_logging = APP_LOG_AUTOSTART;

    app_event_log(esp_timer_get_time(), SRC_SYS, EVT_BOOT);
    if (s_logging) {
        app_event_log(esp_timer_get_time(), SRC_SYS, EVT_LOG_START);
    }
    status_set_logging(s_logging);

    return ESP_OK;
}

void logger_start_task(void)
{
    if (s_task_started) {
        return;
    }
    s_task_started = true;
    xTaskCreate(logger_task, "logger", APP_STACK_LOGGER, NULL, APP_PRIO_LOGGER, NULL);
}

void logger_get_status(logger_status_t *out)
{
    if (!out) {
        return;
    }
    status_lock();
    *out = s_status;
    status_unlock();
}

bool logger_is_logging(void)
{
    return s_logging;
}
