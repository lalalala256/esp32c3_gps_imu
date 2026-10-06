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
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

static const char *TAG = "logger";

/* Tag used by the IDF FATFS/SDMMC VFS layer, so background retries can be
 * silenced without muting the rest of the driver. */
#define VFS_FAT_TAG "vfs_fat_sdmmc"

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

/*
 * Session identity, and the anchor that ties the monotonic clock to UTC.
 *
 * The four files of one recording all share `s_base` (see open_session() for
 * the naming scheme). `s_synced` latches once the recording has been restarted
 * under the GNSS time, so a receiver that loses and later re-acquires time does
 * not roll the files over a second time.
 *
 * The anchor is a single (utc_us, t_us) pair: every row's absolute time is then
 * anchor_utc + (row_t_us - anchor_t_us), which is what lets each file carry a
 * microsecond UTC column while the logger runs at 100 Hz.
 */
static char               s_base[APP_LOG_BASE_BYTES];
static bool               s_synced;
static bool               s_utc_anchor_valid;
static int64_t            s_utc_anchor_us;
static int64_t            s_utc_anchor_t_us;

static SemaphoreHandle_t  s_status_mutex;
static logger_status_t    s_status;

/* Recursive, so helpers may nest (e.g. open_run_files() calling close_files()
 * on its failure path) without the caller tracking whether it holds the lock. */
static SemaphoreHandle_t  s_fs_mutex;

static uint64_t           s_pending_bytes;
static uint32_t           s_consec_errors;
static unsigned           s_mirror_count[LOGGER_FILE_COUNT];
static bool               s_logging;
static bool               s_task_started;

/*
 * Set once the sleep path has closed the card, and never cleared: waking from
 * deep sleep is a reboot, so this run of the firmware has no card to return to.
 *
 * Without it the logger's "keep trying to mount a card that was absent" loop
 * happily remounts and reopens the files seconds after they were closed, which
 * is exactly the state deep sleep must not be entered in - the reopened files
 * would still hold unflushed data when the chip powers down.
 */
static volatile bool      s_shutdown;

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

/* ------------------------------------------------------------------ locking */

/*
 * Serialises TF card access between the logger task and the web file browser.
 * Even if FATFS is built reentrant, the logger can unmount and remount the
 * volume on repeated write errors, which would pull the card out from under an
 * in-flight download; an application-level lock is the only thing that covers
 * that case.
 */
static void fs_lock(void)
{
    if (s_fs_mutex) {
        xSemaphoreTakeRecursive(s_fs_mutex, portMAX_DELAY);
    }
}

static void fs_unlock(void)
{
    if (s_fs_mutex) {
        xSemaphoreGiveRecursive(s_fs_mutex);
    }
}

bool sd_fs_lock(uint32_t timeout_ms)
{
    if (!s_fs_mutex) {
        return false;
    }
    return xSemaphoreTakeRecursive(s_fs_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void sd_fs_unlock(void)
{
    fs_unlock();
}

bool logger_sd_mounted(void)
{
    return s_status.sd_mounted;
}

/* Defined below, next to the other file handling. */
static void close_files(void);

void logger_shutdown_for_sleep(void)
{
    /*
     * Set before anything is closed, so the logger task cannot slip a remount
     * in between the close and the unmount. Also stops the logging toggle: a
     * key event arriving during the teardown must not restart recording into
     * files that are about to be closed.
     */
    s_shutdown = true;
    s_logging = false;
    status_set_logging(false);

    close_files();

    fs_lock();
    if (s_card) {
        const esp_err_t e = esp_vfs_fat_sdcard_unmount(APP_LOG_MOUNT_POINT, s_card);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "unmount failed: %s", esp_err_to_name(e));
        }
        s_card = NULL;
    }
    fs_unlock();

    status_lock();
    s_status.sd_mounted = false;
    status_unlock();

    ESP_LOGI(TAG, "logging closed for sleep");
}

/* ------------------------------------------------------- time bookkeeping */

/*
 * Days since 1970-01-01 for a proleptic Gregorian date.
 *
 * Written out rather than calling timegm(), which in newlib reads the TZ
 * environment and would silently shift the timestamps of a board with no clock
 * configuration at all.
 */
static int64_t days_from_civil(long y, unsigned m, unsigned d)
{
    y -= (m <= 2);
    const long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153u * (m + (m > 2 ? -3 : 9)) + 2u) / 5u + d - 1u;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097L + (long)doe - 719468L;
}

/*
 * UTC of a GNSS epoch in microseconds since the Unix epoch, or 0 when the
 * receiver has no usable time.
 *
 * The year floor is not paranoia: a receiver that has not decoded the almanac
 * yet can report a plausible-looking but wrong date, and naming a recording
 * after it would be worse than waiting for a real one.
 */
static int64_t gnss_utc_us(const gnss_sample_t *s)
{
    if (!s->utc_valid || s->year < APP_LOG_MIN_YEAR || s->month < 1 ||
        s->month > 12 || s->day < 1 || s->day > 31 ||
        s->hour > 23 || s->minute > 59 || s->second > 60) {
        return 0;
    }

    const int64_t days = days_from_civil((long)s->year, s->month, s->day);
    const int64_t secs = days * 86400 + (int64_t)s->hour * 3600 +
                         (int64_t)s->minute * 60 + s->second;

    return secs * 1000000 + (int64_t)s->millisecond * 1000;
}

/*
 * Re-anchor on every epoch that carries a time. Each one is a fresh, exact
 * correspondence between the two clocks, so using the newest keeps the derived
 * UTC from drifting with the local oscillator.
 */
static void update_utc_anchor(const gnss_sample_t *s)
{
    const int64_t utc = gnss_utc_us(s);
    if (utc <= 0) {
        return;
    }
    s_utc_anchor_us = utc;
    s_utc_anchor_t_us = s->t_us;
    s_utc_anchor_valid = true;
}

static int64_t utc_us_of(int64_t t_us)
{
    return s_utc_anchor_valid ? (s_utc_anchor_us + (t_us - s_utc_anchor_t_us)) : 0;
}

/* Time only: the 100 Hz streams have no use for a position. */
static log_ctx_t ctx_time(int64_t t_us)
{
    const log_ctx_t c = { .utc_us = utc_us_of(t_us) };
    return c;
}

/*
 * Time plus the last known position, for the event file: this is what lets a
 * KEY2 mark be plotted on a map straight from that one file.
 */
static log_ctx_t ctx_mark(int64_t t_us)
{
    gnss_sample_t g;
    pps_stats_t pps;
    app_state_get_gnss(&g, &pps);

    const log_ctx_t c = {
        .utc_us    = utc_us_of(t_us),
        .lat_deg   = g.pos_valid ? g.lat_deg : 0.0,
        .lon_deg   = g.pos_valid ? g.lon_deg : 0.0,
        .pos_valid = g.pos_valid,
    };
    return c;
}

/* ------------------------------------------------------------- file handling */

static void close_files(void)
{
    fs_lock();
    for (int i = 0; i < LOGGER_FILE_COUNT; i++) {
        if (s_f[i]) {
            fflush(s_f[i]);
            fclose(s_f[i]);
            s_f[i] = NULL;
        }
    }
    s_pending_bytes = 0;
    fs_unlock();
}

static void write_metadata(int idx, const char *base)
{
    FILE *f = s_f[idx];
    if (!f) {
        return;
    }

    /*
     * The anchor line first, because it is what makes an offline join possible
     * without opening any other file: utc_us and t_us there are the same
     * instant, so every row's absolute time is anchor_utc + (t_us - anchor_t).
     */
    if (s_utc_anchor_valid) {
        fprintf(f, "# anchor utc_us=%lld t_us=%lld\n",
                (long long)s_utc_anchor_us, (long long)s_utc_anchor_t_us);
    } else {
        fprintf(f, "# anchor none (the receiver had no usable time yet)\n");
    }

    switch (idx) {
    case LOGGER_FILE_IMU:
        fprintf(f, "# imu session=%s odr_hz=%d acc_g=%d gyr_dps=%d int1=%d\n",
                base, APP_IMU_ODR_HZ, META_ACC_G, META_GYR_DPS,
                APP_BMI160_USE_INT1);
        break;
    case LOGGER_FILE_MAG:
        fprintf(f, "# mag session=%s odr_hz=%d osr=%d rng_g=%d\n",
                base, APP_MAG_RATE_HZ, META_MAG_OSR, META_MAG_G);
        break;
    case LOGGER_FILE_GNSS:
        fprintf(f, "# gnss session=%s baud=%d uart=%d epoch=RMC pps_gpio=%d\n",
                base, APP_GNSS_BAUD, APP_GNSS_UART_PORT, PIN_GNSS_PPS);
        break;
    default:
        fprintf(f, "# events session=%s\n", base);
        break;
    }

    fprintf(f, "%s\n", s_header[idx]);
}

/*
 * Open the four files of one session.
 *
 * Naming
 * ------
 * The four files of a recording share a base name, so a later fusion step can
 * find them by prefix with no manifest:
 *
 *   20261006_062222_imu.csv
 *   20261006_062222_mag.csv
 *   20261006_062222_gnss.csv
 *   20261006_062222_events.csv
 *
 * The base is the GNSS UTC of the first epoch that carried a usable time, which
 * is what makes the name meaningful later and what tells recordings from two
 * boards apart. Until the receiver has a time there is nothing but uptime to go
 * on, so those files are `nosync_NNNN_*` and the recording restarts - into a
 * fresh, time-named set - the moment a time arrives. The nosync set is not
 * wasted, it still holds everything recorded before the fix.
 *
 * `append` reopens existing files instead of truncating them, which is what a
 * remount after a transient write error needs: a flaky card is not a reason to
 * throw away the recording so far.
 */
static esp_err_t open_session(const char *base, bool append)
{
    if (s_shutdown) {
        return ESP_ERR_INVALID_STATE;
    }

    fs_lock();
    for (int i = 0; i < LOGGER_FILE_COUNT; i++) {
        snprintf(s_path[i], sizeof(s_path[i]), "%s/%s_%s.csv",
                 APP_LOG_MOUNT_POINT, base, s_ext[i]);

        s_f[i] = fopen(s_path[i], append ? "a" : "w");
        if (!s_f[i]) {
            ESP_LOGE(TAG, "cannot open %s", s_path[i]);
            close_files();
            fs_unlock();
            return ESP_FAIL;
        }
        if (setvbuf(s_f[i], NULL, _IOFBF, APP_LOG_FILE_BUF_BYTES) != 0) {
            ESP_LOGW(TAG, "setvbuf failed for %s", s_path[i]);
        }
        write_metadata(i, base);
    }

    snprintf(s_base, sizeof(s_base), "%s", base);
    status_lock();
    snprintf(s_status.session, sizeof(s_status.session), "%s", base);
    status_unlock();
    fs_unlock();

    ESP_LOGI(TAG, "session %s -> %s/%s_{%s,%s,%s,%s}.csv", base,
             APP_LOG_MOUNT_POINT, base, s_ext[0], s_ext[1], s_ext[2], s_ext[3]);
    return ESP_OK;
}

/*
 * Make `base` unique on the card by appending a counter.
 *
 * A GNSS-second name can only collide after a restart inside the same second,
 * but "the file already existed" is a bad way to discover that a recording has
 * been overwritten.
 */
static void unique_base(char *base, size_t n)
{
    char candidate[APP_LOG_BASE_BYTES];
    char probe[96];

    for (unsigned suffix = 0; suffix < 100; suffix++) {
        if (suffix == 0) {
            snprintf(candidate, sizeof(candidate), "%s", base);
        } else {
            snprintf(candidate, sizeof(candidate), "%s_%u", base, suffix);
        }

        snprintf(probe, sizeof(probe), "%s/%s_%s.csv", APP_LOG_MOUNT_POINT,
                 candidate, s_ext[LOGGER_FILE_IMU]);

        /* Under the card lock: the web server reads the same filesystem from
         * another task, and FATFS itself is only safe because every access in
         * this project goes through this mutex. */
        fs_lock();
        FILE *f = fopen(probe, "r");
        if (!f) {
            fs_unlock();
            snprintf(base, n, "%s", candidate);
            return;                 /* free */
        }
        fclose(f);
        fs_unlock();
    }
}

/* Next free nosync index, so boots without a fix never overwrite earlier data. */
static void nosync_base(char *base, size_t n)
{
    unsigned max = 0;

    fs_lock();
    DIR *dir = opendir(APP_LOG_MOUNT_POINT);
    if (dir) {
        struct dirent *e;
        while ((e = readdir(dir)) != NULL) {
            unsigned idx;
            if (strncmp(e->d_name, "nosync_", 7) == 0 &&
                sscanf(e->d_name, "%*[^_]_%u", &idx) == 1 && idx > max) {
                max = idx;
            }
        }
        closedir(dir);
    }
    fs_unlock();

    snprintf(base, n, "nosync_%04u", max + 1);
}

/*
 * Get the logger recording, without discarding anything already recorded.
 *
 * Reopens the current session when there is one (a card that came back after a
 * write error), and otherwise starts a fresh one. False means the files could
 * not be opened and the logger is on the serial mirror only.
 */
static bool start_session(void)
{
    if (s_base[0] && open_session(s_base, true) == ESP_OK) {
        return true;
    }

    char base[APP_LOG_BASE_BYTES];
    nosync_base(base, sizeof(base));
    unique_base(base, sizeof(base));
    return open_session(base, false) == ESP_OK;
}

/*
 * Restart the recording under a name taken from the GNSS clock. Called for
 * every GNSS epoch until it succeeds.
 */
static void sync_to_gps_time(const gnss_sample_t *s)
{
    if (s_synced) {
        return;
    }

    const int64_t utc = gnss_utc_us(s);
    if (utc <= 0) {
        return;
    }

    char base[APP_LOG_BASE_BYTES];
    snprintf(base, sizeof(base), "%04u%02u%02u_%02u%02u%02u",
             (unsigned)s->year, (unsigned)s->month, (unsigned)s->day,
             (unsigned)s->hour, (unsigned)s->minute, (unsigned)s->second);

    /*
     * Anchor before the files move, so the first row written into the new
     * session already carries an absolute time.
     */
    s_utc_anchor_us = utc;
    s_utc_anchor_t_us = s->t_us;
    s_utc_anchor_valid = true;
    s_synced = true;

    ESP_LOGW(TAG, "GNSS time available: restarting the recording as %s", base);

    if (!s_status.sd_mounted) {
        return;                 /* serial-only: the anchor is all there is to do */
    }

    fs_lock();
    close_files();
    fs_unlock();

    unique_base(base, sizeof(base));
    if (open_session(base, false) != ESP_OK) {
        /* Do not leave the logger with nothing open because a rename failed. */
        ESP_LOGE(TAG, "cannot start the time-named session, keeping %s open",
                 s_base);
        start_session();
        return;
    }

    app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SESSION_SYNC);
}

/* One mount attempt at a given SPI clock. */
static esp_err_t sd_mount_at(uint32_t freq_khz)
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = APP_SD_SPI_HOST;
    host.max_freq_khz = freq_khz;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.host_id = APP_SD_SPI_HOST;
    slot.gpio_cs = PIN_SD_CS;

    const esp_vfs_fat_mount_config_t mnt = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };

    return esp_vfs_fat_sdspi_mount(APP_LOG_MOUNT_POINT, &host, &slot, &mnt, &s_card);
}

/*
 * Raw SD SPI handshake, independent of the SDSPI protocol layer.
 *
 * This replays the driver's own sequence by hand - CMD0, CMD8, then the next
 * two commands the driver sends (CMD59 CRC_ON_OFF and CMD58 READ_OCR), then
 * ACMD41 and CMD59 again - and reports every R1 byte, which separates faults
 * that the driver's error codes conflate:
 *
 *   - everything 0xFF       -> nothing drives MISO at all (wiring or power)
 *   - a byte with bit 7 set -> crosstalk on a floating DAT0, not a reply, since
 *                             a real R1 always has bit 7 clear
 *   - CMD0/CMD8 answer but    the card implements SPI mode incompletely, even
 *     CMD59 is rejected       though it may be perfect in a PC's SD-mode reader
 *
 * CMD59 is the interesting one because IDF sends it *before* ACMD41
 * (sdmmc_init_spi_crc runs ahead of sdmmc_init_ocr in sdmmc_init.c). A card that
 * only accepts CMD59 after leaving the idle state therefore fails the mount no
 * matter how good the wiring is, so it is tested on both sides of ACMD41.
 */
#define PROBE_WINDOW 12

/*
 * Send one command frame and return the R1 byte the card answers with (0xFF if
 * it never answers). The four bytes that follow R1 - only CMD8 and CMD58 have
 * them - are copied into `trailer`, which is filled with 0xFF when not supplied.
 *
 * A fresh transaction is built on every call on purpose: the polling API writes
 * rxlength back into the structure, so reusing one across transfers of
 * different lengths makes the next call fail validation and leaves the rx
 * buffer holding whatever it had before - which reads exactly like a hardware
 * fault.
 */
static uint8_t probe_send(spi_device_handle_t dev, uint8_t cmd, uint32_t arg,
                          uint8_t crc, uint8_t *trailer)
{
    uint8_t tx[6 + PROBE_WINDOW];
    uint8_t rx[6 + PROBE_WINDOW];

    memset(tx, 0xFF, sizeof(tx));   /* idle level: MOSI must be high, not low */
    tx[0] = cmd;
    tx[1] = (uint8_t)(arg >> 24);
    tx[2] = (uint8_t)(arg >> 16);
    tx[3] = (uint8_t)(arg >> 8);
    tx[4] = (uint8_t)arg;
    tx[5] = crc;

    spi_transaction_t t = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    if (trailer) {
        memset(trailer, 0xFF, 4);
    }
    if (spi_device_polling_transmit(dev, &t) != ESP_OK) {
        return 0xFF;
    }

    /* The card is allowed a few bytes of latency before it answers, so look for
     * the first byte that differs from the idle level. */
    int r1 = -1;
    for (int i = 6; i < (int)sizeof(rx); i++) {
        if (rx[i] != 0xFF) {
            r1 = i;
            break;
        }
    }
    if (r1 < 0) {
        return 0xFF;
    }

    if (trailer) {
        for (int k = 0; k < 4 && r1 + 1 + k < (int)sizeof(rx); k++) {
            trailer[k] = rx[r1 + 1 + k];
        }
    }
    return rx[r1];
}

/* One pass of the driver's command sequence on one clock. */
typedef struct {
    uint8_t cmd0;
    uint8_t cmd8;
    uint8_t cmd5;
    uint8_t c59_before;
    uint8_t c58;
    uint8_t acmd41;
    uint8_t c59_after;
    uint8_t echo[4];
    uint8_t ocr[4];
} probe_run_t;

static void probe_run(spi_device_handle_t dev, probe_run_t *out)
{
    /*
     * CMD0, CMD8 and CMD59 - exactly the path the driver takes now that the SDIO
     * probe is skipped in SPI mode (see PATCHES.md). CMD5 is deliberately not
     * sent: this card rejects CMD59 once it has seen CMD5, which is the fault the
     * patch works around, so sending it would only re-create a misleading failure.
     */
    out->cmd0 = probe_send(dev, 0x40, 0x00000000, 0x95, NULL);        /* CMD0  */
    out->cmd8 = probe_send(dev, 0x48, 0x000001AA, 0x87, out->echo);   /* CMD8  */
    out->c59_before = probe_send(dev, 0x7B, 0x00000001, 0xFF, NULL);  /* CMD59 */

    out->cmd5 = 0xFF;
    out->c58 = 0xFF;
    out->acmd41 = 0xFF;
    out->c59_after = 0xFF;
    memset(out->ocr, 0xFF, sizeof(out->ocr));
}

/*
 * Just 400 kHz, because that is the clock the driver uses for the whole
 * identification phase (SDMMC_FREQ_PROBING in sdspi_host_init_slot). A 100/200/400
 * sweep was run while chasing this fault and gave identical results at every
 * clock, so the clock is not a variable worth logging.
 */
static const uint32_t s_probe_clocks_khz[] = { 400 };
#define PROBE_CLOCKS (sizeof(s_probe_clocks_khz) / sizeof(s_probe_clocks_khz[0]))

static void sd_raw_probe(bool verbose)
{
    probe_run_t res[PROBE_CLOCKS];
    bool ran[PROBE_CLOCKS];
    uint8_t idle[2] = { 0xFF, 0xFF };
    bool idle_ok = false;

    memset(res, 0xFF, sizeof(res));

    for (size_t i = 0; i < PROBE_CLOCKS; i++) {
        ran[i] = false;

        /* CS is driven by hand instead of by the SPI driver, because the spec
         * wants the >=74 initial clocks sent with CS *deasserted* and a device
         * configured with spics_io_num would hold CS low for the whole
         * transfer. */
        const spi_device_interface_config_t devcfg = {
            .clock_speed_hz = (int)(s_probe_clocks_khz[i] * 1000),
            .mode = 0,
            .spics_io_num = -1,
            .queue_size = 1,
        };

        spi_device_handle_t dev = NULL;
        if (spi_bus_add_device(APP_SD_SPI_HOST, &devcfg, &dev) != ESP_OK) {
            continue;
        }

        gpio_set_direction(PIN_SD_CS, GPIO_MODE_OUTPUT);

        /* Phase 1: >=74 clocks with CS high. This is the SPI mode reset, and it
         * is what actually got this card to answer at all. */
        uint8_t clocks_tx[10];
        uint8_t clocks_rx[10];
        memset(clocks_tx, 0xFF, sizeof(clocks_tx));
        spi_transaction_t t_clocks = {
            .length = sizeof(clocks_tx) * 8,
            .tx_buffer = clocks_tx,
            .rx_buffer = clocks_rx,
        };

        gpio_set_level(PIN_SD_CS, 1);
        const esp_err_t clocks_err = spi_device_polling_transmit(dev, &t_clocks);

        /* Phase 2: everything from here shares one CS-low window, which is what
         * the spec asks for once the card is selected. */
        gpio_set_level(PIN_SD_CS, 0);
        if (clocks_err == ESP_OK) {
            probe_run(dev, &res[i]);
            ran[i] = true;
        }

        /* Phase 3: CS high with nothing driving MISO. 0xFF here means the line
         * is released and the pull-up works; 0x00 means it is held low, i.e. a
         * short to ground or a card that is not seated correctly. Measured on
         * the first pass only, because it does not depend on the clock. */
        gpio_set_level(PIN_SD_CS, 1);
        if (i == 0 && clocks_err == ESP_OK) {
            spi_transaction_t t_idle = {
                .length = sizeof(idle) * 8,
                .tx_buffer = clocks_tx,
                .rx_buffer = idle,
            };
            idle_ok = (spi_device_polling_transmit(dev, &t_idle) == ESP_OK);
        }

        spi_bus_remove_device(dev);
    }

    /* ------------------------------------------------------------ reporting */

    /* One line per clock: the whole point of the sweep is to see where the card
     * stops answering cleanly. */
    for (size_t i = 0; i < PROBE_CLOCKS; i++) {
        if (!ran[i]) {
            ESP_LOGW(TAG, "sd probe @%u kHz: SPI device could not be added",
                     (unsigned)s_probe_clocks_khz[i]);
            continue;
        }
        ESP_LOGW(TAG, "sd probe @%u kHz: CMD0=0x%02X CMD8=0x%02X CMD59=0x%02X",
                 (unsigned)s_probe_clocks_khz[i], res[i].cmd0, res[i].cmd8,
                 res[i].c59_before);
    }

    /* The slowest clock that got an answer is the reference, since a link that is
     * marginal at speed may answer nothing at all when it is fast. */
    const probe_run_t *ref = NULL;
    for (size_t i = 0; i < PROBE_CLOCKS; i++) {
        if (ran[i] && res[i].cmd0 != 0xFF) {
            ref = &res[i];
            break;
        }
    }

    if (!ref) {
        if (verbose) {
            ESP_LOGW(TAG, "MISO stayed 0xFF at every clock, so the card never drove the data "
                     "line. Check that a card is inserted and properly seated, and that "
                     "DAT0 (pin 7) reaches GPIO%d", PIN_SD_MISO);
        } else {
            /* Short heartbeat so the 2 s retry cadence stays visible. */
            ESP_LOGI(TAG, "sd probe: no response (all 0xFF)");
        }
        return;
    }

    /* A real R1 always has bit 7 clear, so a byte with bit 7 set means nothing is
     * driving MISO and we are seeing crosstalk from CLK and MOSI. */
    if ((ref->cmd0 & 0x80) != 0) {
        ESP_LOGW(TAG, "sd probe: CMD0 answered 0x%02X, whose bit 7 is set, so that is "
                 "crosstalk on a floating DAT0 and not a card reply (check pin 7 -> "
                 "GPIO%d)", ref->cmd0, PIN_SD_MISO);
        return;
    }

    /* A line that is low with CS high and nothing transmitting is not a reply. */
    if (idle_ok && idle[0] == 0x00 && idle[1] == 0x00) {
        ESP_LOGW(TAG, "sd probe: MISO is held LOW even with CS high and the bus idle "
                 "-> that is a short to ground, or a card that is not seated correctly");
        return;
    }

    /* The card answered for real. */
    status_lock();
    s_status.card_seen = true;
    s_status.probe_hits++;
    status_unlock();

    static TickType_t s_last_trace;
    const TickType_t now = xTaskGetTickCount();
    if (now - s_last_trace > pdMS_TO_TICKS(15000)) {
        s_last_trace = now;
        ESP_LOGI(TAG, "  CMD8 echo=%02X %02X %02X %02X, OCR=%02X %02X %02X %02X, "
                 "MISO with CS high and the bus idle=%02X %02X",
                 ref->echo[0], ref->echo[1], ref->echo[2], ref->echo[3],
                 ref->ocr[0], ref->ocr[1], ref->ocr[2], ref->ocr[3],
                 idle[0], idle[1]);
    }

    const bool echo_ok = (ref->echo[0] == 0x00 && ref->echo[1] == 0x00 &&
                          ref->echo[2] == 0x01 && ref->echo[3] == 0xAA);
    if (ref->cmd8 != 0xFF && !echo_ok) {
        ESP_LOGW(TAG, "sd probe: CMD8's trailing bytes are %02X %02X %02X %02X instead of "
                 "00 00 01 AA, so the end of a response is not arriving cleanly",
                 ref->echo[0], ref->echo[1], ref->echo[2], ref->echo[3]);
    }

    if ((ref->c59_before & 0x04) != 0) {
        ESP_LOGW(TAG, "sd probe: the card rejects CMD59 as an illegal command, which is what "
                 "makes the mount fail - see PATCHES.md about the SDIO probe (CMD5) in "
                 "sdmmc_card_init()");
    }
}

/*
 * Mount the card, retrying at progressively slower clocks. A TF module on long
 * jumper wires, or one without pull-ups on MISO/CS, commonly times out at
 * 20 MHz yet mounts fine at 4 MHz or 400 kHz. A failed mount unregisters its
 * VFS path, so retrying is safe.
 */
static esp_err_t sd_mount(bool verbose)
{
    /* Background retries keep the SPI traffic (so a scope still triggers) but
     * reduce logging to a one-line verdict per attempt. */
    esp_err_t result = ESP_FAIL;

    fs_lock();

    /* FATFS logs an ERROR for every failed speed attempt, which is pure noise
     * on a repeating background retry. */
    if (!verbose) {
        esp_log_level_set(VFS_FAT_TAG, ESP_LOG_NONE);
    }

    if (!s_bus_inited) {
        const spi_bus_config_t bus = {
            .mosi_io_num     = PIN_SD_MOSI,
            .miso_io_num     = PIN_SD_MISO,
            .sclk_io_num     = PIN_SD_SCK,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = APP_SD_MAX_TRANSFER_SZ,
        };
        const esp_err_t err = spi_bus_initialize(APP_SD_SPI_HOST, &bus, SDSPI_DEFAULT_DMA);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
            result = err;
            goto out;
        }
        s_bus_inited = true;
    }

    sd_raw_probe(verbose);

    static const uint32_t speeds[] = { APP_SD_SPI_FREQ_KHZ, 4000, 400 };

    for (size_t i = 0; i < sizeof(speeds) / sizeof(speeds[0]); i++) {
        if (i > 0 && speeds[i] == speeds[i - 1]) {
            continue;
        }

        s_card = NULL;
        const esp_err_t err = sd_mount_at(speeds[i]);
        if (err == ESP_OK) {
            if (i > 0) {
                ESP_LOGW(TAG, "TF card mounted at a reduced clock (%u kHz)",
                         (unsigned)speeds[i]);
            }
            status_lock();
            s_status.sd_mounted = true;
            s_status.spi_khz = speeds[i];
            status_unlock();
            result = ESP_OK;
            goto out;
        }

        if (verbose) {
            ESP_LOGE(TAG, "TF card mount at %u kHz failed: %s",
                     (unsigned)speeds[i], esp_err_to_name(err));
        }
    }

    if (verbose) {
        ESP_LOGE(TAG, "TF card unusable: check that a card is inserted and that "
                 "MISO/MOSI/CS are not swapped and have pull-ups");
    }
    s_card = NULL;

out:
    if (!verbose) {
        esp_log_level_set(VFS_FAT_TAG, ESP_LOG_ERROR);
    }
    fs_unlock();
    return result;
}

/* ------------------------------------------------------------- write path */

static void write_row(logger_file_t idx, const char *line)
{
    int n = -1;
    bool have;

    /*
     * The pointer is read under the same lock that close_files() takes. The
     * sleep path closes the files from another task, so reading s_f[idx]
     * outside the lock would let an in-flight write land on an already-closed
     * FILE.
     */
    fs_lock();
    have = (s_f[idx] != NULL) && !s_shutdown;
    if (have) {
        n = fprintf(s_f[idx], "%s\n", line);
    }
    fs_unlock();

    if (!have) {
        return;                 /* no file open: nothing to record */
    }

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
    fs_lock();
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
    fs_unlock();

    if (s_consec_errors >= APP_LOG_MAX_CONSEC_ERR) {
        logger_recover();
    }
}

static void logger_recover(void)
{
    ESP_LOGE(TAG, "%u consecutive write errors, attempting remount",
             (unsigned)s_consec_errors);
    app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SD_ERROR);

    fs_lock();
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

    if (sd_mount(false) != ESP_OK) {
        fs_unlock();
        ESP_LOGE(TAG, "remount failed, continuing serial-only");
        return;
    }

    /*
     * Reopen the session that was interrupted rather than starting a new one: a
     * transient write error is not a reason to split a recording in two, and
     * appending keeps whatever was already captured.
     */
    if (!start_session()) {
        fs_unlock();
        ESP_LOGE(TAG, "reopening files failed, continuing serial-only");
        return;
    }

    status_lock();
    s_status.remounts++;
    status_unlock();
    fs_unlock();

    app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SD_REMOUNT);
    ESP_LOGW(TAG, "remounted, continuing session %s", s_base);
}

/* ----------------------------------------------------------- queue draining */

static void drain_imu(void)
{
    imu_sample_t s;
    while (xQueueReceive(q_imu, &s, 0) == pdTRUE) {
        if (!s_logging) {
            continue;
        }
        const log_ctx_t ctx = ctx_time(s.t_us);
        char line[APP_LOG_LINE_BYTES];
        if (log_fmt_imu(line, sizeof(line), &s, &ctx) > 0) {
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
        const log_ctx_t ctx = ctx_time(s.t_us);
        char line[APP_LOG_LINE_BYTES];
        if (log_fmt_mag(line, sizeof(line), &s, &ctx) > 0) {
            write_row(LOGGER_FILE_MAG, line);
        }
    }
}

static void drain_gnss(void)
{
    gnss_sample_t s;
    while (xQueueReceive(q_gnss, &s, 0) == pdTRUE) {
        /*
         * Take the time before formatting. The epoch that carries the first
         * usable time is what opens the time-named session, and it has to land
         * in the new file rather than the one being closed.
         */
        update_utc_anchor(&s);

        if (!s_logging) {
            continue;
        }

        sync_to_gps_time(&s);

        const log_ctx_t ctx = ctx_time(s.t_us);
        char line[APP_LOG_LINE_BYTES];
        if (log_fmt_gnss(line, sizeof(line), &s, &ctx) > 0) {
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
        const log_ctx_t ctx = ctx_mark(row.t_us);
        char line[APP_LOG_LINE_BYTES];
        if (log_fmt_event(line, sizeof(line), &row, &ctx) > 0) {
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
#if APP_SD_RETRY_MS > 0
    TickType_t next_mount = xTaskGetTickCount() + pdMS_TO_TICKS(APP_SD_RETRY_MS);
    unsigned mount_retries = 0;
#endif

    for (;;) {
        /* Drain every queue completely and non-blockingly. Producers are never
         * blocked; a full queue is reported via the drop counters instead. */
        drain_imu();
        drain_mag();
        drain_gnss();
        drain_key();

        /*
         * Everything below opens, flushes or remounts the card. Once the sleep
         * path has closed it, doing any of that would reopen the files and
         * leave unflushed data behind when the chip powers down, so the task
         * goes idle. The drains above stay, which keeps the producers from
         * filling their queues while the board waits for the key release.
         */
        if (s_shutdown) {
            vTaskDelay(poll_ticks ? poll_ticks : 1);
            continue;
        }

        const TickType_t now = xTaskGetTickCount();
        if ((int32_t)(now - next_flush) >= 0 ||
            s_pending_bytes >= APP_LOG_BATCH_FLUSH_BYTES) {
            flush_files();
            next_flush = now + flush_ticks;
        }

#if APP_SD_RETRY_MS > 0
        /*
         * Keep trying to mount a card that was absent at boot. Besides making a
         * late insertion just work, each attempt emits a raw SPI handshake, so
         * a scope has a repeating transaction to trigger on.
         */
        if (!s_status.sd_mounted && (int32_t)(now - next_mount) >= 0) {
            next_mount = now + pdMS_TO_TICKS(APP_SD_RETRY_MS);

            const bool verbose = (mount_retries < 3);
            mount_retries++;

            if (sd_mount(verbose) == ESP_OK) {
                if (start_session()) {
                    app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SD_MOUNTED);
                    ESP_LOGI(TAG, "TF card appeared, recording into session %s",
                             s_base);
                }
            }
        }
#endif

        vTaskDelay(poll_ticks ? poll_ticks : 1);
    }
}

/* -------------------------------------------------------------------- API */

esp_err_t logger_init(void)
{
    s_status_mutex = xSemaphoreCreateMutex();
    s_fs_mutex = xSemaphoreCreateRecursiveMutex();
    if (!s_status_mutex || !s_fs_mutex) {
        ESP_LOGE(TAG, "cannot create logger mutexes");
        return ESP_ERR_NO_MEM;
    }

    s_status.logging = false;

    if (!APP_LOG_TO_SD) {
        ESP_LOGW(TAG, "TF logging disabled in menuconfig (serial mirror only)");
    } else if (sd_mount(true) != ESP_OK) {
        app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SD_MOUNT_FAIL);
    } else {
        app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SD_MOUNTED);

        /*
         * Recording starts here, with no key press required: the boot session
         * is named nosync_NNNN until the receiver supplies a time, at which
         * point it is replaced by one named after that time.
         */
        if (!start_session()) {
            ESP_LOGE(TAG, "opening the session files failed, serial-only");
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
