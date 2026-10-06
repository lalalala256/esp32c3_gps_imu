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
    fs_lock();
    for (int i = 0; i < LOGGER_FILE_COUNT; i++) {
        snprintf(s_path[i], sizeof(s_path[i]), "%s/%s_%04u.csv",
                 APP_LOG_MOUNT_POINT, s_ext[i], (unsigned)run);

        s_f[i] = fopen(s_path[i], "w");
        if (!s_f[i]) {
            ESP_LOGE(TAG, "cannot open %s", s_path[i]);
            close_files();
            fs_unlock();
            return ESP_FAIL;
        }
        if (setvbuf(s_f[i], NULL, _IOFBF, APP_LOG_FILE_BUF_BYTES) != 0) {
            ESP_LOGW(TAG, "setvbuf failed for %s", s_path[i]);
        }
        write_metadata(i, run);
    }
    fs_unlock();

    ESP_LOGI(TAG, "run %u: %s/{%s,%s,%s,%s}_%04u.csv",
             (unsigned)run, APP_LOG_MOUNT_POINT,
             s_ext[0], s_ext[1], s_ext[2], s_ext[3], (unsigned)run);
    return ESP_OK;
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

static uint32_t scan_next_run_index(void)
{
    DIR *dir;

    fs_lock();
    dir = opendir(APP_LOG_MOUNT_POINT);
    if (!dir) {
        fs_unlock();
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
    fs_unlock();

    return max + 1;
}

/* ------------------------------------------------------------- write path */

static void write_row(logger_file_t idx, const char *line)
{
    FILE *f = s_f[idx];
    if (!f) {
        return;
    }

    int n;
    fs_lock();
    n = fprintf(f, "%s\n", line);
    fs_unlock();

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

    const uint32_t run = scan_next_run_index();
    if (open_run_files(run) != ESP_OK) {
        fs_unlock();
        ESP_LOGE(TAG, "reopening files failed, continuing serial-only");
        return;
    }

    status_lock();
    s_status.run_index = run;
    s_status.remounts++;
    status_unlock();
    fs_unlock();

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
                const uint32_t run = scan_next_run_index();
                if (open_run_files(run) == ESP_OK) {
                    status_lock();
                    s_status.run_index = run;
                    status_unlock();
                    app_event_log(esp_timer_get_time(), SRC_SYS, EVT_SD_MOUNTED);
                    ESP_LOGI(TAG, "TF card appeared, now logging to run %u",
                             (unsigned)run);
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
