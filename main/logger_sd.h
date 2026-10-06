/*
 * logger_sd.h - CSV logging to the TF card, with a serial mirror.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    LOGGER_FILE_IMU = 0,
    LOGGER_FILE_MAG,
    LOGGER_FILE_GNSS,
    LOGGER_FILE_EVENT,
    LOGGER_FILE_COUNT,
} logger_file_t;

typedef struct {
    bool     sd_mounted;
    bool     logging;
    uint32_t run_index;
    uint32_t spi_khz;
    uint64_t rows[LOGGER_FILE_COUNT];
    uint64_t bytes_written;
    uint32_t write_errors;
    uint32_t remounts;
    /* Raw-probe history: proves the card drove MISO at least once, which
     * separates "nothing is wired up" from "the card is there but the link
     * is unreliable". The status LED uses this to pick its blink pattern. */
    bool     card_seen;
    uint32_t probe_hits;
} logger_status_t;

/*
 * Mount the TF card, pick the next run index and open one CSV per source.
 * A missing or unusable card is not fatal: the logger degrades to
 * serial-only and reports it through the status snapshot.
 */
esp_err_t logger_init(void);

/* Create the logger task (call after logger_init and after the queues exist). */
void logger_start_task(void);

void logger_get_status(logger_status_t *out);
bool logger_is_logging(void);

/* True while a card is mounted and the run files are open. */
bool logger_sd_mounted(void);

/*
 * Serialises TF card access between the logger task and other users (the web
 * file browser). The mutex is recursive, so nesting is safe. Callers outside
 * the logger should pass a finite timeout and bail out if it is not granted,
 * so a stalled card cannot wedge a request handler.
 */
bool sd_fs_lock(uint32_t timeout_ms);
void sd_fs_unlock(void);
