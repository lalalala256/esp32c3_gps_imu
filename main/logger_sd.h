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
    uint64_t rows[LOGGER_FILE_COUNT];
    uint64_t bytes_written;
    uint32_t write_errors;
    uint32_t remounts;
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
