/*
 * log_format.h - pure CSV row formatters.
 *
 * These functions perform no I/O so they can be unit-tested on the host and
 * reused unchanged if the transport (SD / serial / network) ever changes.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "data_types.h"

/* Column headers written once beneath each file's '#' metadata line. */
extern const char LOG_HEADER_IMU[];
extern const char LOG_HEADER_MAG[];
extern const char LOG_HEADER_GNSS[];
extern const char LOG_HEADER_EVENT[];

/*
 * Each formatter writes one complete CSV row (no trailing newline) into buf and
 * returns the number of characters written, or -1 when buf is too small.
 */
int log_fmt_imu(char *buf, size_t n, const imu_sample_t *s);
int log_fmt_mag(char *buf, size_t n, const mag_sample_t *s);
int log_fmt_gnss(char *buf, size_t n, const gnss_sample_t *s);
int log_fmt_event(char *buf, size_t n, const key_event_t *s);

const char *log_event_name(uint8_t evt);
const char *log_source_name(uint8_t src);
