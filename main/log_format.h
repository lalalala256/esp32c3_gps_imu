/*
 * log_format.h - pure CSV row formatters.
 *
 * These functions perform no I/O so they can be unit-tested on the host and
 * reused unchanged if the transport (SD / serial / network) ever changes.
 *
 * Every row begins with the same two time columns:
 *
 *   t_us     microseconds from esp_timer_get_time(), monotonic since boot.
 *            This is the only clock that is continuous and identical across all
 *            four files, so it is what a fusion step should join and
 *            interpolate on.
 *   utc_us   microseconds since the Unix epoch, or 0 while the receiver has not
 *            supplied a time. It is derived from t_us through the anchor the
 *            logger takes from each GNSS epoch, so it keeps the microsecond
 *            resolution of the local clock rather than the receiver's 1 Hz -
 *            and it is what makes a row placeable on a map.
 *
 *            Note that only the GNSS rows land exactly on a second boundary. A
 *            row on another stream is interpolated from the most recent epoch,
 *            and consecutive epochs do disagree slightly about the length of a
 *            second, so expect up to about a millisecond of movement in utc_us
 *            between anchors. That is the receiver's resolution, not a bug in
 *            the arithmetic.
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
 * Per-row information that is not part of the sample itself.
 *
 * The position is the last one the receiver reported. Carrying it in the event
 * rows means a KEY2 mark can be plotted straight from that file, without having
 * to join it against the GNSS file first.
 */
typedef struct {
    int64_t utc_us;
    double  lat_deg;
    double  lon_deg;
    bool    pos_valid;
} log_ctx_t;

/*
 * Each formatter writes one complete CSV row (no trailing newline) into buf and
 * returns the number of characters written, or -1 when buf is too small.
 */
int log_fmt_imu(char *buf, size_t n, const imu_sample_t *s, const log_ctx_t *c);
int log_fmt_mag(char *buf, size_t n, const mag_sample_t *s, const log_ctx_t *c);
int log_fmt_gnss(char *buf, size_t n, const gnss_sample_t *s, const log_ctx_t *c);
int log_fmt_event(char *buf, size_t n, const key_event_t *s, const log_ctx_t *c);

const char *log_event_name(uint8_t evt);
const char *log_source_name(uint8_t src);
