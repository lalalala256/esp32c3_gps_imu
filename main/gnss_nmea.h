/*
 * gnss_nmea.h - minimal NMEA 0183 parser.
 *
 * Supported sentences (any talker ID: GP, GN, GL, GA, GB, BD, ...):
 *   GGA - fix quality, satellites used, HDOP, altitude, geoid separation
 *   RMC - date, time, validity, position, speed, course
 *   GSA - fix mode (1 = none, 2 = 2D, 3 = 3D)
 *   GSV - satellites in view (fallback when GGA reports none)
 *   VTG - course and speed (fallback)
 *
 * RMC is used as the once-per-second epoch trigger, because it is the only
 * sentence carrying the full date and velocity. GGA normally precedes RMC in
 * the stream, so the fix quality fields are already merged by the time the
 * epoch fires.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "data_types.h"

typedef struct {
    /* time (UTC) */
    bool     utc_valid;
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
    uint16_t millisecond;

    /* position */
    bool   pos_valid;
    double lat_deg, lon_deg;
    bool   alt_valid;
    float  alt_msl_m, geoid_sep_m;

    /* quality */
    uint8_t fix_type;      /* GGA quality indicator */
    uint8_t fix_mode;      /* GSA fix mode */
    uint8_t sats_used;     /* GGA */
    uint8_t sats_view;     /* GSV */
    bool    hdop_valid;
    float   hdop;

    /* velocity */
    bool  speed_valid;
    float speed_mps;
    bool  course_valid;
    float course_deg;

    /* epoch bookkeeping */
    bool     have_epoch;
    uint32_t last_epoch_key;
} gnss_nmea_t;

typedef enum {
    GNSS_NMEA_INVALID = 0, /* not a supported/valid sentence */
    GNSS_NMEA_OK,          /* parsed, no new epoch */
    GNSS_NMEA_EPOCH,       /* parsed, a new UTC second is complete */
    GNSS_NMEA_STATUS,      /* RMC arrived but carries no UTC time: no fix yet */
} gnss_nmea_res_t;

void gnss_nmea_reset(gnss_nmea_t *st);

/*
 * Feed one NMEA sentence. `sentence` must start with '$'; the trailing CR/LF
 * may be present or absent. Sentences are copied internally, so the caller's
 * buffer is not modified.
 */
gnss_nmea_res_t gnss_nmea_feed(gnss_nmea_t *st, const char *sentence);

/* Copy the current merged state into a log record (t_us/PPS fields are zeroed
 * and must be filled in by the caller). */
void gnss_nmea_snapshot(const gnss_nmea_t *st, gnss_sample_t *out);
