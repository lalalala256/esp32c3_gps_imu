/*
 * gnss_nmea.c - see gnss_nmea.h.
 *
 * The parser is deliberately small and self-contained: it validates the NMEA
 * checksum, tokenises the sentence in place on a private copy, and updates a
 * "current state" structure field by field. Nothing is emitted until RMC closes
 * a UTC second, which keeps the output at a clean 1 Hz with all fields merged.
 */
#include "gnss_nmea.h"

#include <stdlib.h>
#include <string.h>

#include "app_config.h"

#define NMEA_MAX_FIELDS  24

/* Knots -> m/s */
#define KNOTS_TO_MPS     0.514444f

/* ------------------------------------------------------------------ helpers */

static bool checksum_ok(const char *s)
{
    const char *star = strchr(s, '*');
    if (!star) {
        /* Some receivers omit the checksum; accept rather than lose the fix. */
        return true;
    }
    if (star[1] == '\0' || star[2] == '\0') {
        return false;
    }

    uint8_t sum = 0;
    for (const char *p = s + 1; p < star; p++) {
        sum ^= (uint8_t)*p;
    }

    const char hex[3] = { star[1], star[2], '\0' };
    return (uint8_t)strtol(hex, NULL, 16) == sum;
}

/* Split on commas in place and drop the "*CS" suffix from the last field. */
static int split_fields(char *s, char *fields[], int max)
{
    int n = 0;
    char *p = s;

    while (n < max) {
        fields[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) {
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }

    if (n > 0) {
        char *star = strchr(fields[n - 1], '*');
        if (star) {
            *star = '\0';
        }
    }
    return n;
}

/* "hhmmss.sss" -> hour, minute, second, millisecond */
static bool parse_time(const char *f, uint8_t *h, uint8_t *m, uint8_t *s,
                       uint16_t *ms)
{
    if (!f || strlen(f) < 6) {
        return false;
    }

    char b[3] = { f[0], f[1], '\0' };
    *h = (uint8_t)atoi(b);
    b[0] = f[2]; b[1] = f[3];
    *m = (uint8_t)atoi(b);
    b[0] = f[4]; b[1] = f[5];
    *s = (uint8_t)atoi(b);

    *ms = 0;
    const char *dot = strchr(f, '.');
    if (dot && dot[1]) {
        /* Right-pad the fraction so ".5" becomes 500 ms, not 5 ms. */
        char frac[4] = { '0', '0', '0', '\0' };
        for (int i = 0; i < 3 && dot[1 + i]; i++) {
            frac[i] = dot[1 + i];
        }
        *ms = (uint16_t)atoi(frac);
    }
    return true;
}

/* "ddmmyy" -> 20yy-mm-dd */
static bool parse_date(const char *f, uint16_t *y, uint8_t *mo, uint8_t *d)
{
    if (!f || strlen(f) < 6) {
        return false;
    }

    char b[3] = { f[0], f[1], '\0' };
    *d = (uint8_t)atoi(b);
    b[0] = f[2]; b[1] = f[3];
    *mo = (uint8_t)atoi(b);
    b[0] = f[4]; b[1] = f[5];
    *y = (uint16_t)(2000 + atoi(b));
    return true;
}

/* "4807.038" -> 48.1173 degrees (ddmm.mmmm / dddmm.mmmm) */
static bool parse_coord(const char *f, double *deg)
{
    if (!f || !*f) {
        return false;
    }

    const char *dot = strchr(f, '.');
    const size_t int_len = dot ? (size_t)(dot - f) : strlen(f);
    if (int_len < 3 || int_len > 5) {
        return false;
    }

    const size_t deg_digits = int_len - 2;

    char deg_buf[4];
    memcpy(deg_buf, f, deg_digits);
    deg_buf[deg_digits] = '\0';

    const int d = atoi(deg_buf);
    const double minutes = atof(f + deg_digits);

    *deg = (double)d + minutes / 60.0;
    return true;
}

/* ------------------------------------------------------------- sentences */

static void handle_gga(gnss_nmea_t *st, char *f[], int n)
{
    uint8_t h, m, s;
    uint16_t ms;

    if (n > 1 && parse_time(f[1], &h, &m, &s, &ms)) {
        st->utc_valid = true;
        st->hour = h;
        st->minute = m;
        st->second = s;
        st->millisecond = ms;
    }

    if (n > 5 && f[2][0] && f[4][0]) {
        double lat, lon;
        if (parse_coord(f[2], &lat) && parse_coord(f[4], &lon)) {
            if (f[3][0] == 'S') lat = -lat;
            if (f[5][0] == 'W') lon = -lon;
            st->lat_deg = lat;
            st->lon_deg = lon;
            st->pos_valid = true;
        }
    }

    if (n > 6 && f[6][0]) {
        st->fix_type = (uint8_t)atoi(f[6]);
    }
    if (n > 7 && f[7][0]) {
        st->sats_used = (uint8_t)atoi(f[7]);
    }
    if (n > 8 && f[8][0]) {
        st->hdop = (float)atof(f[8]);
        st->hdop_valid = true;
    }
    if (n > 9 && f[9][0]) {
        st->alt_msl_m = (float)atof(f[9]);
        st->geoid_sep_m = (n > 11 && f[11][0]) ? (float)atof(f[11]) : 0.0f;
        st->alt_valid = true;
    }
}

static gnss_nmea_res_t handle_rmc(gnss_nmea_t *st, char *f[], int n)
{
    uint8_t h, m, s;
    uint16_t ms;
    bool time_ok = false;

    if (n > 1 && parse_time(f[1], &h, &m, &s, &ms)) {
        st->utc_valid = true;
        st->hour = h;
        st->minute = m;
        st->second = s;
        st->millisecond = ms;
        time_ok = true;
    }

    if (n > 9 && f[9][0]) {
        parse_date(f[9], &st->year, &st->month, &st->day);
    }

    /* Field 2 is 'A' (valid) or 'V' (void). */
    const bool fix_ok = (n > 2 && f[2][0] == 'A');

    if (fix_ok && n > 6 && f[3][0] && f[5][0]) {
        double lat, lon;
        if (parse_coord(f[3], &lat) && parse_coord(f[5], &lon)) {
            if (f[4][0] == 'S') lat = -lat;
            if (f[6][0] == 'W') lon = -lon;
            st->lat_deg = lat;
            st->lon_deg = lon;
            st->pos_valid = true;
        }
    } else if (!fix_ok) {
        st->pos_valid = false;
    }

    if (n > 7 && f[7][0]) {
        st->speed_mps = (float)atof(f[7]) * KNOTS_TO_MPS;
        st->speed_valid = true;
    }
    if (n > 8 && f[8][0]) {
        st->course_deg = (float)atof(f[8]);
        st->course_valid = true;
    }

    if (!time_ok) {
        return GNSS_NMEA_INVALID;
    }

    const uint32_t key = (uint32_t)st->hour * 3600u +
                         (uint32_t)st->minute * 60u +
                         (uint32_t)st->second;

    if (!st->have_epoch || key != st->last_epoch_key) {
        st->have_epoch = true;
        st->last_epoch_key = key;
        return GNSS_NMEA_EPOCH;
    }
    return GNSS_NMEA_OK;
}

static void handle_gsa(gnss_nmea_t *st, char *f[], int n)
{
    if (n > 2 && f[2][0]) {
        st->fix_mode = (uint8_t)atoi(f[2]);
    }
}

static void handle_gsv(gnss_nmea_t *st, char *f[], int n)
{
    /* Satellite count is only taken from the first GSV message of a group. */
    const bool first_message = (n > 2 && f[2][0]) ? (atoi(f[2]) == 1) : true;
    if (n > 3 && f[3][0] && first_message) {
        st->sats_view = (uint8_t)atoi(f[3]);
    }
}

static void handle_vtg(gnss_nmea_t *st, char *f[], int n)
{
    /* Field 1: course over ground, true. Field 7: speed in km/h. */
    if (n > 1 && f[1][0]) {
        st->course_deg = (float)atof(f[1]);
        st->course_valid = true;
    }
    if (n > 7 && f[7][0]) {
        st->speed_mps = (float)atof(f[7]) / 3.6f;
        st->speed_valid = true;
    }
}

/* -------------------------------------------------------------------- API */

void gnss_nmea_reset(gnss_nmea_t *st)
{
    if (st) {
        memset(st, 0, sizeof(*st));
    }
}

gnss_nmea_res_t gnss_nmea_feed(gnss_nmea_t *st, const char *sentence)
{
    if (!st || !sentence || sentence[0] != '$') {
        return GNSS_NMEA_INVALID;
    }
    if (!checksum_ok(sentence)) {
        return GNSS_NMEA_INVALID;
    }

    char buf[APP_GNSS_LINE_BYTES];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *f[NMEA_MAX_FIELDS];
    const int n = split_fields(buf, f, NMEA_MAX_FIELDS);
    if (n < 1 || strlen(f[0]) < 6) {
        return GNSS_NMEA_INVALID;
    }

    const char *type = f[0] + 3;   /* skip "$tt" talker ID */

    if (strncmp(type, "GGA", 3) == 0) {
        handle_gga(st, f, n);
        return GNSS_NMEA_OK;
    }
    if (strncmp(type, "RMC", 3) == 0) {
        return handle_rmc(st, f, n);
    }
    if (strncmp(type, "GSA", 3) == 0) {
        handle_gsa(st, f, n);
        return GNSS_NMEA_OK;
    }
    if (strncmp(type, "GSV", 3) == 0) {
        handle_gsv(st, f, n);
        return GNSS_NMEA_OK;
    }
    if (strncmp(type, "VTG", 3) == 0) {
        handle_vtg(st, f, n);
        return GNSS_NMEA_OK;
    }

    return GNSS_NMEA_INVALID;
}

void gnss_nmea_snapshot(const gnss_nmea_t *st, gnss_sample_t *out)
{
    memset(out, 0, sizeof(*out));

    out->utc_valid   = st->utc_valid;
    out->year        = st->year;
    out->month       = st->month;
    out->day         = st->day;
    out->hour        = st->hour;
    out->minute      = st->minute;
    out->second      = st->second;
    out->millisecond = st->millisecond;

    out->pos_valid    = st->pos_valid;
    out->lat_deg      = st->lat_deg;
    out->lon_deg      = st->lon_deg;
    out->alt_msl_m    = st->alt_msl_m;
    out->geoid_sep_m  = st->geoid_sep_m;

    out->fix_type = st->fix_type;
    out->fix_mode = st->fix_mode;
    out->sats     = st->sats_used ? st->sats_used : st->sats_view;
    out->hdop     = st->hdop;

    out->speed_mps  = st->speed_mps;
    out->course_deg = st->course_deg;
}
