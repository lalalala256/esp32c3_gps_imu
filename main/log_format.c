#include "log_format.h"

#include <stdio.h>

const char LOG_HEADER_IMU[]   = "t_us,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,valid";
const char LOG_HEADER_MAG[]   = "t_us,mx_mG,my_mG,mz_mG,valid";
const char LOG_HEADER_GNSS[]  = "t_us,pps_t_us,pps_seq,utc_date,utc_time,"
                                "lat_deg,lon_deg,alt_msl_m,geoid_sep_m,"
                                "speed_mps,course_deg,fix_type,fix_mode,sats,"
                                "hdop,valid";
const char LOG_HEADER_EVENT[] = "t_us,src,event";

/*
 * Truncation-safe snprintf wrapper: returns the number of characters that were
 * actually stored, or -1 when the output did not fit.
 */
static int finish(char *buf, size_t n, int want)
{
    if (want < 0 || (size_t)want >= n) {
        if (n) {
            buf[n - 1] = '\0';
        }
        return -1;
    }
    return want;
}

int log_fmt_imu(char *buf, size_t n, const imu_sample_t *s)
{
    return finish(buf, n, snprintf(buf, n,
        "%lld,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d",
        (long long)s->t_us,
        s->ax, s->ay, s->az,
        s->gx, s->gy, s->gz,
        s->valid ? 1 : 0));
}

int log_fmt_mag(char *buf, size_t n, const mag_sample_t *s)
{
    return finish(buf, n, snprintf(buf, n,
        "%lld,%.2f,%.2f,%.2f,%d",
        (long long)s->t_us,
        s->mx, s->my, s->mz,
        s->valid ? 1 : 0));
}

int log_fmt_gnss(char *buf, size_t n, const gnss_sample_t *s)
{
    char date[16] = "";
    char time[24] = "";

    if (s->utc_valid) {
        snprintf(date, sizeof(date), "%04u-%02u-%02u",
                 (unsigned)s->year, (unsigned)s->month, (unsigned)s->day);
        snprintf(time, sizeof(time), "%02u:%02u:%02u.%03u",
                 (unsigned)s->hour, (unsigned)s->minute, (unsigned)s->second,
                 (unsigned)s->millisecond);
    }

    const int valid = (s->utc_valid || s->pos_valid) ? 1 : 0;

    return finish(buf, n, snprintf(buf, n,
        "%lld,%lld,%u,%s,%s,%.7f,%.7f,%.3f,%.3f,%.3f,%.2f,%u,%u,%u,%.2f,%d",
        (long long)s->t_us,
        (long long)s->pps_t_us,
        (unsigned)s->pps_seq,
        date, time,
        s->lat_deg, s->lon_deg,
        s->alt_msl_m, s->geoid_sep_m,
        s->speed_mps, s->course_deg,
        (unsigned)s->fix_type, (unsigned)s->fix_mode, (unsigned)s->sats,
        s->hdop,
        valid));
}

int log_fmt_event(char *buf, size_t n, const key_event_t *s)
{
    return finish(buf, n, snprintf(buf, n,
        "%lld,%s,%s",
        (long long)s->t_us,
        log_source_name(s->src),
        log_event_name(s->event)));
}

const char *log_source_name(uint8_t src)
{
    switch (src) {
    case SRC_KEY1: return "KEY1";
    case SRC_KEY2: return "KEY2";
    default:       return "SYS";
    }
}

const char *log_event_name(uint8_t evt)
{
    switch (evt) {
    case EVT_BOOT:            return "BOOT";
    case EVT_LOG_START:       return "LOG_START";
    case EVT_LOG_STOP:        return "LOG_STOP";
    case EVT_MARK:            return "MARK";
    case EVT_KEY1_PRESS:      return "KEY1_PRESS";
    case EVT_KEY2_PRESS:      return "KEY2_PRESS";
    case EVT_SD_MOUNTED:      return "SD_MOUNTED";
    case EVT_SD_MOUNT_FAIL:   return "SD_MOUNT_FAIL";
    case EVT_SD_ERROR:        return "SD_ERROR";
    case EVT_SD_REMOUNT:      return "SD_REMOUNT";
    case EVT_SENSOR_MISSING:  return "SENSOR_MISSING";
    default:                  return "UNKNOWN";
    }
}
