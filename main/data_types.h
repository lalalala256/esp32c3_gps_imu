/*
 * data_types.h - the record types exchanged between the sensor producers and
 * the logging/status consumers.
 *
 * Time base
 * ---------
 * Every record carries `t_us`: a 64-bit microsecond timestamp from
 * esp_timer_get_time() (monotonic since boot, 1 us resolution). It is captured
 * as close to the physical event as possible:
 *   - IMU: inside the BMI160 INT1 data-ready ISR.
 *   - Magnetometer / GNSS: at the moment the sample is read or parsed.
 * GNSS records additionally carry the absolute UTC that the receiver reported,
 * which is what later lets an INS/GNSS fusion stage anchor `t_us` to UTC.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* ------------------------------------------------------------- IMU sample */
typedef struct {
    int64_t t_us;       /* timestamp captured in the INT1 ISR */
    float   ax, ay, az; /* acceleration, g */
    float   gx, gy, gz; /* angular rate, deg/s */
    bool    valid;      /* false when the read failed */
} imu_sample_t;

/* ------------------------------------------------------ magnetometer sample */
typedef struct {
    int64_t t_us;       /* timestamp captured at read time */
    float   mx, my, mz; /* magnetic field, milligauss */
    bool    valid;
} mag_sample_t;

/* ------------------------------------------------------------- GNSS sample */
typedef struct {
    int64_t  t_us;        /* when the epoch sentence was parsed */
    int64_t  pps_t_us;    /* paired PPS edge timestamp, 0 when unpaired */
    uint32_t pps_seq;     /* PPS edge sequence number, 0 when unpaired */

    bool     utc_valid;
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
    uint16_t millisecond; /* sub-second part of the reported fix time */

    bool     pos_valid;
    double   lat_deg, lon_deg;
    float    alt_msl_m, geoid_sep_m;

    float    speed_mps, course_deg;

    uint8_t  fix_type;    /* GGA quality: 0 none, 1 GPS, 2 DGPS, ... */
    uint8_t  fix_mode;    /* GSA: 1 no fix, 2 2D, 3 3D */
    uint8_t  sats;        /* satellites used / in view */
    float    hdop;
} gnss_sample_t;

/* ------------------------------------------------------------ PPS tracking */
typedef struct {
    uint32_t seq;              /* edges seen since boot */
    int64_t  last_t_us;        /* timestamp of the most recent edge */
    uint32_t last_interval_us; /* spacing between the last two edges */
    uint32_t min_interval_us;
    uint32_t max_interval_us;
} pps_stats_t;

typedef struct {
    int64_t  t_us;
    uint32_t seq;
} pps_event_t;

/* -------------------------------------------------------- key / system log */
typedef enum {
    EVT_BOOT = 0,
    EVT_LOG_START,
    EVT_LOG_STOP,
    EVT_MARK,
    EVT_KEY1_PRESS,
    EVT_KEY2_PRESS,
    EVT_SD_MOUNTED,
    EVT_SD_MOUNT_FAIL,
    EVT_SD_ERROR,
    EVT_SD_REMOUNT,
    EVT_SENSOR_MISSING,
    /* Appended rather than inserted: this enum is written into the event log,
     * so the existing values must keep their numbers. */
    EVT_SESSION_SYNC,
} log_event_t;

typedef enum {
    SRC_SYS = 0,
    SRC_KEY1,
    SRC_KEY2,
} log_source_t;

typedef struct {
    int64_t t_us;
    uint8_t src;    /* log_source_t */
    uint8_t event;  /* log_event_t */
} key_event_t;

/* ---------------------------------------------------------------- counters */
typedef struct {
    volatile uint32_t imu_samples;
    volatile uint32_t mag_samples;
    volatile uint32_t gnss_samples;
    volatile uint32_t key_events;
    volatile uint32_t imu_drops;
    volatile uint32_t mag_drops;
    volatile uint32_t gnss_drops;
    volatile uint32_t key_drops;
    volatile uint32_t pps_edges;
    volatile uint32_t imu_errors;
    volatile uint32_t mag_errors;
} app_stats_t;

/* --------------------------------------------------- shared pipeline state */
extern QueueHandle_t q_imu;
extern QueueHandle_t q_mag;
extern QueueHandle_t q_gnss;
extern QueueHandle_t q_key;
extern QueueHandle_t q_pps;

extern app_stats_t g_stats;

/* Guards g_last_gnss and g_pps: written by the GNSS task, read by status. */
extern SemaphoreHandle_t g_gnss_mutex;
extern gnss_sample_t     g_last_gnss;
extern pps_stats_t       g_pps;

/* Snapshot helpers (copy under g_gnss_mutex). */
void app_state_get_gnss(gnss_sample_t *out, pps_stats_t *pps_out);
void app_state_set_gnss(const gnss_sample_t *in);
void app_state_set_pps(const pps_stats_t *in);

/*
 * Latest raw sensor sample, for the web live view.
 *
 * Written by the IMU and magnetometer tasks, read by the HTTP handlers. It has
 * its own mutex rather than sharing g_gnss_mutex so that a slow (or stalled)
 * browser can never delay the GNSS path, which carries the timing information.
 */
extern SemaphoreHandle_t g_sensor_mutex;
extern imu_sample_t      g_last_imu;
extern mag_sample_t      g_last_mag;

void app_state_set_imu(const imu_sample_t *in);
void app_state_set_mag(const mag_sample_t *in);
void app_state_get_sensors(imu_sample_t *imu_out, mag_sample_t *mag_out);

/* Queue a log event; never blocks the caller (drops and counts on overflow). */
void app_event_log(int64_t t_us, log_source_t src, log_event_t evt);
