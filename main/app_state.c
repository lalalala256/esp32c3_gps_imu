/*
 * app_state.c - definitions of the shared queues, counters and the
 * mutex-protected GNSS/PPS snapshot used to feed the status task.
 */
#include "data_types.h"
#include "app_config.h"

QueueHandle_t q_imu;
QueueHandle_t q_mag;
QueueHandle_t q_gnss;
QueueHandle_t q_key;
QueueHandle_t q_pps;

app_stats_t g_stats;

SemaphoreHandle_t g_gnss_mutex;
gnss_sample_t     g_last_gnss;
pps_stats_t       g_pps;

SemaphoreHandle_t g_sensor_mutex;
imu_sample_t      g_last_imu;
mag_sample_t      g_last_mag;

void app_state_set_imu(const imu_sample_t *in)
{
    if (!in || !g_sensor_mutex) {
        return;
    }
    if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
        g_last_imu = *in;
        xSemaphoreGive(g_sensor_mutex);
    }
}

void app_state_set_mag(const mag_sample_t *in)
{
    if (!in || !g_sensor_mutex) {
        return;
    }
    if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
        g_last_mag = *in;
        xSemaphoreGive(g_sensor_mutex);
    }
}

void app_state_get_sensors(imu_sample_t *imu_out, mag_sample_t *mag_out)
{
    if (!g_sensor_mutex) {
        if (imu_out) {
            *imu_out = g_last_imu;
        }
        if (mag_out) {
            *mag_out = g_last_mag;
        }
        return;
    }
    if (xSemaphoreTake(g_sensor_mutex, portMAX_DELAY) == pdTRUE) {
        if (imu_out) {
            *imu_out = g_last_imu;
        }
        if (mag_out) {
            *mag_out = g_last_mag;
        }
        xSemaphoreGive(g_sensor_mutex);
    }
}

void app_state_get_gnss(gnss_sample_t *out, pps_stats_t *pps_out)
{
    if (!g_gnss_mutex) {
        if (out) {
            *out = g_last_gnss;
        }
        if (pps_out) {
            *pps_out = g_pps;
        }
        return;
    }
    if (xSemaphoreTake(g_gnss_mutex, portMAX_DELAY) == pdTRUE) {
        if (out) {
            *out = g_last_gnss;
        }
        if (pps_out) {
            *pps_out = g_pps;
        }
        xSemaphoreGive(g_gnss_mutex);
    }
}

void app_state_set_gnss(const gnss_sample_t *in)
{
    if (!in || !g_gnss_mutex) {
        return;
    }
    if (xSemaphoreTake(g_gnss_mutex, portMAX_DELAY) == pdTRUE) {
        g_last_gnss = *in;
        xSemaphoreGive(g_gnss_mutex);
    }
}

void app_state_set_pps(const pps_stats_t *in)
{
    if (!in || !g_gnss_mutex) {
        return;
    }
    if (xSemaphoreTake(g_gnss_mutex, portMAX_DELAY) == pdTRUE) {
        g_pps = *in;
        xSemaphoreGive(g_gnss_mutex);
    }
}

void app_event_log(int64_t t_us, log_source_t src, log_event_t evt)
{
    if (!q_key) {
        return;
    }
    key_event_t e = { .t_us = t_us, .src = (uint8_t)src, .event = (uint8_t)evt };
    if (xQueueSend(q_key, &e, 0) == pdPASS) {
        g_stats.key_events++;
    } else {
        g_stats.key_drops++;
    }
}
