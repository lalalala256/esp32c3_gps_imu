/*
 * sensor_mag.h - QMC5883L 3-axis magnetometer acquisition.
 */
#pragma once

#include "esp_err.h"

/* Configure the QMC5883L for continuous conversion. */
esp_err_t sensor_mag_init(void);

/* Create the acquisition task. Only call after sensor_mag_init() succeeded. */
void sensor_mag_start_task(void);
