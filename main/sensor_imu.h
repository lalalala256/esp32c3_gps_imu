/*
 * sensor_imu.h - BMI160 accelerometer + gyroscope acquisition.
 */
#pragma once

#include "esp_err.h"

/*
 * Configure the BMI160 and route its data-ready interrupt to INT1 (GPIO0).
 * Returns ESP_ERR_NOT_FOUND when the chip does not answer with the expected
 * chip ID, so the caller can continue in degraded mode.
 */
esp_err_t sensor_imu_init(void);

/* Create the acquisition task. Only call after sensor_imu_init() succeeded. */
void sensor_imu_start_task(void);
