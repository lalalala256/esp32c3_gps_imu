/*
 * pps_capture.h - GNSS pulse-per-second capture.
 */
#pragma once

#include "data_types.h"
#include "esp_err.h"

/*
 * Configure GPIO10 as a rising-edge interrupt and start feeding q_pps.
 * q_pps must already exist (it is drained by the GNSS task, which performs the
 * PPS <-> UTC pairing).
 */
esp_err_t pps_capture_init(void);
