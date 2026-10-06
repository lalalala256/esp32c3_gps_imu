/*
 * status.h - periodic health/status line.
 */
#pragma once

#include "esp_err.h"

esp_err_t status_init(void);
void status_start_task(void);
